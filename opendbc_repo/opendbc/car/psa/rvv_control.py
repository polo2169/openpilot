"""Prepare a T9 RVV replacement from physical CAN and native CarControl.

This is the host part of the integration. A candidate is diagnostic data,
never a sendcan message. The installed Panda policy does not allow 0x50E.
Releasing interception would restore the stock (possibly higher) setpoint;
it is NOT a validated way to cancel cruise or brake the vehicle.
"""

from dataclasses import asdict, dataclass, replace
import json
import math

from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.psa.lka import fresh
from opendbc.car.psa.rvv import CONTROL_TIMEOUT_NS, MAX_SETPOINT_KPH, MIN_SETPOINT_KPH, RvvInputs, T9RvvAdapter


SETTLE_NS = 2_000_000_000
TEMPLATE_AGE_NS = 50_000_000


def setpoint_parity(value: int) -> int:
  if type(value) is not int or not 0 <= value <= 255:
    raise ValueError('invalid_setpoint_byte')
  return ((value >> 4).bit_count() % 2) * 2 + ((value & 15).bit_count() % 2)


@dataclass(frozen=True)
class RvvStock:
  data: bytes
  setpoint_kph: int
  mode: int
  activation: bool
  counter: int


def decode_stock(data: bytes) -> RvvStock:
  if len(data) != 8:
    raise ValueError('stock_dlc_invalid')
  if (data[0] >> 4) & 3 != setpoint_parity(data[6]):
    raise ValueError('stock_parity_invalid')
  return RvvStock(bytes(data), data[6], (data[7] >> 5) & 3, bool(data[7] & 0x80), data[7] & 15)


def replacement_payload(template: bytes, setpoint_kph: int) -> bytes:
  """Preserve the stock counter, activation, mode and every opaque bit."""
  stock = decode_stock(template)
  if stock.mode != 1 or not stock.activation:
    raise ValueError('stock_rvv_not_active')
  if not MIN_SETPOINT_KPH <= stock.setpoint_kph <= MAX_SETPOINT_KPH:
    raise ValueError('stock_setpoint_out_of_range')
  if type(setpoint_kph) is not int or not MIN_SETPOINT_KPH <= setpoint_kph <= stock.setpoint_kph:
    raise ValueError('replacement_outside_driver_ceiling')
  data = bytearray(stock.data)
  data[6] = setpoint_kph
  data[0] = (data[0] & 0xCF) | (setpoint_parity(setpoint_kph) << 4)
  return bytes(data)


class T9RvvControl:
  """Native host integration, evaluated without authorizing CAN emission.

  Use physical RX before CarState merges buses; bus 128/130 are TX echoes,
  never evidence of another physical source. A fresh RVV off observation is
  required before a first request and after every interruption. Within an
  episode a generic CarControl request cannot increase the candidate. The
  replay/bench join can recover gradually from a fresh following decision,
  bounded by the physical driver setpoint. Lead loss never grants recovery.
  Physical restoration of the original setpoint remains unvalidated.
  """

  def __init__(self):
    self.adapter = T9RvvAdapter()
    self.stock = None
    self.stock_ns = self.engine_ns = 0
    self.engine_active = False
    self.engine_state = None
    self.first_ns = self.wrong_side_ns = 0
    self.stock_problem = 'stock_missing'
    self.last_clock = self.consumed_ns = 0
    self.saw_inactive = self.started = False
    self.blocked_reason = None
    self.ceiling = None
    self.candidate = None
    self.status = {}
    self._log_key = None
    self._log_ns = 0

  def observe(self, packets):
    for timestamp, frames in packets:
      if type(timestamp) is not int or timestamp <= 0:
        continue
      for address, data, bus in frames:
        if bus not in (0, 2) or address not in (0x50E, 0x208):
          continue
        if not self.first_ns:
          self.first_ns = timestamp
        if (address == 0x50E and bus == 0) or (address == 0x208 and bus == 2):
          self.wrong_side_ns = max(self.wrong_side_ns, timestamp)
          continue
        if address == 0x208:
          if timestamp <= self.engine_ns or len(data) != 8:
            self.engine_ns = 0
          else:
            self.engine_ns = timestamp
            self.engine_state = (data[4] >> 2) & 3
            self.engine_active = self.engine_state == 2
        else:
          previous, previous_ns = self.stock, self.stock_ns
          self.stock = None
          self.stock_ns = timestamp
          try:
            if timestamp <= previous_ns:
              raise ValueError('stock_time_not_progressing')
            stock = decode_stock(data)
            if previous is not None and timestamp - previous_ns <= 250_000_000 and stock.counter == previous.counter:
              raise ValueError('stock_counter_not_progressing')
            self.stock = stock
            self.stock_problem = None
          except ValueError as exc:
            self.stock_problem = str(exc)

  def readiness_problem(self, now, inputs):
    if self.stock_problem:
      return self.stock_problem
    if not fresh(now, self.stock_ns):
      return 'stock_template_stale'
    if not fresh(now, self.engine_ns):
      return 'engine_feedback_stale'
    if not self.first_ns or now - max(self.first_ns, self.wrong_side_ns) < SETTLE_NS:
      return 'bus_separation_not_observed'
    if inputs.cruise_active != self.engine_active:
      return 'engine_state_disagrees'
    if self.stock.mode != 1 or not self.stock.activation:
      return 'stock_rvv_not_active'
    if not math.isfinite(inputs.stock_setpoint_kph) or abs(inputs.stock_setpoint_kph - self.stock.setpoint_kph) > 0.01:
      return 'stock_setpoint_disagrees'
    return self.adapter.problem(now, inputs)

  def physical_off_observed(self, now, *, can_valid, safety_rx_nanos, cruise_active):
    """Fresh physical cancellation, not accelerator override or software off."""
    return (type(now) is int and now > 0 and now >= self.last_clock
            and can_valid and fresh(now, safety_rx_nanos)
            and fresh(now, self.engine_ns) and self.engine_state in (0, 3)
            and not cruise_active and self.stock_problem is None
            and self.stock is not None and fresh(now, self.stock_ns)
            and self.stock.mode == 1 and not self.stock.activation and self.stock.setpoint_kph == 255)

  def _update(self, now, inputs, *, candidate_increase_permitted=False):
    self.candidate = None
    valid_clock = type(now) is int and now > 0 and now >= self.last_clock
    problem = self.readiness_problem(now, inputs) if valid_clock else 'invalid_clock'
    if valid_clock:
      self.last_clock = now
    # An inactive request from software alone cannot re-arm the session.
    # Recorded state 2 -> 1 can mean accelerator override, while the BSI still
    # requests RVV at the original setpoint. It is not an explicit rearm.
    # Require the observed stock-off combination on both physical sources.
    physical_off = valid_clock and self.physical_off_observed(now, can_valid=inputs.can_valid,
      safety_rx_nanos=inputs.safety_rx_nanos, cruise_active=inputs.cruise_active)
    if physical_off:
      self.saw_inactive = True
      self.started = False
      self.blocked_reason = None
      self.ceiling = None
      self.adapter = T9RvvAdapter()

    phase, reason = 'disabled', 'not_requested'
    decision = None
    if inputs.requested:
      reason = self.blocked_reason or problem
      if reason is None and not self.started and not self.saw_inactive:
        reason = 'physical_rvv_rearm_required'
      if reason is None:
        self.started = True
        self.saw_inactive = False
        # readiness_problem checked the fresh physical CAN setpoint against
        # CarState. Keep its integer value, not a float32 m/s round-trip floor.
        cap = self.stock.setpoint_kph
        if self.ceiling is not None and not candidate_increase_permitted:
          cap = min(cap, self.ceiling)
        target = min(inputs.target_setpoint_kph, cap) if inputs.target_setpoint_kph is not None else None
        decision = self.adapter.update(now, replace(inputs, stock_setpoint_kph=cap, target_setpoint_kph=target))
        if decision.phase != 'tracking':
          reason = decision.reason
        else:
          self.ceiling = decision.proposed_setpoint_kph
          phase, reason = 'prepared', 'waiting_for_stock_frame'
          if self.stock_ns > self.consumed_ns:
            self.consumed_ns = self.stock_ns
            if fresh(now, self.stock_ns, TEMPLATE_AGE_NS):
              data = replacement_payload(self.stock.data, self.ceiling)
              self.candidate = {'address': 0x50E, 'bus': 0, 'data_hex': data.hex(),
                                'stock_rx_ns': self.stock_ns, 'counter': self.stock.counter,
                                'setpoint_kph': self.ceiling}
              reason = 'candidate_only_panda_rvv_not_integrated'
            else:
              reason = 'stock_frame_too_old_to_replace'
      if phase != 'prepared':
        self.blocked_reason = reason
        phase = 'blocked'
    elif self.started or self.blocked_reason is not None:
      self.blocked_reason = self.blocked_reason or 'longitudinal_request_removed'
      phase, reason = 'blocked', self.blocked_reason

    self.status = {
      'mode': 'rvv_prepare_only', 'phase': phase, 'reason': reason, 'tx_allowed': False,
      'readiness_problem': problem, 'rearm_required': self.blocked_reason is not None or not (self.saw_inactive or self.started),
      'stock_setpoint_kph': self.stock.setpoint_kph if self.stock else None,
      'stock_data_hex': self.stock.data.hex() if self.stock else None,
      'stock_counter': self.stock.counter if self.stock else None,
      'stock_rx_ns': self.stock_ns, 'engine_rx_ns': self.engine_ns,
      'engine_state_raw': self.engine_state, 'physical_off_observed': physical_off,
      'candidate_setpoint_kph': self.ceiling if phase == 'prepared' else None,
      'candidate': self.candidate, 'adapter': asdict(decision) if decision else None,
      'service_brake_required': reason == 'service_brake_required',
      'automatic_increase_allowed': False,
      'candidate_increase_permitted': phase == 'prepared' and candidate_increase_permitted,
      'physical_cruise_cancellation_validated': False,
    }
    return self.status

  def update(self, now, inputs):
    return self._update(now, inputs)

  def update_following(self, now, inputs, following):
    """Replay/bench join; no fabricated CarControl engagement or CAN output."""
    waiting = following.reason in ('waiting_for_lead', 'waiting_for_confident_lead', 'stock_setpoint_sufficient')
    if waiting and not self.started and self.blocked_reason is None:
      inputs = replace(inputs, requested=False)
    elif inputs.requested and following.target_kph is None:
      self.blocked_reason = self.blocked_reason or following.reason
    candidate_increase_permitted = False
    if following.target_kph is not None:
      if following.rearm_required or following.driver_intervention_required or following.reason != 'vision_following_candidate':
        self.blocked_reason = self.blocked_reason or 'following_decision_inconsistent'
      decision_fresh = (fresh(now, following.computed_ns, CONTROL_TIMEOUT_NS)
                        and fresh(now, following.lead_rx_ns) and fresh(now, following.model_ns, 500_000_000))
      if inputs.requested and not decision_fresh:
        self.blocked_reason = self.blocked_reason or 'following_decision_stale'
      candidate_increase_permitted = (decision_fresh and following.candidate_increase_permitted
        and following.reason == 'vision_following_candidate'
        and not following.rearm_required and not following.driver_intervention_required)
      inputs = replace(inputs, requested_accel_ms2=following.requested_accel_ms2,
                       target_setpoint_kph=following.target_kph)
    status = self._update(now, inputs, candidate_increase_permitted=candidate_increase_permitted)
    status['following'] = asdict(following)
    return status

  def update_from_carcontrol(self, CC, CS, now):
    state = CS.out
    inputs = RvvInputs(
      requested=bool(CC.enabled and CC.longActive), requested_accel_ms2=float(CC.actuators.accel),
      speed_kph=float(state.vEgoRaw) * 3.6, stock_setpoint_kph=float(state.cruiseState.speed) * 3.6,
      cruise_active=bool(state.cruiseState.enabled), cruise_available=bool(state.cruiseState.available),
      brake_pressed=bool(state.brakePressed), gas_pressed=bool(state.gasPressed), cancel=bool(CC.cruiseControl.cancel),
      vehicle_ready=bool(state.gearShifter == structs.CarState.GearShifter.drive
                         and not state.parkingBrake and not state.doorOpen and not state.seatbeltUnlatched),
      can_valid=bool(state.canValid and not state.canTimeout and getattr(CS, 't9_pedal_valid', False)),
      safety_rx_nanos=getattr(CS, 't9_shadow_safety_rx_nanos', 0),
      stock_rx_nanos=getattr(CS, 't9_shadow_rvv_rx_nanos', 0),
    )
    self.update(now, inputs)
    key = (self.status['phase'], self.status['reason'], self.status['readiness_problem'],
           self.status['candidate_setpoint_kph'], self.status['rearm_required'])
    if key != self._log_key or now - self._log_ns >= 1_000_000_000:
      # Ignore cadence-only changes, but retain numerical decisions and faults.
      if key[1] in ('waiting_for_stock_frame', 'candidate_only_panda_rvv_not_integrated'):
        key = (key[0], 'prepared', *key[2:])
      if key != self._log_key or now - self._log_ns >= 1_000_000_000:
        self._log_key, self._log_ns = key, now
        snapshot = self.status | {'mono_ns': now, 'inputs': asdict(inputs)}
        def clean(value):
          if isinstance(value, dict):
            return {k: clean(v) for k, v in value.items()}
          return None if isinstance(value, float) and not math.isfinite(value) else value
        try:
          carlog.info('psa_t9_rvv ' + json.dumps(clean(snapshot), separators=(',', ':'), allow_nan=False))
        except Exception:
          pass  # Logging cannot change command behavior.
