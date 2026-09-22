"""Dedicated, bounded T9 lateral development controller.

Only 0x3F2 may be queued, to bus 0, after observing isolation of bus 0/2.
An independent Panda C policy checks every frame before physical emission.
"""
from dataclasses import asdict
from collections import deque
import json
import math

from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.psa.lka import ACTIVE_TORQUE_LIMIT, LkaInputs, LkaPhase, T9LkaLifecycle, fresh
from opendbc.car.psa.lka_feedback import T9LkaCanObserver
from opendbc.car.psa.rvv_wire import COMBINED_SAFETY_PARAM, SPLIT_SAFETY_PARAM, EPS_CYCLE_SAFETY_PARAM

SAFETY_PARAM = 0x1308
TORQUE_SCALE = ACTIVE_TORQUE_LIMIT
PERIOD_NS = 50_000_000
MIN_PERIOD_NS = 45_000_000


def enabled(CP):
  return (not CP.dashcamOnly and not CP.openpilotLongitudinalControl and
          len(CP.safetyConfigs) == 1 and CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.psa
          and CP.safetyConfigs[0].safetyParam in (SAFETY_PARAM, COMBINED_SAFETY_PARAM, SPLIT_SAFETY_PARAM, EPS_CYCLE_SAFETY_PARAM))


def steering_frame(template, state, factor, torque):
  if len(template) != 8 or not 2 <= state <= 4 or not 0 <= factor <= 100 or not -TORQUE_SCALE <= torque <= TORQUE_SCALE:
    raise ValueError('Invalid T9 steering request')
  if template[5] & 1 or template[6] or template[7] & 0xFC:
    raise ValueError('Factory stream is not the recorded torque API')
  if torque and (state != 4 or factor != 100):
    raise ValueError('Nonzero torque requires active state and full factor')
  if state == 2 and (torque or factor):
    raise ValueError('Disabled request must be zero')
  data = bytearray(template)
  raw = torque & 0x7FF
  data[3] = raw >> 3
  data[4] = (data[4] & 3) | ((raw & 7) << 5) | (state << 2)
  data[5] = factor << 1
  return (0x3F2, bytes(data), 0)


class T9LateralTestController:
  def __init__(self, *, pause_supported=False, cycle_supported=False):
    self.pause_supported = pause_supported
    self.cycle_supported = cycle_supported
    self.observer = T9LkaCanObserver()
    self.lateral = T9LkaLifecycle(torque_limit=TORQUE_SCALE, cycle_supported=cycle_supported)
    self.template = None
    self.last_wrong_side = 0
    self.first_observation = None
    self.last_frame = 0
    self.next_frame = 0
    self.replacement_queued = False
    self.last_clock = 0
    self.last_applied_raw = 0
    self.status = {}
    self._merged_last = {}
    self._debug_key = None
    self._debug_nanos = 0
    # Receive-only diagnostic from the EPS. Community DBC calls this driver
    # torque activity; it is not proof of physical hands or an enable input.
    self.eps_driver_activity_candidate = None
    self.eps_activity_nanos = 0
    self.eps_feedback_hex = None
    self.physical_rearm_count = 0
    self.last_rearm_ns = 0
    self.previous_stop_reason = None
    self.queued_commands = deque(maxlen=8)
    self.last_tx_rejection_ns = 0

  def rearm_on_physical_cruise_off(self, now):
    previous_reason = self.lateral.reason
    if self.lateral.rearm_on_physical_cruise_off(now):
      self.physical_rearm_count += 1
      self.last_rearm_ns = now
      self.previous_stop_reason = previous_reason
      self.queued_commands.clear()

  def driver_activity(self, now):
    return self.eps_driver_activity_candidate if fresh(now, self.eps_activity_nanos) else None

  def log_status(self, now, inputs, state):
    feedback = self.observer.feedback
    key = (self.status['phase'], self.status['reason'], self.status['topology_ready'],
           self.status['rearm_required'], feedback.eps_state, feedback.stock_state,
           self.status['replacement_queued'], self.driver_activity(now))
    if key == self._debug_key and 0 <= now-self._debug_nanos < 1_000_000_000:
      return
    self._debug_key, self._debug_nanos = key, now
    # card forwards carlog to logMessage/rlog and /data/log. No file I/O here.
    # Full-rate input/output values remain in the native cereal services.
    def age(sample):
      return now-sample if 0 < sample <= now else None
    snapshot = self.status | {
      'mono_ns': now, 'requested': inputs.requested, 'torque_requested': inputs.torque,
      'speed_kph': inputs.speed_kph, 'driver_torque_raw': inputs.driver_torque_raw,
      'brake_pressed': inputs.brake_pressed, 'gas_pressed': bool(state.gasPressed),
      'cruise_enabled': bool(state.cruiseState.enabled), 'vehicle_ready': inputs.vehicle_ready,
      'can_valid': inputs.can_valid, 'eps_state': feedback.eps_state,
      'stock_state': feedback.stock_state, 'stock_factor': feedback.stock_factor,
      'eps_age_ns': age(feedback.eps_nanos), 'stock_age_ns': age(feedback.stock_nanos),
      'safety_rx_age_ns': age(inputs.safety_rx_nanos),
      'eps_driver_activity_candidate': self.driver_activity(now),
      'eps_feedback_hex': self.eps_feedback_hex if fresh(now, self.eps_activity_nanos) else None,
      'last_tx_rejection_ns': self.last_tx_rejection_ns,
    }
    snapshot = {k: None if isinstance(v, float) and not math.isfinite(v) else v for k, v in snapshot.items()}
    try:
      carlog.info('psa_t9_lateral ' + json.dumps(snapshot, separators=(',', ':'), allow_nan=False))
    except Exception:
      # Logging failure must not change command generation or safety release.
      pass

  def merge_for_carstate(self, packets):
    merged = []
    for timestamp, frames in packets:
      output = []
      for address, data, bus in frames:
        if bus not in (0, 2): continue
        previous = self._merged_last.get(address)
        if previous is not None:
          ts, origin, payload = previous
          if origin != bus and payload == data and 0 <= timestamp-ts <= 20_000_000:
            continue
        self._merged_last[address] = (timestamp, bus, bytes(data))
        output.append((address, data, 0))
      merged.append((timestamp, output))
    return merged

  def observe(self, packets):
    for timestamp, frames in packets:
      if self.first_observation is None: self.first_observation = timestamp
      observed = []
      for address, data, bus in frames:
        # Panda marks rejected host transmissions with bus 192 (0 + 0xC0).
        # Stop this episode on a recent rejection of our own queued command.
        # Continuing the requested ramp after a rejected sample can cause
        # further rate-limit rejects and exhaust the MCU command watchdog.
        # The normal state-2 release below still passes every Panda check.
        if (address == 0x3F2 and bus == 192 and len(data) == 8 and self.replacement_queued
            and any(bytes(data) == payload and fresh(timestamp, sent, 150_000_000)
                    for sent, payload in self.queued_commands)):
          self.last_tx_rejection_ns = max(self.last_tx_rejection_ns, timestamp)
          if self.lateral.phase != LkaPhase.BLOCKED:
            self.lateral._block('panda_steering_tx_rejected')
        if (address == 0x3F2 and bus == 0) or (address == 0x495 and bus == 2):
          self.last_wrong_side = max(self.last_wrong_side, timestamp)
        if address == 0x3F2 and bus == 2:
          self.template = bytes(data) if len(data) == 8 else None
          observed.append((address, data, 0))
        elif address == 0x495 and bus == 0:
          valid = len(data) == 4 and timestamp > 0 and timestamp >= self.eps_activity_nanos
          self.eps_driver_activity_candidate = bool(data[2] & 2) if valid else None
          self.eps_feedback_hex = bytes(data).hex() if valid else None
          self.eps_activity_nanos = max(timestamp, self.eps_activity_nanos)
          observed.append((address, data, 0))
      self.observer.update([(timestamp, observed)])

  def topology_ready(self, now):
    start = max(self.last_wrong_side, self.first_observation or now)
    return bool(self.template is not None and now-start >= 2_000_000_000
                and fresh(now, self.observer.feedback.stock_nanos, 150_000_000)
                and fresh(now, self.observer.feedback.eps_nanos))

  def update(self, CC, CS, now):
    state = CS.out
    topology = self.topology_ready(now)
    ready = (state.gearShifter == structs.CarState.GearShifter.drive and not state.parkingBrake
             and not state.doorOpen and not state.seatbeltUnlatched and not state.gasPressed
             and state.cruiseState.enabled and not state.steerFaultTemporary and not state.steerFaultPermanent)
    pause_requested = bool(self.pause_supported and CC.psaLateralPause)
    # The EPS ACK and carState/controlsd return travel through separate
    # processes. Allow one bounded fresh-control window for latActive to
    # return after ACK, only while the explicit cycle gate remains true.
    cycle_continuation = (self.cycle_supported and CC.psaEpsCycleReady
      and (self.lateral.cycling or 0 < now <= self.lateral.cycle_resume_until))
    requested = bool(CC.enabled and (CC.latActive or pause_requested or cycle_continuation)
                     and state.cruiseState.enabled and not CC.cruiseControl.cancel)
    inputs = LkaInputs(requested=requested, torque=float(CC.actuators.torque),
      pause_supported=self.pause_supported,
      cycle_ready=bool(self.cycle_supported and CC.psaEpsCycleReady),
      driver_activity=self.driver_activity(now) is True,
      resume_allowed=bool(self.pause_supported and CC.psaLateralResume and CC.latActive),
      pause_requested=pause_requested or (self.pause_supported and (state.leftBlinker or state.rightBlinker)),
      # Remove float32 m/s conversion noise, below the wheel CAN resolution.
      speed_kph=round(float(state.vEgoRaw)*3.6, 4), driver_torque_raw=float(state.steeringTorque),
      driver_override=bool(state.steeringPressed), brake_pressed=bool(state.brakePressed),
      vehicle_ready=bool(ready), can_valid=bool(state.canValid and not state.canTimeout and topology),
      safety_rx_nanos=getattr(CS, 't9_shadow_safety_rx_nanos', 0), feedback=self.observer.feedback)
    clock_valid = isinstance(now, int) and now > 0 and now >= self.last_clock
    periodic_due = (clock_valid and now >= self.next_frame and
                    (not self.last_frame or now-self.last_frame >= MIN_PERIOD_NS))
    # Only the validated physical-off path above clears a native episode.
    # Engine state 1 during accelerator override is not a manual rearm.
    if self.lateral.phase == LkaPhase.BLOCKED:
      decision = self.lateral._decision()
    else:
      decision = self.lateral.update(now, inputs, command_tick=periodic_due)
    messages = []
    # Allow an immediate zero release between normal 20 Hz command ticks.
    zero_release = decision.torque_raw == 0 and self.last_applied_raw != 0
    finish_replacement = self.replacement_queued and decision.state == 2 and not self.lateral.cycling
    # While disabled, preserve the complete factory stream and its timing.
    # Queue a state-2 release only to end a previously queued takeover.
    needs_command = decision.state != 2 or self.replacement_queued
    if topology and clock_valid and needs_command and (zero_release or finish_replacement or periodic_due):
      try:
        messages.append(steering_frame(self.template, decision.state, decision.factor, decision.torque_raw))
      except ValueError:
        self.lateral._block('factory_template_invalid')
      else:
        self.last_frame = now
        self.last_applied_raw = decision.torque_raw
        self.replacement_queued = decision.state != 2 or self.lateral.cycling
        if self.replacement_queued:
          self.queued_commands.append((now, bytes(messages[-1][1])))
        else:
          self.queued_commands.clear()
        if not self.replacement_queued:
          self.next_frame = 0
        elif not self.next_frame or zero_release:
          self.next_frame = now + PERIOD_NS
        else:
          # Keep the original 20 Hz deadlines. Setting last_send + 50 ms on
          # each jittered 100 Hz call accumulated delay and produced ~18 Hz.
          self.next_frame += (1 + (now-self.next_frame)//PERIOD_NS) * PERIOD_NS
    if not topology or not clock_valid:
      self.last_applied_raw = 0
    self.last_clock = now
    output = structs.CarControl.Actuators()
    output.torque = self.last_applied_raw / TORQUE_SCALE
    output.torqueOutputCan = self.last_applied_raw
    # This is the requested/applied-to-safety output, not proof of EPS torque.
    # Native pandaStates and CAN TX rejects remain the independent evidence.
    self.status = asdict(self.lateral._decision()) | {'mode': 'lateral_test', 'topology_ready': topology,
      'replacement_queued': self.replacement_queued,
      'queued_steering_frames': len(messages), 'longitudinal_enabled': False,
      'torque_raw_queued': self.last_applied_raw, 'expected_safety_param': SAFETY_PARAM}
    self.status.update(eps_cycle_enabled=self.cycle_supported, eps_cycling=self.lateral.cycling,
                       eps_cycle_count=self.lateral.cycle_count, physical_rearm_count=self.physical_rearm_count,
                       last_rearm_ns=self.last_rearm_ns, previous_stop_reason=self.previous_stop_reason)
    self.log_status(now, inputs, state)
    return output, messages
