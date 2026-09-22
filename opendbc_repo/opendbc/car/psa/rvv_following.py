"""Vision-to-RVV proposal, with recovery behind a fresh, confident lead.

The gap law comes from HIL/sim/cruise_speed_assistant.py. This version is
owned by the port and also checks native message/model/calibration freshness.
It is not a brake controller and never authorizes physical actuation.
"""
from dataclasses import asdict, dataclass, replace
import math
import struct

from opendbc.car.psa.lka import fresh
from opendbc.car.psa.rvv import MIN_SETPOINT_KPH, MAX_SETPOINT_KPH, normalize_stock_setpoint_kph
from opendbc.car.psa import rvv_wire

ANTICIPATION_SECONDS = 4.0
RECOVERY_STABLE_NS = 1_000_000_000
RECOVERY_MAX_CLOSING_MS = 0.5
ANTICIPATION_ACQUIRE_NS = 200_000_000


@dataclass(frozen=True)
class RvvLead:
  present: bool = False
  distance_m: float = 0.
  speed_ms: float = 0.
  relative_speed_ms: float = 0.
  probability: float = 0.
  rx_ns: int = 0
  model_ns: int = 0
  valid: bool = False
  radar: bool = False


@dataclass(frozen=True)
class FollowingDecision:
  reason: str
  target_kph: int | None = None
  requested_accel_ms2: float | None = None
  desired_gap_m: float | None = None
  ttc_s: float | None = None
  driver_intervention_required: bool = False
  rearm_required: bool = False
  tx_allowed: bool = False
  computed_ns: int = 0
  lead_rx_ns: int = 0
  model_ns: int = 0
  candidate_increase_permitted: bool = False
  common_fault: bool = False
  anticipated_gap_m: float | None = None
  recovery_waiting: bool = False
  anticipation_only: bool = False


class T9RvvFollowing:
  def __init__(self):
    self.last_clock = 0
    self.target = None
    self.blocked = None
    self.recovery_since = 0

  def _block(self, reason):
    self.blocked = self.blocked or reason
    return FollowingDecision(self.blocked, driver_intervention_required=True, rearm_required=True)

  def update(self, now, *, active, car_valid, car_ns, speed_kph, stock_setpoint_kph,
             calibrated, calibration_ns, lead):
    if type(now) is not int or now <= 0 or now < self.last_clock:
      return self._block('invalid_clock')
    gap = self.last_clock and now - self.last_clock > 250_000_000
    self.last_clock = now
    if not car_valid or not fresh(now, car_ns):
      return self._block('car_state_invalid_or_stale')
    if not active:
      self.blocked = self.target = None
      self.recovery_since = 0
      return FollowingDecision('stock_rvv_inactive')
    if self.blocked:
      return self._block(self.blocked)
    if gap and self.target is not None:
      return self._block('following_update_timeout')
    if not all(math.isfinite(v) for v in (speed_kph, stock_setpoint_kph)):
      return self._block('nonfinite_vehicle_input')
    stock_setpoint_kph = normalize_stock_setpoint_kph(stock_setpoint_kph)
    if not MIN_SETPOINT_KPH <= stock_setpoint_kph <= MAX_SETPOINT_KPH or speed_kph < MIN_SETPOINT_KPH:
      return self._block('outside_rvv_envelope')
    if not calibrated or not fresh(now, calibration_ns, 2_000_000_000):
      return self._block('calibration_invalid_or_stale')
    if not lead.valid or not fresh(now, lead.rx_ns) or not fresh(now, lead.model_ns, 500_000_000):
      return self._block('lead_invalid_or_stale')
    if not lead.present:
      if self.target is None:
        return FollowingDecision('waiting_for_lead')
      return self._block('lead_lost')
    if not all(math.isfinite(v) for v in (lead.distance_m, lead.speed_ms, lead.relative_speed_ms, lead.probability)):
      return self._block('nonfinite_lead')
    if not 0. <= lead.probability <= 1.0:
      return self._block('lead_probability_invalid')
    if lead.probability < 0.75:
      if self.target is None:
        return FollowingDecision('waiting_for_confident_lead')
      return self._block('lead_low_confidence')
    if lead.distance_m <= 0 or lead.speed_ms < 0:
      return self._block('lead_geometry_invalid')
    ego_ms = speed_kph / 3.6
    if abs((lead.speed_ms - ego_ms) - lead.relative_speed_ms) > 3.0:
      return self._block('lead_speed_inconsistent')
    gap_m = 5.0 + 2.0 * ego_ms
    closing_ms = max(-lead.relative_speed_ms, 0.)
    ttc = lead.distance_m / closing_ms if closing_ms > 0.1 else None
    if (ttc is not None and ttc < 4.) or lead.distance_m < max(5., gap_m * 0.5):
      return self._block('fixed_cruise_cannot_handle_critical_lead')
    base_target = math.floor(min(stock_setpoint_kph, (lead.speed_ms + 0.20 * (lead.distance_m - gap_m)) * 3.6))
    if base_target < MIN_SETPOINT_KPH:
      return self._block('lead_target_below_rvv_minimum')
    # Begin reducing the setpoint while the current gap is still comfortable
    # but closing. This distance projection is not a deadline for attaining
    # the target or a claim about available engine deceleration.
    anticipated_gap = gap_m + ANTICIPATION_SECONDS * closing_ms
    target = max(MIN_SETPOINT_KPH, math.floor(min(stock_setpoint_kph,
      (lead.speed_ms + 0.20 * (lead.distance_m - anticipated_gap)) * 3.6)))
    recovery_waiting = False
    if self.target is not None:
      recovering = target > self.target
      recovery_ready = closing_ms <= RECOVERY_MAX_CLOSING_MS and lead.distance_m >= gap_m
      if not recovering or not recovery_ready:
        self.recovery_since = 0
      elif not self.recovery_since:
        self.recovery_since = now
      if recovering and (not self.recovery_since or now - self.recovery_since < RECOVERY_STABLE_NS):
        target = min(self.target, math.floor(stock_setpoint_kph))
        recovery_waiting = True
    if self.target is None and target == stock_setpoint_kph:
      # No adaptive reduction is requested. Leave the original BSI stream
      # untouched and keep waiting; a brief lead detection must not acquire
      # control and then latch a loss-of-lead fault for an unchanged setpoint.
      # After a real reduction, recovery to the ceiling remains an owned
      # episode and keeps every existing loss/rearm protection.
      return FollowingDecision('stock_setpoint_sufficient', desired_gap_m=gap_m, ttc_s=ttc,
                               computed_ns=now, lead_rx_ns=lead.rx_ns, model_ns=lead.model_ns,
                               anticipated_gap_m=anticipated_gap)
    # Propose a cruise setpoint, not a speed to achieve within two seconds.
    # Panda applies separate bounded down/up setpoint ramps. Neither
    # that ramp nor this target estimates or guarantees vehicle deceleration.
    self.target = target
    return FollowingDecision('vision_following_candidate', target_kph=target, desired_gap_m=gap_m, ttc_s=ttc,
                             computed_ns=now, lead_rx_ns=lead.rx_ns, model_ns=lead.model_ns,
                             candidate_increase_permitted=True, anticipated_gap_m=anticipated_gap,
                             recovery_waiting=recovery_waiting, anticipation_only=base_target == stock_setpoint_kph)


class T9RvvFollowingObserver:
  """Use controlsd's existing carState; only radarState adds a subscription.

  This observer logs a proposal. It does not modify CarControl, engagement,
  longitudinalPlan, Panda configuration or the active lateral controller.
  """
  def __init__(self, *, split_axes=False):
    self.split_axes = split_axes
    self.planner = T9RvvFollowing()
    self.pending_planner = None
    self.status = {}
    self.decision = FollowingDecision("initializing")
    self.log_key = None
    self.log_ns = 0
    self.anticipation_since = 0
    self.anticipation_model_ns = 0
    self.anticipation_samples = 0
    self.last_update = 0

  @property
  def request_episode_started(self):
    return self.planner.target is not None or self.planner.blocked is not None

  def command_published(self, payload):
    """Commit a first request only after its nonzero wire publication succeeds.

    A split-profile local stop published during an engaged session also
    commits its latch, even before the first nonzero request. Disabled
    observations never do. An encoded request is not proof of Panda
    acceptance or engine response. Once one has been published, keep the planner's interruption/rearm state
    even when later native disengagement makes the wire target zero.
    """
    pending, self.pending_planner = self.pending_planner, None
    if pending is None:
      return
    try:
      _, target, flags, _, computed_ns, _, _ = rvv_wire.WIRE.unpack(payload)
    except (struct.error, TypeError):
      return
    published_request = target != 0 and flags == 1
    published_local_stop = self.split_axes and flags == 3 and pending.blocked is not None
    if ((published_request or published_local_stop)
        and payload == rvv_wire.command(self.decision, now=computed_ns, engaged=True, split_axes=self.split_axes)):
      self.planner = pending

  def update(self, sm, now):
    car, radar, calibration = sm['carState'], sm['radarState'], sm['liveCalibration']
    native = radar.leadOne
    lead = RvvLead(bool(native.status), float(native.dRel), float(native.vLead), float(native.vRel),
                   float(native.modelProb), int(sm.logMonoTime['radarState']), int(radar.mdMonoTime),
                   bool(sm.valid['radarState'] and sm.alive['radarState']), bool(native.radar))
    # Observation alone must not acquire a request episode. In particular,
    # native openpilot may be disabled while Peugeot cruise remains active.
    # Recompute prospective decisions from current inputs until a nonzero
    # request is actually published; retain every current inhibition on that
    # decision. An already requested episode retains its persistent planner.
    planner = self.planner if self.request_episode_started else T9RvvFollowing()
    decision = planner.update(now, active=bool(car.cruiseState.enabled),
      car_valid=bool(sm.valid['carState'] and sm.alive['carState'] and car.canValid and not car.canTimeout),
      car_ns=int(sm.logMonoTime['carState']), speed_kph=round(float(car.vEgoRaw) * 3.6, 2),
      stock_setpoint_kph=float(car.cruiseState.speed) * 3.6,
      calibrated=bool(sm.valid['liveCalibration'] and sm.alive['liveCalibration'] and str(calibration.calStatus) == 'calibrated'),
      calibration_ns=int(sm.logMonoTime['liveCalibration']), lead=lead)
    if self.split_axes:
      # Re-check shared inputs even when a prior local lead failure has
      # latched the RVV planner. Its first reason must not hide a new common
      # model/CAN/calibration failure from the independent steering axis.
      shared_valid = (car.canValid and not car.canTimeout
        and sm.valid['carState'] and sm.alive['carState'] and fresh(now, sm.logMonoTime['carState'])
        and sm.valid['modelV2'] and sm.alive['modelV2'] and fresh(now, sm.logMonoTime['modelV2'], 500_000_000)
        and sm.valid['liveCalibration'] and sm.alive['liveCalibration']
        and str(calibration.calStatus) == 'calibrated' and fresh(now, sm.logMonoTime['liveCalibration'], 2_000_000_000))
      decision = replace(decision, common_fault=not shared_valid)
    # An earlier takeover needs a stable series, not the isolated marginal
    # detection seen before the recorded truck approach. This read-only
    # acquisition history cannot commit an actuation episode or clear a stop.
    early = (not self.request_episode_started and decision.reason == 'vision_following_candidate'
             and decision.anticipation_only and not decision.common_fault)
    if (not early or not 0 < self.last_update <= now or now - self.last_update > 150_000_000
        or lead.model_ns < self.anticipation_model_ns):
      self.anticipation_since = self.anticipation_model_ns = self.anticipation_samples = 0
    if early:
      if not self.anticipation_since:
        self.anticipation_since = now
      new_model = lead.model_ns > self.anticipation_model_ns
      if new_model:
        self.anticipation_model_ns = lead.model_ns
        self.anticipation_samples += 1
      if now - self.anticipation_since < ANTICIPATION_ACQUIRE_NS or self.anticipation_samples < 3 or not new_model:
        decision = replace(decision, reason='waiting_for_stable_anticipation', target_kph=None,
                           candidate_increase_permitted=False)
    self.last_update = now
    self.pending_planner = planner
    self.decision = decision
    self.status = asdict(decision) | {'mode': 'rvv_vision_observation', 'mono_ns': now,
      'lead': asdict(lead), 'stock_setpoint_kph': float(car.cruiseState.speed) * 3.6,
      'speed_kph': float(car.vEgoRaw) * 3.6, 'stock_rvv_active': bool(car.cruiseState.enabled),
      'request_episode_started': self.request_episode_started}
    key = (decision.reason, decision.target_kph, decision.rearm_required, self.request_episode_started)
    if key != self.log_key or now - self.log_ns >= 1_000_000_000:
      self.log_key, self.log_ns = key, now
      return self.status
    return None
