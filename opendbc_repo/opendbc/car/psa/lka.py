"""Candidate T9 torque activation lifecycle, evaluated without CAN output.

LKA request 4 (0x3F2) and EPS response 3 (0x495) are distinct signals.
Timings below are a conservative experiment, not a validated EPS handshake.
No transport, packer, safety-mode switch or real-vehicle enable lives here.
"""

from dataclasses import dataclass, field
from enum import StrEnum
import math


PERIOD_NS = 50_000_000
INPUT_TIMEOUT_NS = 250_000_000
STOCK_TIMEOUT_NS = 150_000_000
CONTROL_TIMEOUT_NS = 150_000_000
PREPARE_NS = 100_000_000
EPS_ACK_TIMEOUT_NS = 500_000_000
EPS_RELEASE_TIMEOUT_NS = 1_500_000_000
MIN_SPEED_KPH = 67.1
MAX_SPEED_KPH = 140.0
DRIVER_TORQUE_LIMIT = 15
TORQUE_LIMIT = 1  # Initial passive shadow envelope; never the observed factory maximum.
ACTIVE_TORQUE_LIMIT = 15  # Experimental command envelope requested after the +/-10 road capture.
NORMALIZED_TORQUE_SCALE = ACTIVE_TORQUE_LIMIT
EPS_CYCLE_PERIOD_NS = 12_000_000_000
EPS_CYCLE_TIMEOUT_NS = 2_000_000_000

# One recorded activation, NOT a universal PSA requirement. Used only for
# this candidate's diagnostics; no fabricated EPS response follows this ramp.
FACTOR_STEPS = (1, 1, 1, 1, 10, 16, 29, 40, 55, 71, 85, 94, 100)


class LkaPhase(StrEnum):
  DISABLED = "disabled"
  PREPARING = "preparing"
  WAITING_EPS = "waiting_eps"
  ACTIVE = "active"
  PAUSED = "paused"
  CYCLE_ARMING = "cycle_arming"
  CYCLE_RELEASING = "cycle_releasing"
  RELEASING = "releasing"
  BLOCKED = "blocked"


@dataclass(frozen=True)
class LkaFeedback:
  stock_state: int | None = None
  stock_factor: int = 0
  stock_angle_raw: int = 0
  stock_lxa: int = 0
  stock_nanos: int = 0
  eps_state: int | None = None
  eps_nanos: int = 0


@dataclass(frozen=True)
class LkaInputs:
  requested: bool = False
  pause_supported: bool = False
  pause_requested: bool = False
  resume_allowed: bool = False
  cycle_ready: bool = False
  driver_activity: bool = False
  torque: float = 0.0  # openpilot normalized actuator, not Nm
  speed_kph: float = 0.0
  driver_torque_raw: float = 0.0
  driver_override: bool = False
  brake_pressed: bool = False
  vehicle_ready: bool = False
  can_valid: bool = False
  safety_rx_nanos: int = 0
  feedback: LkaFeedback = field(default_factory=LkaFeedback)


@dataclass(frozen=True)
class LkaDecision:
  phase: LkaPhase
  state: int
  factor: int
  torque_raw: int
  reason: str
  rearm_required: bool
  eps_confirmed: bool

  @property
  def tx_allowed(self) -> bool:
    return False


def fresh(now_nanos: int, sample_nanos: int, limit_ns: int = INPUT_TIMEOUT_NS) -> bool:
  return 0 < sample_nanos <= now_nanos and now_nanos - sample_nanos <= limit_ns


class T9LkaLifecycle:
  """Deterministic shadow supervisor called on every control update.

  Inhibitions are checked even between 20 Hz proposal ticks. Any interrupted
  engagement needs request=false followed by a new request after EPS release.
  Acceleration pedal and RVV state do not engage/disengage this lateral-only
  candidate; they remain inputs of the separate longitudinal supervisor.
  """

  def __init__(self, torque_limit: int = TORQUE_LIMIT, *, cycle_supported: bool = False):
    if not isinstance(torque_limit, int) or not 1 <= torque_limit <= ACTIVE_TORQUE_LIMIT:
      raise ValueError(f"Candidate torque limit must be 1..{ACTIVE_TORQUE_LIMIT} raw")
    self.torque_limit = torque_limit
    self.phase = LkaPhase.DISABLED
    self.reason = "not_requested"
    self.factor = 0
    self.torque_raw = 0
    self.rearm_required = False
    self._previous_request = False
    self._last_update: int | None = None
    self._last_tick: int | None = None
    self._phase_started = 0
    self._activation_started = 0
    self._factor_index = 0
    self.cycle_supported = cycle_supported
    self.cycling = False
    self.cycle_count = 0
    self._active_since = 0
    self._cycle_started = 0
    self.cycle_resume_until = 0

  def _block(self, reason: str):
    self.cycle_resume_until = 0
    self.cycling = False
    self._active_since = 0
    self.phase = LkaPhase.BLOCKED
    self.reason = reason
    self.factor = 0
    self.torque_raw = 0
    self.rearm_required = True

  def rearm_on_physical_cruise_off(self, now: int) -> bool:
    """Clear an old episode after the caller validates physical RVV off.

    This is not an enable request. The next request must pass all current
    input checks and obtain a new EPS acknowledgement. In particular, an
    EPS fault may still be present while the driver turns cruise off.
    """
    if type(now) is not int or now <= 0 or (self._last_update is not None and now < self._last_update):
      return False
    if self.phase == LkaPhase.DISABLED:
      return False
    self.phase = LkaPhase.DISABLED
    self.reason = 'physical_rvv_off'
    self.factor = self.torque_raw = 0
    self.rearm_required = self._previous_request = False
    self._last_update = now
    self._phase_started = self._activation_started = self._factor_index = 0
    self.cycling = False
    self.cycle_resume_until = 0
    self._active_since = 0
    return True

  @staticmethod
  def _feedback_problem(now: int, inputs: LkaInputs) -> str | None:
    feedback = inputs.feedback
    if not inputs.can_valid:
      return "can_invalid"
    if not fresh(now, inputs.safety_rx_nanos):
      return "safety_rx_stale"
    if not fresh(now, feedback.stock_nanos, STOCK_TIMEOUT_NS):
      return "stock_lka_stale"
    if feedback.stock_state not in (2, 3, 4):
      return "stock_lka_unavailable"
    if not 0 <= feedback.stock_factor <= 100 or feedback.stock_angle_raw != 0 or feedback.stock_lxa != 0:
      return "stock_torque_api_invalid"
    if not fresh(now, feedback.eps_nanos):
      return "eps_feedback_stale"
    # Keep the physical reason distinct: lack of authorization is not proof
    # of a permanent fault. None includes malformed or out-of-order feedback.
    if feedback.eps_state == 0:
      return "eps_not_authorized"
    if feedback.eps_state == 4:
      return "eps_fault"
    if feedback.eps_state not in (1, 2, 3):
      return "eps_feedback_invalid"
    return None

  @staticmethod
  def _engagement_problem(inputs: LkaInputs, *, allow_override=False) -> str | None:
    if not all(math.isfinite(value) for value in (inputs.torque, inputs.speed_kph, inputs.driver_torque_raw)):
      return "nonfinite_input"
    if inputs.brake_pressed:
      return "brake_pressed"
    driver_high = abs(inputs.driver_torque_raw) > DRIVER_TORQUE_LIMIT
    if not allow_override and (inputs.driver_override or driver_high):
      return "driver_override"
    if not inputs.vehicle_ready:
      return "vehicle_not_ready"
    if inputs.pause_supported and inputs.speed_kph < MIN_SPEED_KPH:
      return "speed_below_lateral_envelope"
    if not MIN_SPEED_KPH <= inputs.speed_kph <= MAX_SPEED_KPH:
      return "speed_outside_candidate_envelope"
    if inputs.feedback.stock_state not in (3, 4):
      return "stock_lka_not_authorized"
    return None

  def _decision(self) -> LkaDecision:
    state = 2
    if self.phase in (LkaPhase.PREPARING, LkaPhase.RELEASING):
      state = 3
    elif self.phase in (LkaPhase.WAITING_EPS, LkaPhase.ACTIVE, LkaPhase.PAUSED, LkaPhase.CYCLE_ARMING):
      state = 4
    return LkaDecision(self.phase, state, self.factor, self.torque_raw, self.reason,
                       self.rearm_required, self.phase in (LkaPhase.ACTIVE, LkaPhase.PAUSED))

  def update(self, now_nanos: int, inputs: LkaInputs, *, command_tick: bool | None = None) -> LkaDecision:
    if not isinstance(now_nanos, int) or isinstance(now_nanos, bool) or now_nanos <= 0:
      self._block("invalid_clock")
      return self._decision()
    if self._last_update is not None and now_nanos < self._last_update:
      self._block("clock_reversed")
      return self._decision()

    rising_request = inputs.requested and not self._previous_request
    self._previous_request = inputs.requested
    gap = self._last_update is not None and now_nanos - self._last_update > CONTROL_TIMEOUT_NS
    self._last_update = now_nanos
    # The native sender supplies its scheduled tick so torque/factor changes
    # cannot advance twice between two CAN commands. Shadow callers retain
    # their own clock; inhibitions below are always evaluated on every call.
    tick = (self._last_tick is None or now_nanos - self._last_tick >= PERIOD_NS) if command_tick is None else command_tick
    if tick:
      self._last_tick = now_nanos

    if gap and (self.cycling or self.phase in (LkaPhase.PREPARING, LkaPhase.WAITING_EPS, LkaPhase.ACTIVE, LkaPhase.PAUSED, LkaPhase.RELEASING)):
      self._block("control_update_timeout")
      return self._decision()

    feedback_problem = self._feedback_problem(now_nanos, inputs)
    if self.phase == LkaPhase.BLOCKED:
      if not inputs.requested and feedback_problem is None and inputs.feedback.eps_state in (1, 2):
        self.phase = LkaPhase.DISABLED
        self.reason = "rearmed_request_off"
        self.rearm_required = False
      return self._decision()

    if self.phase == LkaPhase.DISABLED:
      if not rising_request:
        return self._decision()
      problem = feedback_problem or self._engagement_problem(inputs)
      if problem is None and inputs.feedback.eps_state == 3:
        problem = "eps_not_released"
      if problem:
        self._block(problem)
      else:
        self.phase = LkaPhase.PREPARING
        self._phase_started = now_nanos
        self.reason = "prepare_zero_torque"
      return self._decision()

    if feedback_problem:
      self._block(feedback_problem)
      return self._decision()

    if self.cycling:
      problem = self._engagement_problem(inputs)
      if not inputs.requested or not inputs.cycle_ready or inputs.pause_requested:
        problem = problem or 'eps_cycle_interrupted'
      if now_nanos - self._cycle_started >= EPS_CYCLE_TIMEOUT_NS:
        problem = problem or 'eps_cycle_timeout'
      if problem:
        self._block(problem)
        return self._decision()
      if self.phase == LkaPhase.CYCLE_ARMING:
        if inputs.feedback.eps_state != 3:
          self._block('eps_activation_lost')
        elif tick:
          self.phase = LkaPhase.CYCLE_RELEASING
          self._phase_started = now_nanos
          self.reason = 'eps_cycle_wait_release'
        return self._decision()
      if self.phase == LkaPhase.CYCLE_RELEASING:
        if (tick and inputs.feedback.eps_state in (1, 2)
            and inputs.feedback.eps_nanos > self._phase_started):
          self.phase = LkaPhase.PREPARING
          self._phase_started = now_nanos
          self.reason = 'eps_cycle_prepare'
        elif now_nanos - self._phase_started >= EPS_RELEASE_TIMEOUT_NS:
          self._block('eps_release_timeout')
        return self._decision()

    if self.phase == LkaPhase.RELEASING:
      if inputs.feedback.stock_state == 2:
        self._block("stock_lka_not_authorized")
        return self._decision()
      if tick:
        self.factor = max(0, self.factor - 5)
      if self.factor == 0 and inputs.feedback.eps_state in (1, 2) and inputs.feedback.eps_nanos > self._phase_started:
        self.phase = LkaPhase.BLOCKED if inputs.requested else LkaPhase.DISABLED
        self.rearm_required = inputs.requested
        self.reason = "new_request_required" if inputs.requested else "released"
      elif now_nanos - self._phase_started >= EPS_RELEASE_TIMEOUT_NS:
        self._block("eps_release_timeout")
      return self._decision()

    can_pause = inputs.pause_supported and self.phase in (LkaPhase.ACTIVE, LkaPhase.PAUSED)
    problem = self._engagement_problem(inputs, allow_override=can_pause)
    if problem:
      self._block(problem)
      return self._decision()

    if not inputs.requested:
      self.torque_raw = 0
      self.rearm_required = True
      self.phase = LkaPhase.RELEASING
      self._phase_started = now_nanos
      self.reason = "request_released"
      return self._decision()

    if can_pause:
      if inputs.feedback.eps_state != 3:
        self._block('eps_activation_lost')
        return self._decision()
      if (inputs.pause_requested or inputs.driver_override or abs(inputs.driver_torque_raw) > DRIVER_TORQUE_LIMIT
          or (self.phase == LkaPhase.PAUSED and not inputs.resume_allowed)):
        self.phase = LkaPhase.PAUSED
        self.reason = 'lateral_pause'
        self.torque_raw = 0
        return self._decision()
      if self.phase == LkaPhase.PAUSED:
        self.phase = LkaPhase.ACTIVE
        self.reason = 'lateral_resumed'
        self.torque_raw = 0
        return self._decision()

    # Explicit zero-factor marker precedes state 2. Only the dedicated MCU
    # profile recognizes it; an ordinary stop never becomes a cycle request.
    # Physical EPS driver activity must be true at entry. A cycle cannot be
    # used to recover a revoked authorization or a latched stop.
    if (self.cycle_supported and self.phase == LkaPhase.ACTIVE and not self.cycling
        and self._active_since and now_nanos - self._active_since >= EPS_CYCLE_PERIOD_NS
        and inputs.cycle_ready and inputs.driver_activity and tick):
      self.cycling = True
      self._cycle_started = now_nanos
      self.phase = LkaPhase.CYCLE_ARMING
      self.factor = self.torque_raw = 0
      self.reason = 'eps_cycle_zero_marker'
      return self._decision()

    if self.phase == LkaPhase.PREPARING:
      if inputs.feedback.eps_state == 3:
        self._block("eps_active_before_request")
      elif tick and now_nanos - self._phase_started >= PREPARE_NS:
        self.phase = LkaPhase.WAITING_EPS
        self._activation_started = now_nanos
        self._factor_index = 0
        self.factor = FACTOR_STEPS[0]
        self.reason = "waiting_for_observed_eps_active"
      return self._decision()

    if self.phase == LkaPhase.WAITING_EPS:
      # Reject late acknowledgements, even if EPS happens to be active now.
      if now_nanos - self._activation_started >= EPS_ACK_TIMEOUT_NS:
        self._block("eps_activation_timeout")
        return self._decision()
      if inputs.feedback.eps_state == 3 and inputs.feedback.eps_nanos > self._activation_started:
        self.phase = LkaPhase.ACTIVE
        self._active_since = now_nanos
        if self.cycling:
          self.cycle_count += 1
          self.cycle_resume_until = now_nanos + CONTROL_TIMEOUT_NS
        self.cycling = False
        self.reason = "eps_active_observed"
        # Confirmation update still proposes zero torque.
        return self._decision()
    elif inputs.feedback.eps_state != 3:
      self._block("eps_activation_lost")
      return self._decision()

    if tick:
      # One step per actual proposal tick; never catch up after a timing gap.
      self._factor_index = min(self._factor_index + 1, len(FACTOR_STEPS) - 1)
      self.factor = FACTOR_STEPS[self._factor_index]
      if self.phase == LkaPhase.ACTIVE and self.factor == 100:
        requested_raw = int(round(max(-1.0, min(1.0, inputs.torque)) * NORMALIZED_TORQUE_SCALE))
        target = max(-self.torque_limit, min(self.torque_limit, requested_raw))
        self.torque_raw += max(-1, min(1, target - self.torque_raw))
    return self._decision()
