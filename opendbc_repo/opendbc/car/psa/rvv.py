"""T9 conventional-cruise speed adapter, strictly without a CAN transport.

Ports the setpoint envelope from the local RVV simulator/ESP32 bench bridge.
The -0.30 m/s² engine-braking value is an unvalidated simulation assumption,
not an acceleration guarantee. Decisions are never permission to actuate.
"""

from dataclasses import dataclass
import math

from opendbc.car.psa.lka import fresh


STEP_NS = 500_000_000
DOWN_STEP_NS = 100_000_000
CONTROL_TIMEOUT_NS = 150_000_000
MIN_SETPOINT_KPH = 40
MAX_SETPOINT_KPH = 140
ENGINE_BRAKE_PRIOR_MS2 = 0.30


def normalize_stock_setpoint_kph(value: float) -> float:
  """Recover integer CAN km/h after CarState's float32 m/s round trip.

  Only the physical driver's setpoint has integer-km/h resolution. Never
  apply this tolerance to measured speed or a continuously computed target.
  """
  if math.isfinite(value) and abs(value - round(value)) <= 1e-4:
    return float(round(value))
  return value


@dataclass(frozen=True)
class RvvInputs:
  requested: bool = False
  requested_accel_ms2: float | None = 0.0  # None only for an explicit speed-setpoint request.
  speed_kph: float = 0.0
  stock_setpoint_kph: float = 0.0
  cruise_active: bool = False
  cruise_available: bool = False
  brake_pressed: bool = False
  gas_pressed: bool = False
  cancel: bool = False
  vehicle_ready: bool = False
  can_valid: bool = False
  safety_rx_nanos: int = 0
  stock_rx_nanos: int = 0
  target_setpoint_kph: float | None = None


@dataclass(frozen=True)
class RvvDecision:
  phase: str
  reason: str
  proposed_setpoint_kph: int | None = None
  target_setpoint_kph: int | None = None
  service_brake_required: bool = False
  rearm_required: bool = False
  tx_allowed: bool = False


class T9RvvAdapter:
  """Adjust only an already engaged RVV; never activate, resume or brake.

  Explicit setpoints step down by 1 km/h per 100 ms and up per 500 ms.
  The legacy offline acceleration proposal retains its 500 ms step.
  No path catches up after a delay. Interruptions
  latch until request=false and then a new request. Driver setpoint reductions
  take effect immediately, including between proposal ticks.
  """

  def __init__(self):
    self._last_update: int | None = None
    self._last_step: int | None = None
    self._proposed: int | None = None
    self._previous_request = False
    self._blocked_reason: str | None = None

  @staticmethod
  def problem(now: int, inputs: RvvInputs) -> str | None:
    if not inputs.can_valid:
      return "can_invalid"
    if not fresh(now, inputs.safety_rx_nanos):
      return "safety_rx_stale"
    if not fresh(now, inputs.stock_rx_nanos):
      return "rvv_rx_stale"
    if not all(math.isfinite(v) for v in (inputs.speed_kph, inputs.stock_setpoint_kph)):
      return "nonfinite_input"
    if inputs.requested_accel_ms2 is None:
      if inputs.target_setpoint_kph is None:
        return "missing_acceleration_or_target"
    elif not math.isfinite(inputs.requested_accel_ms2):
      return "nonfinite_input"
    stock = normalize_stock_setpoint_kph(inputs.stock_setpoint_kph)
    if inputs.target_setpoint_kph is not None:
      if not math.isfinite(inputs.target_setpoint_kph):
        return "nonfinite_target"
      if not MIN_SETPOINT_KPH <= inputs.target_setpoint_kph <= stock:
        return "target_outside_driver_ceiling"
    if inputs.cancel:
      return "driver_cancel"
    if inputs.brake_pressed:
      return "brake_pressed"
    if inputs.gas_pressed:
      return "driver_accelerator"
    if not inputs.vehicle_ready:
      return "vehicle_not_ready"
    if not inputs.cruise_available:
      return "rvv_mode_not_selected"
    if not inputs.cruise_active:
      return "rvv_not_active"
    if not MIN_SETPOINT_KPH <= stock <= MAX_SETPOINT_KPH:
      return "setpoint_out_of_range"
    if inputs.speed_kph < MIN_SETPOINT_KPH:
      return "speed_below_rvv_envelope"
    if inputs.requested_accel_ms2 is not None and inputs.requested_accel_ms2 < -ENGINE_BRAKE_PRIOR_MS2:
      return "service_brake_required"
    return None

  def _block(self, reason: str) -> RvvDecision:
    self._blocked_reason = reason
    self._proposed = None
    self._last_step = None
    return RvvDecision("blocked", reason, service_brake_required=reason == "service_brake_required", rearm_required=True)

  def update(self, now: int, inputs: RvvInputs) -> RvvDecision:
    if not isinstance(now, int) or isinstance(now, bool) or now <= 0:
      return self._block("invalid_clock")
    if self._last_update is not None and now < self._last_update:
      return self._block("clock_reversed")
    gap = self._last_update is not None and now - self._last_update > CONTROL_TIMEOUT_NS
    self._last_update = now
    rising = inputs.requested and not self._previous_request
    self._previous_request = inputs.requested

    if not inputs.requested:
      self._proposed = None
      self._last_step = None
      self._blocked_reason = None
      return RvvDecision("disabled", "not_requested")
    if gap and self._proposed is not None:
      return self._block("control_update_timeout")
    if self._blocked_reason is not None:
      return self._block(self._blocked_reason)
    problem = self.problem(now, inputs)
    if problem:
      return self._block(problem)
    if self._proposed is None and not rising:
      return self._block("new_request_required")

    stock = math.floor(normalize_stock_setpoint_kph(inputs.stock_setpoint_kph))
    if self._proposed is None:
      self._proposed = stock
      self._last_step = now
    # A new driver ceiling overrides the previous proposal immediately.
    self._proposed = min(self._proposed, stock)
    # Same two-second speed horizon as the offline RVV simulation. Positive
    # acceleration only returns gradually to the driver's existing ceiling.
    target = stock
    if inputs.requested_accel_ms2 is not None and inputs.requested_accel_ms2 < -0.05:
      target = math.floor(min(stock, inputs.speed_kph + inputs.requested_accel_ms2 * 2.0 * 3.6))
    if inputs.target_setpoint_kph is not None:
      target = min(target, math.floor(inputs.target_setpoint_kph))
    if target < MIN_SETPOINT_KPH:
      return self._block("target_below_rvv_envelope")
    target = min(MAX_SETPOINT_KPH, target)
    interval = DOWN_STEP_NS if inputs.requested_accel_ms2 is None and target < self._proposed else STEP_NS
    if self._last_step is not None and now - self._last_step >= interval:
      self._proposed += max(-1, min(1, target - self._proposed))
      self._last_step = now
    return RvvDecision("tracking", "shadow_only", self._proposed, target)
