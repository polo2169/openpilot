"""P87 planned renewal opportunities requiring real native hands feedback.

Ten seconds is an experimental scheduling interval, not an observed EPS
deadline. Defer in bends or without a fresh clear corridor. Require a stable
native hands indication before renewal; never recover an unsolicited dropout.
"""
import math

PERIOD_NS = 10_000_000_000
RELEASE_TIMEOUT_NS = 1_000_000_000
TOTAL_TIMEOUT_NS = 3_000_000_000
MODEL_TIMEOUT_NS = 150_000_000
STABLE_NS = 300_000_000
HANDS_STABLE_NS = 1_000_000_000
MAX_LAT_ACCEL = .30
MAX_START_TORQUE = 15
RAMP_DOWN = 4  # raw counts per 50 ms, within the existing native slew limits


def native_hands_ok(state):
  signals = getattr(state, 'is_dat_dira', {})
  return (signals.get('STEERWHL_HOLD_BY_DRV') == 1 and
          signals.get('TRQ_LIMIT_STATE') == 0 and
          signals.get('STEERING_REBOOT_REQUEST') == 0)


def straight_corridor(car, model):
  try:
    if not math.isfinite(car.vEgo) or car.vEgo <= 0:
      return False
    current = car.yawRate * car.vEgo
    desired = model.action.desiredCurvature * car.vEgo ** 2
    if any(not math.isfinite(v) or abs(v) > MAX_LAT_ACCEL for v in (current, desired)):
      return False
    times, rates, speeds = model.orientationRate.t, model.orientationRate.z, model.velocity.x
    if not 2 <= len(times) == len(rates) == len(speeds):
      return False
    previous = None
    for t, rate, speed in zip(times, rates, speeds, strict=True):
      if not all(math.isfinite(v) for v in (t, rate, speed)) or t < 0 or speed < 0:
        return False
      if (previous is None and t > .1) or (previous is not None and not 0 < t-previous <= .5):
        return False
      previous = t
      if abs(rate * speed) > MAX_LAT_ACCEL:
        return False
      if t >= 5.:
        return True
    return False
  except (AttributeError, TypeError, ValueError):
    return False


class P87EpsCycleGate:
  def __init__(self):
    self.hands_since = 0
    self.clear_since = self.last_model = self.models = 0
    self.ready = False
    self.reason = 'not_active'

  def update(self, now, *, requested, active, eligible, hands_ok, car, model, model_valid, model_ns):
    if not requested or not active or not hands_ok:
      self.hands_since = 0
    elif not self.hands_since:
      self.hands_since = now
    hands_confirmed = self.hands_since > 0 and now-self.hands_since >= HANDS_STABLE_NS
    fresh = model_valid and 0 < model_ns <= now and now-model_ns <= MODEL_TIMEOUT_NS
    self.reason = ('not_active' if not active or not requested else
                   'hands_not_confirmed' if not hands_ok else
                   'validating_native_hands' if not hands_confirmed else
                   'below_cycle_speed' if not math.isfinite(car.vEgoRaw) or car.vEgoRaw * 3.6 < 50. else
                   'ineligible' if not eligible else
                   'model_stale_or_invalid' if not fresh or model_ns < self.last_model else
                   'curve_or_prediction_invalid' if not straight_corridor(car, model) else None)
    self.ready = False
    if self.reason is not None:
      self.clear_since = self.last_model = self.models = 0
    else:
      if not self.clear_since:
        self.clear_since, self.last_model, self.models = now, model_ns, 1
      elif model_ns > self.last_model:
        self.last_model, self.models = model_ns, self.models+1
      self.ready = now-self.clear_since >= STABLE_NS and self.models >= 3
      self.reason = 'ready' if self.ready else 'validating_straight'
    return self.ready
