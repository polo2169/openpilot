"""Read-only straight-road gate for the explicit EPS cycle experiment."""
import math

from opendbc.car.psa.lateral_pause import lanes_ready

STABLE_NS = 500_000_000
MODEL_TIMEOUT_NS = 150_000_000
MAX_LAT_ACCEL = .10
PREDICTION_SECONDS = 2.


def straight_ready(car, model):
  try:
    if not lanes_ready(model) or not math.isfinite(car.yawRate * car.vEgo):
      return False
    if abs(car.yawRate * car.vEgo) > MAX_LAT_ACCEL:
      return False
    times, yaw, speeds = model.orientationRate.t, model.orientationRate.z, model.velocity.x
    if not 2 <= len(times) == len(yaw) == len(speeds):
      return False
    previous = None
    covered = False
    for t, rate, speed in zip(times, yaw, speeds, strict=True):
      if not all(math.isfinite(v) for v in (t, rate, speed)) or t < 0 or speed < 0:
        return False
      if previous is None:
        if t > .1:
          return False
      elif not 0 < t - previous <= .5:
        return False
      if abs(rate * speed) > MAX_LAT_ACCEL:
        return False
      previous = t
      if t >= PREDICTION_SECONDS:
        covered = True
        break
    return covered
  except (AttributeError, TypeError, ValueError):
    return False


class T9EpsCycleGate:
  def __init__(self):
    self.since = self.last_clock = self.last_model = 0
    self.ready = False

  def update(self, now, *, eligible, car, model, model_valid, model_ns):
    clock_ok = type(now) is int and now > 0 and (not self.last_clock or 0 <= now - self.last_clock <= 150_000_000)
    fresh = model_valid and 0 < model_ns <= now and now - model_ns <= MODEL_TIMEOUT_NS
    driver = float(car.steeringTorque)
    valid = (eligible and clock_ok and fresh and model_ns >= self.last_model
             and not car.leftBlinker and not car.rightBlinker and not car.steeringPressed
             and not car.psaLateralPaused and math.isfinite(driver) and abs(driver) <= 15
             and straight_ready(car, model))
    self.last_clock = now if type(now) is int and now > 0 else self.last_clock
    if not valid:
      self.since = self.last_model = 0
      self.ready = False
    else:
      if not self.since:
        self.since = now
      if model_ns > self.last_model and now - self.since >= STABLE_NS:
        self.ready = True
      self.last_model = model_ns
    return self.ready
