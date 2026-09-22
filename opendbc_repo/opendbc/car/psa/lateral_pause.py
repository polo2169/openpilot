"""Temporary lateral suspension within an already enabled T9 session."""
import math

DRIVER_PAUSE_RAW = 15
RESUME_STABLE_NS = 500_000_000
MODEL_TIMEOUT_NS = 500_000_000
LANE_PROBABILITY_MIN = 0.75


def lanes_ready(model):
  """Require two plausible, confident boundaries around the vehicle.

  Model probabilities are not a calibrated safety score. This is a resume
  gate only; it never grants EPS or driver-monitoring authorization.
  """
  try:
    probs = tuple(model.laneLineProbs)
    lines = model.laneLines
    if len(probs) != 4 or len(lines) != 4 or any(not math.isfinite(p) or not 0 <= p <= 1 for p in probs):
      return False
    left, right = float(lines[1].y[0]), float(lines[2].y[0])
    width = abs(right - left)
    return (min(probs[1:3]) >= LANE_PROBABILITY_MIN and math.isfinite(left) and math.isfinite(right)
            and left * right < 0 and 2.4 <= width <= 4.5 and abs((left + right) / 2) <= 0.75
            and str(model.meta.laneChangeState) == 'off')
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


class T9LateralPause:
  def __init__(self):
    self.paused = False
    self.clear_since = 0
    self.last_clock = 0
    self.last_model = 0
    self.resume_requested = False

  def update(self, now, *, eligible, car, model, model_valid, model_ns):
    clock_valid = type(now) is int and now > 0 and now >= self.last_clock
    update_gap = clock_valid and self.last_clock > 0 and now - self.last_clock > 150_000_000
    self.last_clock = now if clock_valid else self.last_clock
    if not eligible:
      self.paused = False
      self.resume_requested = False
      self.clear_since = self.last_model = 0
      return False
    driver = float(car.steeringTorque)
    trigger = (car.leftBlinker or car.rightBlinker or car.steeringPressed
               or not math.isfinite(driver) or abs(driver) > DRIVER_PAUSE_RAW)
    if not car.psaLateralPaused:
      self.resume_requested = False
    if car.psaLateralPaused and not self.resume_requested:
      self.paused = True
    if trigger or not clock_valid:
      self.paused = True
      self.resume_requested = False
      self.clear_since = self.last_model = 0
    elif update_gap:
      self.clear_since = self.last_model = 0
    fresh = model_valid and 0 < model_ns <= now and now - model_ns <= MODEL_TIMEOUT_NS
    if self.resume_requested and (not fresh or not lanes_ready(model) or update_gap):
      self.resume_requested = False
      self.paused = True
      self.clear_since = self.last_model = 0
    if not self.paused:
      return False
    if trigger or not clock_valid or not fresh or not lanes_ready(model) or model_ns < self.last_model:
      self.clear_since = 0
      self.last_model = 0
      return True
    # A single good model held in memory cannot satisfy the stability window.
    if not self.clear_since:
      self.clear_since = now
      self.last_model = model_ns
    elif model_ns > self.last_model:
      self.last_model = model_ns
      if now - self.clear_since >= RESUME_STABLE_NS:
        self.paused = False
        self.resume_requested = True
        self.clear_since = 0
    return self.paused
