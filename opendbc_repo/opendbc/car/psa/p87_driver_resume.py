"""Released driver effort and fresh inputs for bounded P87 resumption."""
import math

DRIVER_RELEASE_RAW = 30
DRIVER_RELEASE_NS = 300_000_000
MODEL_TIMEOUT_NS = 150_000_000
OVERRIDE_TIMEOUT_NS = 2_000_000_000
RESUME_TIMEOUT_NS = 5_000_000_000
RESUME_COOLDOWN_NS = 5_000_000_000


class P87DriverResumeGate:
  def __init__(self):
    self.release_since = 0
    self.released = self.ready = False
    self.reason = 'not_pending'

  def update(self, now, *, pending, driver_torque, steering_pressed, eps_status_ok, model_valid, model_ns):
    self.released = math.isfinite(driver_torque) and abs(driver_torque) <= DRIVER_RELEASE_RAW and not steering_pressed
    self.ready = False
    if not pending:
      self.release_since = 0
      self.reason = 'not_pending'
      return False
    if not self.released:
      self.release_since = 0
    elif not self.release_since:
      self.release_since = now
    fresh = model_valid and 0 < model_ns <= now and now-model_ns <= MODEL_TIMEOUT_NS
    self.reason = ('driver_not_released' if not self.released else
                   'eps_status_not_ready' if not eps_status_ok else
                   'driver_release_stabilizing' if now-self.release_since < DRIVER_RELEASE_NS else
                   'model_stale_or_invalid' if not fresh else 'ready')
    self.ready = self.reason == 'ready'
    return self.ready
