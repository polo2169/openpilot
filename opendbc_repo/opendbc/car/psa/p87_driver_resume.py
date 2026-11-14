"""Released driver effort and fresh inputs for bounded P87 resumption."""
import math

DRIVER_RELEASE_RAW = 30
DRIVER_OVERRIDE_RAW = 45
DRIVER_RELEASE_NS = 300_000_000
MODEL_TIMEOUT_NS = 150_000_000
OVERRIDE_TIMEOUT_NS = 2_000_000_000
RESUME_TIMEOUT_NS = 5_000_000_000
RESUME_COOLDOWN_NS = 5_000_000_000


class P87DriverResumeGate:
  def __init__(self):
    self.release_since = 0
    self.release_confirmed = False
    self.released = self.ready = False
    self.reason = 'not_pending'

  def update(self, now, *, pending, driver_torque, steering_pressed, eps_status_ok, model_valid, model_ns):
    finite = math.isfinite(driver_torque)
    initial_release = finite and abs(driver_torque) <= DRIVER_RELEASE_RAW and not steering_pressed
    override = not finite or abs(driver_torque) > DRIVER_OVERRIDE_RAW or steering_pressed
    self.released = initial_release
    self.ready = False
    if not pending:
      self.release_since = 0
      self.release_confirmed = False
      self.reason = 'not_pending'
      return False
    if override or (not self.release_confirmed and not initial_release):
      self.release_since = 0
      self.release_confirmed = False
    elif not self.release_since:
      self.release_since = now
    if self.release_since and now-self.release_since >= DRIVER_RELEASE_NS:
      self.release_confirmed = True
    # Once the required low effort has been stable, normal light contact
    # between the release and override thresholds must not undo it. Actual
    # override/steeringPressed still withdraws readiness immediately.
    self.released = initial_release or (self.release_confirmed and not override)
    fresh = model_valid and 0 < model_ns <= now and now-model_ns <= MODEL_TIMEOUT_NS
    self.reason = ('driver_not_released' if not self.released else
                   'eps_status_not_ready' if not eps_status_ok else
                   'driver_release_stabilizing' if not self.release_confirmed else
                   'model_stale_or_invalid' if not fresh else 'ready')
    self.ready = self.reason == 'ready'
    return self.ready
