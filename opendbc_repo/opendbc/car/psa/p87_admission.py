"""P87 zero-torque admission. An LKA request is distinct from an EPS ACK."""
import math

STABLE_NS = 300_000_000
MODEL_TIMEOUT_NS = 500_000_000
PREPARE_NS = 100_000_000
ACK_TIMEOUT_NS = 1_000_000_000


def lanes_ready(model):
  try:
    probs, lines = tuple(model.laneLineProbs), model.laneLines
    if len(probs) != 4 or len(lines) != 4 or any(not math.isfinite(p) or not 0 <= p <= 1 for p in probs):
      return False
    left, right = float(lines[1].y[0]), float(lines[2].y[0])
    return (min(probs[1:3]) >= .75 and math.isfinite(left) and math.isfinite(right)
            and left * right < 0 and 2.4 <= abs(right-left) <= 4.5
            and abs((left+right)/2) <= .75 and str(model.meta.laneChangeState) == 'off')
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


class P87Admission:
  def __init__(self):
    self.last_clock = 0
    self.reset()

  def reset(self):
    self.phase = 'standby'
    self.since = self.clear_since = self.last_model = self.models = 0
    self.request_ns = self.factor = 0
    self.blocked = False

  def block(self, reason):
    self.phase, self.blocked = reason, True
    self.factor = 0

  def update(self, now, *, requested, reason, eps, eps_ns, model_valid, model_ns, tick):
    clock_ok = type(now) is int and now > 0 and now >= self.last_clock
    gap = self.last_clock > 0 and now-self.last_clock > 150_000_000
    self.last_clock = now if clock_ok else self.last_clock
    if not requested:
      self.reset()
      return 'lateral_not_requested', 2, 0, False
    if not clock_ok or gap:
      self.block('control_update_timeout' if clock_ok else 'invalid_clock')
    if reason:
      if reason in ('driver_torque_override', 'driver_indicator', 'below_lateral_speed') and not self.blocked:
        self.reset()
        self.since = now
        self.phase = reason
      else:
        self.block(reason)
      return self.phase, 2, 0, False
    if self.blocked:
      return self.phase, 2, 0, False
    if self.phase == 'active':
      if eps != 3:
        self.block('eps_authorization_lost')
        return self.phase, 2, 0, False
      if tick:
        self.factor = min(100, self.factor+10)
      return self.phase, 4, self.factor, True

    fresh_model = model_valid and 0 < model_ns <= now and now-model_ns <= MODEL_TIMEOUT_NS
    if not fresh_model or model_ns < self.last_model:
      self.reset()
      self.since = now
      return 'waiting_valid_lanes', 2, 0, False
    if self.phase not in ('preparing_eps', 'waiting_for_eps_ack'):
      if not self.since:
        self.since = now
      if not self.clear_since:
        self.clear_since, self.last_model, self.models = now, model_ns, 1
      elif model_ns > self.last_model:
        self.last_model, self.models = model_ns, self.models+1
      if now-self.clear_since < STABLE_NS or self.models < 3:
        return 'validating_lanes', 2, 0, False
      if eps not in (1, 2) or eps_ns <= self.since:
        return 'waiting_eps_release', 2, 0, False
      if not tick:
        return 'validating_lanes', 2, 0, False
      self.phase, self.since, self.factor = 'preparing_eps', now, 10
    if self.phase == 'preparing_eps':
      if eps not in (1, 2):
        self.block('eps_changed_before_request')
        return self.phase, 2, 0, False
      if now-self.since < PREPARE_NS or not tick:
        return self.phase, 3, self.factor, False
      # Send state 4 while torque is ZERO. Waiting for EPS=3 before sending
      # state 4 creates a circular wait on the observed P87 protocol.
      self.phase, self.request_ns = 'waiting_for_eps_ack', now
    if now-self.request_ns >= ACK_TIMEOUT_NS:
      self.block('eps_ack_timeout')
      return self.phase, 2, 0, False
    if tick:
      self.factor = min(100, self.factor+10)
    if eps == 3 and eps_ns > self.request_ns:
      self.phase = 'active'
      return self.phase, 4, self.factor, True
    return self.phase, 4, self.factor, False
