"""P87 zero-torque admission. An LKA request is distinct from an EPS ACK."""
import math
from opendbc.car.psa.p87_eps_cycle import PERIOD_NS, RELEASE_TIMEOUT_NS, TOTAL_TIMEOUT_NS

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
    self.cycle_count = 0
    self.reset()

  def reset(self):
    self.phase = 'standby'
    self.since = self.clear_since = self.last_model = self.models = 0
    self.request_ns = self.factor = 0
    self.blocked = False
    self.active_since = self.cycle_since = 0
    self.cycling = False

  def block(self, reason):
    if not self.blocked:
      self.phase = reason
    self.blocked = True
    self.factor = 0
    self.cycling = False

  def update(self, now, *, requested, reason, eps, eps_ns, model_valid, model_ns, tick,
             cycle_ready=False, cycle_resume_ready=True):
    clock_ok = type(now) is int and now > 0 and now >= self.last_clock
    gap = self.last_clock > 0 and now-self.last_clock > 150_000_000
    self.last_clock = now if clock_ok else self.last_clock
    if not requested:
      self.reset()
      return 'lateral_not_requested', 2, 0, False
    if not clock_ok or gap:
      self.block('control_update_timeout' if clock_ok else 'invalid_clock')
    if self.cycling and now-self.cycle_since >= TOTAL_TIMEOUT_NS:
      self.block('eps_cycle_timeout')
    # EPS=0 can be a consequence of our own neutral release. It never starts
    # a cycle and cannot authorize preparation or torque. Fault states still veto.
    if self.cycling and self.phase != 'waiting_for_eps_ack' and reason == 'eps_not_authorized' and eps == 0:
      reason = None
    if reason:
      if reason in ('driver_torque_override', 'driver_indicator', 'below_lateral_speed') and not self.blocked and not self.cycling:
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
      if tick and cycle_ready and self.active_since and now-self.active_since >= PERIOD_NS:
        self.phase, self.cycle_since, self.since = 'cycle_releasing', now, now
        self.clear_since = self.last_model = self.models = self.request_ns = self.factor = 0
        self.cycling = True
        return self.phase, 2, 0, False
      if tick:
        self.factor = min(100, self.factor+10)
      return self.phase, 4, self.factor, True

    if self.phase == 'cycle_releasing':
      if now-self.cycle_since >= RELEASE_TIMEOUT_NS:
        self.block('eps_cycle_release_timeout')
        return self.phase, 2, 0, False
      if eps not in (1, 2) or eps_ns <= self.cycle_since:
        return self.phase, 2, 0, False
      self.phase = 'cycle_waiting_corridor'

    # After an intentional neutral release, short prediction/lane changes
    # postpone preparation at ZERO torque. The original total deadline and
    # every hard gate still apply. Never restart an already sent ACK request.
    if self.cycling and (not cycle_resume_ready or eps == 0) and self.phase != 'waiting_for_eps_ack':
      self.phase = 'cycle_waiting_corridor'
      self.clear_since = self.last_model = self.models = self.factor = 0
      return self.phase, 2, 0, False
    if self.phase == 'cycle_waiting_corridor':
      self.phase = 'validating_lanes'

    fresh_model = model_valid and 0 < model_ns <= now and now-model_ns <= MODEL_TIMEOUT_NS
    if not fresh_model or model_ns < self.last_model:
      if self.cycling:
        self.block('eps_cycle_model_invalid')
        return self.phase, 2, 0, False
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
      if self.cycling:
        self.cycle_count += 1
      self.active_since, self.cycling = now, False
      self.phase = 'active'
      return self.phase, 4, self.factor, True
    return self.phase, 4, self.factor, False
