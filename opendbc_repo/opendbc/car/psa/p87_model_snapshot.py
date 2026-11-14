"""Choose a lane snapshot that existed at the start of the control tick.

SubMaster is polled after the tick timestamp is captured. A newly received
model may therefore be a fraction of a tick newer than that timestamp.
Defer that frame, using only an already observed fresh past snapshot. Never
accept a future, stale or reversed frame as current lane authorization.
"""
MODEL_TIMEOUT_NS = 150_000_000
MAX_PUBLICATION_LEAD_NS = 20_000_000


class P87ModelSnapshot:
  def __init__(self):
    self.snapshot = (False, 0, None)
    self.mode = 'unavailable'
    self.deferrals = 0

  def select(self, now, valid, timestamp, model):
    previous_valid, previous_ns, previous_model = self.snapshot
    if timestamp > now:
      if (timestamp-now <= MAX_PUBLICATION_LEAD_NS and previous_ns > 0
          and 0 <= now-previous_ns <= MODEL_TIMEOUT_NS):
        self.mode = 'deferred_newer_publication'
        self.deferrals += 1
        return previous_valid, previous_ns, previous_model
      self.mode = 'future_without_fresh_snapshot'
      return False, timestamp, model
    if timestamp <= 0 or timestamp < previous_ns:
      self.mode = 'missing_or_reversed'
      return False, timestamp, model
    self.snapshot = (valid, timestamp, model)
    self.mode = 'current'
    return self.snapshot
