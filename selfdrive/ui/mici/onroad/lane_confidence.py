"""Lane-presence probabilities from modelV2, not a driving safety score."""
import math
import time

MODEL_TIMEOUT = 0.5


def lane_probabilities(sm, started_frame, now=None):
  now = time.monotonic() if now is None else now
  if (sm.recv_frame['modelV2'] < started_frame or not sm.valid['modelV2'] or not sm.alive['modelV2'] or
      not 0 <= now - sm.recv_time['modelV2'] <= MODEL_TIMEOUT):
    return None
  probs = tuple(sm['modelV2'].laneLineProbs)
  if len(probs) != 4 or any(not math.isfinite(p) or not 0 <= p <= 1 for p in probs):
    return None
  return probs


def draw_lane_confidence(rect, probs):
  # Lazy UI imports also allow offline verification without a display or sockets.
  import pyray as rl
  from openpilot.system.ui.lib.application import FontWeight, gui_app

  box = rl.Rectangle(rect.x + rect.width - 220, rect.y + 10, 206, 51)
  rl.draw_rectangle_rounded(box, 0.2, 6, rl.Color(0, 0, 0, 175))
  font = gui_app.font(FontWeight.MEDIUM)
  rl.draw_text_ex(font, 'Confiance lignes', rl.Vector2(box.x + 10, box.y + 5), 15, 0, rl.LIGHTGRAY)
  values = 'G --     D --' if probs is None else f'G {probs[1]:.0%}     D {probs[2]:.0%}'
  rl.draw_text_ex(font, values, rl.Vector2(box.x + 10, box.y + 25), 19, 0, rl.WHITE)
