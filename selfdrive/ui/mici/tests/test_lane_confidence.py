import math
from types import SimpleNamespace
import unittest

from openpilot.selfdrive.ui.mici.onroad.lane_confidence import lane_probabilities


class ModelState(dict):
  def __init__(self):
    super().__init__(modelV2=SimpleNamespace(laneLineProbs=[0.1, 0.87, 0.32, 0.2]))
    self.recv_frame = {'modelV2': 20}
    self.recv_time = {'modelV2': 100.0}
    self.valid = {'modelV2': True}
    self.alive = {'modelV2': True}


class TestLaneConfidence(unittest.TestCase):
  def test_measured_probabilities_and_real_zero(self):
    sm = ModelState()
    self.assertEqual(lane_probabilities(sm, 10, now=100.1)[1:3], (0.87, 0.32))
    sm['modelV2'].laneLineProbs = [0, 0, 0, 0]
    self.assertEqual(lane_probabilities(sm, 10, now=100.1), (0, 0, 0, 0))

  def test_stale_invalid_and_previous_drive_are_unavailable(self):
    for field, value in [('valid', False), ('alive', False), ('recv_frame', 9), ('recv_time', 99.5), ('recv_time', 101)]:
      with self.subTest(field=field, value=value):
        sm = ModelState()
        getattr(sm, field)['modelV2'] = value
        self.assertIsNone(lane_probabilities(sm, 10, now=100.1))

  def test_bad_probabilities_are_not_clamped_to_confident_values(self):
    for values in [[], [0.9]*3, [0.9]*5, [0, math.nan, 1, 0], [0, 1, math.inf, 0], [0, -0.01, 1, 0], [0, 1, 1.01, 0]]:
      with self.subTest(values=values):
        sm = ModelState()
        sm['modelV2'].laneLineProbs = values
        self.assertIsNone(lane_probabilities(sm, 10, now=100.1))


if __name__ == '__main__':
  unittest.main()
