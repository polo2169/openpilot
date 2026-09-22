import unittest

from opendbc.safety.tests import test_psa_t9 as lateral_tests


class TestT9CombinedSafety(lateral_tests.TestT9LateralSafety):
  """Run the lateral regression suite against the combined authorization."""
  def setUp(self):
    super().setUp()
    self.safety.set_safety_hooks(31, 0x1312)
    self.safety.init_tests()
    for _ in range(45):
      self.feed()
    self.sequence = 0

  def request(self, target):
    self.sequence += 1
    return self.safety.test_t9_rvv_request(0x1100 | target, self.sequence)

  def forwarded(self):
    frame = self.stock_rvv(True, setpoint=self.settings['setpoint'])
    destination = self.safety.safety_fwd_hook(2, 0x50E)
    rewritten = self.safety.test_t9_rvv_rewrite(frame, destination)
    return rewritten, bytes(frame[0].data[0:8])

  def test_direction_and_setpoint_reduction_share_the_manual_engagement(self):
    self.engage()
    values = []
    for _ in range(24):
      self.assertTrue(self.tx(1))
      self.assertEqual(self.request(73), 0)
      changed, data = self.forwarded()
      self.assertTrue(changed)
      values.append(data[6])
      self.feed()
    self.assertEqual(values[0], 75)
    self.assertEqual(values[-1], 73)

  def test_driver_eps_brake_and_lower_speed_stop_both_without_resume(self):
    for change in ({'driver': 16}, {'driver': -16}, {'brake': True}, {'pedal': 1},
                   {'eps': 0}, {'eps': 4}, {'speed': 67.0}):
      with self.subTest(change=change):
        self.setUp(); self.engage()
        self.assertTrue(self.tx(1))
        self.assertEqual(self.request(74), 0)
        self.assertTrue(self.forwarded()[0])
        before = self.settings.copy()
        self.feed(**change)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.forwarded()[0])
        self.assertTrue(self.tx(0, 2, 0))
        self.feed(**(before | {'eps': 1}))
        self.assertNotEqual(self.request(74), 0)
        self.assertFalse(self.tx(1))

  def test_rvv_timeout_or_release_cannot_leave_lateral_authorized(self):
    for timeout in (False, True):
      with self.subTest(timeout=timeout):
        self.setUp(); self.engage()
        self.assertTrue(self.tx(1)); self.assertEqual(self.request(74), 0)
        self.assertTrue(self.forwarded()[0])
        if timeout:
          for _ in range(4):
            self.feed(); self.assertTrue(self.tx(1))
          self.assertFalse(self.forwarded()[0])
        else:
          self.assertEqual(self.request(0), 0)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.tx(1))
        self.assertTrue(self.tx(0, 2, 0))

  def test_probe_never_emits_or_rewrites(self):
    self.safety.set_safety_hooks(31, 0x1313)
    for _ in range(45):
      self.feed(cruise=True)
    self.assertFalse(self.tx(0, 3, 0))
    self.assertNotEqual(self.request(74), 0)
    self.assertFalse(self.forwarded()[0])

  def test_combined_upper_speed_and_setpoint_boundary(self):
    self.feed(speed=140, setpoint=140)
    self.engage()
    self.assertEqual(self.request(140), 0)
    values = []
    for _ in range(14):
      self.assertTrue(self.tx(1))
      self.assertEqual(self.request(139), 0)
      changed, data = self.forwarded()
      self.assertTrue(changed)
      values.append(data[6])
      self.feed()
    self.assertEqual(values[0], 140)
    self.assertEqual(values[-1], 139)
    self.feed(speed=140.01)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.forwarded()[0])
    self.assertTrue(self.tx(0, 2, 0))
    self.feed(speed=140, eps=1)
    self.assertNotEqual(self.request(139), 0)
    self.assertFalse(self.tx(0, 3, 0))


if __name__ == '__main__':
  unittest.main()
