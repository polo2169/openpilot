import os
import unittest
from unittest.mock import patch

from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.rvv_wire import enabled
from opendbc.car.psa.tests import test_lateral_test as lateral_tests
from opendbc.car.psa.values import CAR


class TestRvvInterface(unittest.TestCase):
  def setUp(self):
    self.h = lateral_tests.TestT9LateralIntegration()
    self.h.setUp()
    self.addCleanup(self.h.doCleanups)
    with patch.dict(os.environ, {'PSA_T9_RVV_TEST': '1'}):
      self.cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.h.cp = self.cp
    self.h.ci = CarInterface(self.cp)
    self.h.safety.set_safety_hooks(31, 0x1310)
    self.h.safety.init_tests()

  def test_rvv_only_splits_vehicle_inputs_without_producing_steering_or_raw_can(self):
    self.assertTrue(enabled(self.cp))
    self.assertAlmostEqual(self.cp.minEnableSpeed * 3.6, 40., places=4)
    self.assertFalse(self.cp.openpilotLongitudinalControl)
    self.assertIsNone(self.h.ci.CC.t9_lateral)
    for ms in range(0, 3000, 10):
      applied, sends = self.h.step(ms, speed=40, eps=0, driver=12)
      self.assertEqual(sends, [])
      self.assertEqual(applied.torqueOutputCan, 0)
    self.assertTrue(self.h.ci.CS.out.canValid)
    self.assertTrue(self.h.ci.CS.out.cruiseState.enabled)
    self.assertAlmostEqual(self.h.ci.CS.out.cruiseState.speed * 3.6, 75., places=4)
    self.assertTrue(self.h.safety.get_controls_allowed())

  def test_engine_active_alone_cannot_enable_and_original_ceiling_is_visible(self):
    for ms in range(0, 2500, 10):
      self.h.step(ms, requested=False)
    self.h.step(2500, requested=True, stock_active=False)
    self.assertFalse(self.h.ci.CS.out.cruiseState.enabled)
    self.h.step(2510, requested=True, stock_active=True)
    self.assertTrue(self.h.ci.CS.out.cruiseState.enabled)
    self.h.step(2520, requested=False, stock_active=False)
    self.assertFalse(self.h.ci.CS.out.cruiseState.enabled)


  def test_accelerator_override_does_not_masquerade_as_manual_rearm(self):
    for ms in range(0, 2500, 10):
      self.h.step(ms)
    self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
    self.h.step(2500, requested=True, pedal=10, engine_state=1)
    self.h.step(2510, requested=True, pedal=0, engine_state=2)
    self.assertTrue(self.h.ci.CS.out.cruiseState.enabled)
    self.assertTrue(self.h.ci.CS.out.blockPcmEnable)
    self.assertFalse(self.h.safety.get_controls_allowed())
    self.h.step(2520, requested=False)
    self.h.step(2530, requested=True)
    self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
    self.assertTrue(self.h.safety.get_controls_allowed())


if __name__ == '__main__':
  unittest.main()
