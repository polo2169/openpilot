import os
import unittest
from unittest.mock import patch

from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.lateral_test import enabled as lateral_enabled
from opendbc.car.psa import rvv_wire
from opendbc.car.psa.tests import test_lateral_test as lateral_tests
from opendbc.car.psa.values import CAR


class TestCombinedInterface(unittest.TestCase):
  def setUp(self):
    self.h = lateral_tests.TestT9LateralIntegration()
    self.h.setUp()
    self.addCleanup(self.h.doCleanups)
    with patch.dict(os.environ, {'PSA_T9_RVV_TEST': '1', 'PSA_T9_LATERAL_TEST': '1'}):
      self.cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.h.cp = self.cp
    self.h.ci = CarInterface(self.cp)
    self.h.safety.set_safety_hooks(31, 0x1312)
    self.h.safety.init_tests()

  def test_combined_keeps_existing_lateral_tuning_and_emits_only_steering(self):
    self.assertTrue(lateral_enabled(self.cp))
    self.assertTrue(rvv_wire.enabled(self.cp))
    self.assertFalse(rvv_wire.only(self.cp))
    self.assertAlmostEqual(self.cp.minEnableSpeed*3.6, 67.1, places=4)
    with patch.dict(os.environ, {'PSA_T9_RVV_TEST': '0', 'PSA_T9_LATERAL_TEST': '1'}):
      lateral = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.assertEqual(self.cp.lateralTuning.to_dict(), lateral.lateralTuning.to_dict())
    for ms in range(0, 4001, 10):
      self.h.step(ms)
    self.assertEqual(self.h.output[-1][1], 15)
    self.assertTrue(self.h.safety.get_controls_allowed())
    self.h.step(4001, driver=16)
    self.assertTrue(self.h.ci.CS.out.steeringDisengage)
    self.assertFalse(self.h.safety.get_controls_allowed())

  def test_driver_boundary_matches_native_state_and_panda(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.setUp()
        for ms in range(0, 4001, 10):
          self.h.step(ms)
        for ms in range(4010, 4251, 10):
          applied, _ = self.h.step(ms, driver=sign * 15)
          self.assertEqual(applied.torqueOutputCan, 15)
          self.assertFalse(self.h.ci.CS.out.steeringPressed)
          self.assertFalse(self.h.ci.CS.out.steeringDisengage)
          self.assertTrue(self.h.safety.get_controls_allowed())
        applied, sends = self.h.step(4251, driver=sign * 16)
        self.assertEqual(applied.torqueOutputCan, 0)
        self.assertEqual(len(sends), 1)
        self.assertTrue(self.h.ci.CS.out.steeringDisengage)
        self.assertFalse(self.h.safety.get_controls_allowed())

  def test_combined_controller_and_panda_agree_at_140(self):
    for ms in range(0, 4001, 10):
      self.h.step(ms, speed=140, stock_setpoint=140)
    self.assertEqual(self.h.output[-1][1], 15)
    self.assertTrue(self.h.safety.get_controls_allowed())
    applied, sends = self.h.step(4001, speed=140.01, stock_setpoint=140)
    self.assertEqual(applied.torqueOutputCan, 0)
    self.assertEqual(len(sends), 1)
    self.assertFalse(self.h.safety.get_controls_allowed())


if __name__ == '__main__':
  unittest.main()
