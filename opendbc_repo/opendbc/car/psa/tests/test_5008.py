import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.psacan import create_p87_lka
from opendbc.car.psa.values import CAR, PSA_5008_LATERAL_PARAM


class TestP87Interface(unittest.TestCase):
  def interface(self, lateral=False):
    with patch.dict(os.environ, {'PSA_5008_LATERAL_TEST': '1' if lateral else '0'}):
      cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_5008_P87)
    sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_5008_P87)
    return CarInterface(cp, sp)

  def state(self, **changes):
    state = structs.CarState()
    state.canValid = True
    state.cruiseState.enabled = True
    cs = SimpleNamespace(out=state, speed_kph=60, eps_active=True, eps_state_lka=3,
                         p87_lka_template=bytes.fromhex('a1450d0008000003'), p87_lka_timestamp=1_000_000_000)
    for key, value in changes.items():
      if hasattr(state, key):
        setattr(state, key, value)
      else:
        setattr(cs, key, value)
    return cs

  def control(self):
    cc = structs.CarControl()
    cc.latActive = True
    cc.actuators.torque = 1.0
    return cc.as_reader()

  def test_observation_never_sends_even_when_lat_active(self):
    ci = self.interface()
    self.assertTrue(ci.CP.dashcamOnly)
    self.assertEqual(ci.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    act, sends = ci.CC.update(self.control(), structs.CarControlSP(), self.state(), 1_000_000_000)
    self.assertEqual(sends, [])
    self.assertEqual(act.torqueOutputCan, 0)

  def test_explicit_lateral_uses_its_own_safety_profile(self):
    ci = self.interface(True)
    self.assertFalse(ci.CP.dashcamOnly)
    self.assertEqual(ci.CP.safetyConfigs[0].safetyParam, PSA_5008_LATERAL_PARAM)
    self.assertFalse(ci.CP.openpilotLongitudinalControl)
    self.assertFalse(ci.CP_SP.intelligentCruiseButtonManagementAvailable)

  def test_existing_3008_profile_is_unchanged(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    self.assertFalse(cp.dashcamOnly)
    self.assertEqual(cp.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.psa)
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 0)

  def test_unavailable_wheel_speeds_are_not_displayed_as_655_kph(self):
    for data in (b'\xff' * 8, b'\xff\xfe' * 4):
      ci = self.interface()
      ret, _ = ci.update([(1_000_000_000, [(0x30D, data, 0)])])
      self.assertEqual(ret.vEgoRaw, 0.0)
      self.assertFalse(ret.canValid)

  def test_lateral_only_emits_native_template_lka(self):
    ci = self.interface(True)
    act, sends = ci.CC.update(self.control(), structs.CarControlSP(), self.state(), 1_000_000_000)
    self.assertEqual([(s.address, s.src) for s in sends], [(0x3F2, 0)])
    self.assertEqual(act.torqueOutputCan, 8)
    self.assertEqual(sends[0].dat[:3], bytes.fromhex('a1450d'))

  def test_missing_or_stale_template_sends_nothing(self):
    for changes in ({'p87_lka_template': None}, {'p87_lka_timestamp': 1}, {'p87_lka_timestamp': 2_000_000_000}):
      with self.subTest(changes=changes):
        ci = self.interface(True)
        act, sends = ci.CC.update(self.control(), structs.CarControlSP(), self.state(**changes), 1_000_000_000)
        self.assertEqual(sends, [])
        self.assertEqual(act.torqueOutputCan, 0)

  def test_fault_pedals_and_speed_inhibit_torque(self):
    for changes in ({'accFaulted': True}, {'brakePressed': True}, {'gasPressed': True}, {'canValid': False},
                    {'canTimeout': True}, {'doorOpen': True}, {'seatbeltUnlatched': True},
                    {'parkingBrake': True}, {'speed_kph': 51}, {'speed_kph': 141}):
      with self.subTest(changes=changes):
        ci = self.interface(True)
        act, sends = ci.CC.update(self.control(), structs.CarControlSP(), self.state(**changes), 1_000_000_000)
        self.assertEqual(act.torqueOutputCan, 0)
        self.assertEqual(sends[0].dat[3:7], bytes.fromhex('00080000'))

  def test_torque_waits_for_eps_ack(self):
    ci = self.interface(True)
    act, sends = ci.CC.update(self.control(), structs.CarControlSP(), self.state(eps_active=False, eps_state_lka=2), 1_000_000_000)
    self.assertEqual(act.torqueOutputCan, 0)
    self.assertEqual((sends[0].dat[4] >> 2) & 7, 3)

  def test_template_preserves_unknown_bits_and_never_requests_angle(self):
    template = bytes.fromhex('a1450d33ff0133ff')
    msg = create_p87_lka(template, -8, 100, 4)
    self.assertEqual(msg.dat[:3], template[:3])
    self.assertEqual(msg.dat[4] & 3, template[4] & 3)
    self.assertEqual(msg.dat[5] & 1, template[5] & 1)
    self.assertEqual(msg.dat[6], 0)
    self.assertEqual(msg.dat[7], template[7] & 3)


if __name__ == '__main__':
  unittest.main()
