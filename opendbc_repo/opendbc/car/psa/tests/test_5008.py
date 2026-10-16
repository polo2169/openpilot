import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import structs, Bus
from opendbc.can.packer import CANPacker
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
                         p87_lka_template=bytes.fromhex('a1450d0008000003'), p87_lka_timestamp=1_000_000_000, p87_eps_timestamp=1_000_000_000)
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
    self.assertTrue(ci.CP.pcmCruise)
    self.assertAlmostEqual(ci.CP.minSteerSpeed * 3.6, 30., places=4)
    self.assertFalse(ci.CP.autoResumeSng)
    self.assertFalse(ci.CP_SP.intelligentCruiseButtonManagementAvailable)

  def test_existing_3008_profile_is_unchanged(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    self.assertFalse(cp.dashcamOnly)
    self.assertEqual(cp.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.psa)
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 0)
    ci = CarInterface(cp, CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008))
    self.assertEqual(ci.can_parsers[Bus.main].dbc.addr_to_msg[0x228].size, 5)

  def test_recorded_released_pedal_with_positive_acc_engine_demand(self):
    ci = self.interface()
    packer = CANPacker('psa_p87_generated')
    ret, _ = ci.update([(1_000_000_000, [
      (0x228, bytes.fromhex('4936004100c52032'), 0),
      packer.make_can_msg('Dyn_CMM', 0, {'P002_Com_rAPP': 19.5}),
    ])])
    self.assertTrue(ci.CS.p87_pedal_valid)
    self.assertFalse(ret.gasPressed)

  def test_real_driver_pedal_is_detected_without_engine_demand(self):
    ci = self.interface()
    packer = CANPacker('psa_p87_generated')
    ret, _ = ci.update([(1_000_000_000, [packer.make_can_msg('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0.5})])])
    self.assertTrue(ci.CS.p87_pedal_valid)
    self.assertTrue(ret.gasPressed)
    self.assertEqual(ci.can_parsers[Bus.main].vl['Dyn5_CMM']['P334_ACCPed_Position'], 0.5)

  def test_cruise_armed_active_suspended_and_faulted(self):
    ci = self.interface(True)
    packer = CANPacker('psa_p87_generated')
    for i, acc in enumerate((3, 4, 5, 6, 7, 2, 15)):
      with self.subTest(acc=acc):
        ret, _ = ci.update([(1_000_000_000 + i * 10_000_000, [
          packer.make_can_msg('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0}),
          packer.make_can_msg('Dyn2_CMM', 0, {'P372_T15_st': 1, 'P134_Com_stEng': 3}),
          packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, {'RVV_ACC_ACTIVATION_REQ': 1, 'LONGITUDINAL_REGULATION_TYPE': 3}),
          packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, {'ACC_STATUS': acc}),
        ])])
        self.assertEqual(ret.cruiseState.enabled, acc in (4, 6))
        self.assertEqual(ret.cruiseState.available, acc in (3, 4, 5, 6, 7))
        self.assertEqual(ret.accFaulted, acc == 15)
        self.assertFalse(ret.blockPcmEnable)
        self.assertFalse(ret.carNotReady)

  def test_pedals_block_enable_and_brake_fault_is_preserved(self):
    ci = self.interface(True)
    packer = CANPacker('psa_p87_generated')
    ret, _ = ci.update([(1_000_000_000, [
      packer.make_can_msg('Dyn5_CMM', 0, {'P334_ACCPed_Position': 5}),
      packer.make_can_msg('HS2_DYN_UCF_MDD_32D', 1, {'DEFAUT_ACC_FREIN': 1}),
      packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, {'ACC_STATUS': 4}),
      packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, {'RVV_ACC_ACTIVATION_REQ': 1}),
    ])])
    self.assertTrue(ret.blockPcmEnable)
    self.assertTrue(ret.accFaulted)
    self.assertFalse(ret.cruiseState.enabled)
    self.assertTrue(ret.carNotReady)

  def test_fault_clearing_does_not_create_a_new_host_enable(self):
    ci = self.interface(True)
    packer = CANPacker('psa_p87_generated')
    for i, (request, fault, blocked) in enumerate(((1, 0, False), (1, 1, True), (1, 0, True), (0, 0, False), (1, 0, False))):
      ret, _ = ci.update([(1_000_000_000 + i * 10_000_000, [
        packer.make_can_msg('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0}),
        packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, {'RVV_ACC_ACTIVATION_REQ': request}),
        packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, {'ACC_STATUS': 4}),
        packer.make_can_msg('HS2_DYN_UCF_MDD_32D', 1, {'DEFAUT_ACC_FREIN': fault}),
      ])])
      self.assertEqual(ret.blockPcmEnable, blocked)

  def test_frozen_pedal_counter_invalidates_can(self):
    ci = self.interface()
    packer = CANPacker('psa_p87_generated')
    pedal = packer.make_can_msg('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0})
    for i in range(7):
      ret, _ = ci.update([(1_000_000_000 + i * 10_000_000, [pedal])])
    self.assertFalse(ret.canValid)
    self.assertGreaterEqual(ci.can_parsers[Bus.main].message_states[0x228].counter_fail, 5)

  def test_missing_malformed_or_stale_pedal_is_never_released(self):
    for bus, data in ((0, b'\x00' * 8), (0, bytes.fromhex('4936004100')), (2, bytes.fromhex('4936004100c52032'))):
      with self.subTest(bus=bus, data=data):
        ci = self.interface()
        ret, _ = ci.update([(1_000_000_000, [(0x228, data, bus)])])
        self.assertFalse(ci.CS.p87_pedal_valid)
        self.assertTrue(ret.gasPressed)
        self.assertFalse(ret.canValid)
    ci = self.interface()
    ci.update([(1_000_000_000, [(0x228, bytes.fromhex('4936004100c52032'), 0)])])
    ret, _ = ci.update([(1_250_000_001, [])])
    self.assertFalse(ci.CS.p87_pedal_valid)
    self.assertTrue(ret.gasPressed)
    self.assertFalse(ret.canValid)

  def test_corruption_and_reserved_value_invalidate_a_previous_release(self):
    for reserved in (False, True):
      ci = self.interface()
      packer = CANPacker('psa_p87_generated')
      ci.update([(1_000_000_000, [packer.make_can_msg('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0})])])
      msg = packer.make_can_msg('Dyn5_CMM', 0, {'P334_ACCPed_Position': 127.5 if reserved else 0})
      data = bytearray(msg[1])
      if not reserved:
        data[3] ^= 1
      ret, _ = ci.update([(1_010_000_000, [(0x228, bytes(data), 0)])])
      self.assertFalse(ci.CS.p87_pedal_valid)
      self.assertTrue(ret.gasPressed)
      self.assertFalse(ret.canValid)

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
                    {'parkingBrake': True}, {'speed_kph': 29.99}, {'speed_kph': 141}):
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

  def test_thirty_kph_still_needs_real_fresh_eps_authorization(self):
    for speed in (30., 40., 51.99):
      for eps in (0, 1, 2, 3, 4, 7):
        with self.subTest(speed=speed, eps=eps):
          ci = self.interface(True)
          act, sends = ci.CC.update(self.control(), structs.CarControlSP(),
            self.state(speed_kph=speed, eps_state_lka=eps, eps_active=eps==3), 1_000_000_000)
          self.assertEqual(act.torqueOutputCan != 0, eps==3)
          status=(sends[0].dat[4] >> 2)&7
          self.assertEqual(status, 4 if eps==3 else 3 if eps in (1,2) else 2)
          if eps==0:self.assertEqual(ci.CC.p87_lateral_status['phase'], 'eps_not_authorized')

  def test_frozen_or_missing_eps_cannot_authorize_a_command(self):
    for stamp in (0, 749_999_999, 1_000_000_001):
      ci=self.interface(True)
      _, sends=ci.CC.update(self.control(), structs.CarControlSP(),
        self.state(p87_eps_timestamp=stamp), 1_000_000_000)
      self.assertEqual(ci.CC.p87_lateral_status['phase'], 'eps_feedback_stale')
      self.assertEqual(sends[0].dat[3:7], bytes.fromhex('00080000'))

  def test_diagnostics_distinguish_upstream_inactivity_from_eps_refusal(self):
    ci=self.interface(True);cc=structs.CarControl()
    ci.CC.update(cc.as_reader(), structs.CarControlSP(), self.state(),1_000_000_000)
    self.assertEqual(ci.CC.p87_lateral_status['phase'],'lateral_not_requested')
    ci.CC.frame=5
    ci.CC.update(self.control(), structs.CarControlSP(), self.state(eps_state_lka=0, eps_active=False),1_000_000_000)
    self.assertEqual(ci.CC.p87_lateral_status['phase'],'eps_not_authorized')

  def test_longitudinal_flags_never_send_acc_brake_or_resume(self):
    ci=self.interface(True);cc=structs.CarControl(latActive=True,longActive=True,enabled=True)
    cc.actuators.torque=.2;cc.actuators.accel=1.;cc.cruiseControl.resume=True
    _,sends=ci.CC.update(cc.as_reader(),structs.CarControlSP(),self.state(speed_kph=30.),1_000_000_000)
    self.assertEqual([(f.address,f.src) for f in sends],[(0x3F2,0)])
    self.assertFalse(ci.CC.p87_lateral_status['openpilot_longitudinal'])
    self.assertFalse(ci.CC.p87_lateral_status['auto_resume'])

  def test_template_preserves_unknown_bits_and_never_requests_angle(self):
    template = bytes.fromhex('a1450d33ff0133ff')
    msg = create_p87_lka(template, -8, 100, 4)
    self.assertEqual(msg.dat[:3], template[:3])
    self.assertEqual(msg.dat[4] & 3, template[4] & 3)
    self.assertEqual(msg.dat[5] & 1, template[5] & 1)
    self.assertEqual(msg.dat[6], 0)
    self.assertEqual(msg.dat[7], template[7] & 3)

class TestP87LowSpeedNativeIntegration(unittest.TestCase):
  def test_prepare_ack_torque_and_eps_withdrawal_at_thirty(self):
    from opendbc.safety.tests import test_psa_5008 as native
    helper=TestP87Interface()
    ci=helper.interface(True)
    safety=native.TestP87Safety();safety.setUp()
    cc=helper.control()
    for frame,eps in ((0,2),(5,3),(10,0)):
      ci.CC.frame=frame;safety.feed(speed=30.,eps=eps)
      act,sends=ci.CC.update(cc,structs.CarControlSP(),
        helper.state(speed_kph=30.,eps_state_lka=eps,eps_active=eps==3),1_000_000_000)
      self.assertEqual(act.torqueOutputCan != 0, eps==3)
      self.assertEqual(len(sends),1)
      self.assertTrue(safety.safety.safety_tx_hook(native.make_msg(sends[0].src,sends[0].address,dat=sends[0].dat)))


if __name__ == '__main__':
  unittest.main()
