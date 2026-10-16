"""P87 must use ordinary ACC-coupled engagement, preserving other cars' MADS."""
import unittest
from unittest.mock import MagicMock, patch
from opendbc.car import structs
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.sunnypilot.mads.helpers import supports_independent_lateral, set_alternative_experience
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem


class TestP87Mads(unittest.TestCase):
  def cp(self, fingerprint='PSA_PEUGEOT_5008_P87', brand='psa'):
    return structs.CarParams(carFingerprint=fingerprint, brand=brand)

  def test_only_p87_is_excluded_and_params_are_unchanged(self):
    for fp, brand, expected in (('PSA_PEUGEOT_5008_P87','psa',False),
        ('PSA_PEUGEOT_308_T9','psa',True),('OTHER','hyundai',True)):
      cp=self.cp(fp,brand);params=MagicMock()
      params.get_bool.return_value=True;params.get.return_value=2
      self.assertEqual(supports_independent_lateral(cp),expected)
      set_alternative_experience(cp, structs.CarParamsSP(), params)
      self.assertEqual(bool(cp.alternativeExperience & ALTERNATIVE_EXPERIENCE.ENABLE_MADS),expected)
      params.put.assert_not_called();params.put_bool.assert_not_called();params.remove.assert_not_called()

  def make(self, cp):
    sd=MagicMock();sd.CP=cp;sd.CP_SP=structs.CarParamsSP()
    sd.params.get_bool.return_value=True;sd.params.get.return_value=2
    with patch('openpilot.sunnypilot.mads.mads.Params'):
      mads=ModularAssistiveDrivingSystem(sd)
    return mads,sd

  def test_no_independent_state_machine_or_events_can_start_on_p87(self):
    mads,sd=self.make(self.cp())
    self.assertFalse(mads.enabled_toggle)
    with patch.object(mads,'data_sample') as sample, patch.object(mads,'update_events') as events:
      for _ in range(400):mads.update(structs.CarState())
    sample.assert_not_called();events.assert_not_called()
    self.assertFalse(mads.enabled);self.assertFalse(mads.active)
    self.assertEqual(mads.lateral_mismatch_counter,0)
    sd.params.put_bool.assert_not_called()

  def test_other_platform_keeps_mads_and_mismatch_detection(self):
    mads,sd=self.make(self.cp('OTHER','hyundai'))
    self.assertTrue(mads.enabled_toggle)
    mads.active=True;sd.enabled=False
    ps=MagicMock();ps.safetyModel=structs.CarParams.SafetyModel.hyundai;ps.controlsAllowedLateral=False
    sd.sm={'pandaStates':[ps]}
    for _ in range(200):mads.data_sample()
    self.assertEqual(mads.lateral_mismatch_counter,200)

  def test_actual_controls_and_native_safety_wait_for_eps_at_thirty(self):
    from types import SimpleNamespace as NS
    from opendbc.car.psa.tests import test_5008 as fixtures
    from opendbc.safety.tests import test_psa_5008 as native
    from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
    helper=fixtures.TestP87Interface();ci=helper.interface(True)
    mads,sd=self.make(ci.CP)
    controls=ControlsExt.__new__(ControlsExt);controls.blinker_pause_lateral=MagicMock()
    controls.blinker_pause_lateral.update.return_value=False
    sm={'carState':structs.CarState(),'selfdriveState':NS(active=True),
        'selfdriveStateSP':NS(mads=NS(available=mads.enabled_toggle,active=mads.active))}
    self.assertFalse(sm['selfdriveStateSP'].mads.available)
    safety=native.TestP87Safety();safety.setUp()
    for frame,eps in ((0,2),(5,3),(10,0)):
      ci.CC.frame=frame;safety.feed(speed=30.,eps=eps)
      cc=structs.CarControl(enabled=True,latActive=controls.get_lat_active(sm))
      cc.actuators.torque=.2
      output,sends=ci.CC.update(cc.as_reader(),structs.CarControlSP(),
        helper.state(speed_kph=30.,eps_state_lka=eps,eps_active=eps==3),1_000_000_000)
      self.assertTrue(cc.latActive)
      self.assertEqual(output.torqueOutputCan != 0,eps==3)
      self.assertTrue(safety.safety.safety_tx_hook(native.make_msg(0,0x3F2,dat=sends[0].dat)))

  def test_manual_mode_never_requests_lateral_and_retains_blinker_pause(self):
    from types import SimpleNamespace as NS
    from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
    mads,_=self.make(self.cp());controls=ControlsExt.__new__(ControlsExt)
    controls.blinker_pause_lateral=MagicMock();controls.blinker_pause_lateral.update.return_value=False
    sm={'carState':structs.CarState(vEgo=40/3.6), 'selfdriveState':NS(active=False),
        'selfdriveStateSP':NS(mads=NS(available=mads.enabled_toggle,active=mads.active))}
    for _ in range(400):
      mads.update(sm['carState'])
      self.assertFalse(controls.get_lat_active(sm))
    self.assertEqual(mads.lateral_mismatch_counter,0)
    sm['selfdriveState'].active=True
    self.assertTrue(controls.get_lat_active(sm))
    controls.blinker_pause_lateral.update.return_value=True
    self.assertFalse(controls.get_lat_active(sm))


if __name__=='__main__':unittest.main()
