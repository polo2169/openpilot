"""P87 host tuning: real PID/vehicle model, with optional NN extensions inert."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR, p87_torque_limit
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib import p87_torque
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque, KI, KP_INTERP


class InertExtension:
  def __init__(self, *args):
    self._nnlc_enabled = False

  def update_limits(self):
    pass

  def update_override_torque_params(self, params):
    return False

  def update(self, CS, VM, pid, params, ff, pid_log, *args):
    return pid_log, args[-1]


class TestP87Torque(unittest.TestCase):
  def controller(self, fingerprint=CAR.PSA_PEUGEOT_5008_P87):
    cp=CarInterface.get_non_essential_params(fingerprint).as_reader()
    sp=CarInterface.get_non_essential_params_sp(cp,fingerprint)
    ci=CarInterface(cp,sp)
    with patch('openpilot.selfdrive.controls.lib.latcontrol_torque.LatControlTorqueExt',InertExtension):
      lac=LatControlTorque(cp,sp,ci,.01)
    return lac,VehicleModel(cp)

  def step(self, lac, vm, *, speed=110., desired=.05, active=True):
    cs=structs.CarState(vEgo=speed/3.6,vEgoRaw=speed/3.6)
    params=SimpleNamespace(angleOffsetDeg=0.,roll=0.)
    return lac.update(active,cs,vm,params,False,desired/cs.vEgo**2,None,False,.35)

  def test_p87_halves_feedback_gains_without_changing_other_psa(self):
    p87,_=self.controller();other,_=self.controller(CAR.PSA_PEUGEOT_3008)
    self.assertEqual(p87.pid._k_p[1],[p*.5 for p in KP_INTERP])
    self.assertEqual(p87.pid._k_i[1],[KI*.5])
    self.assertEqual(other.pid._k_p[1],KP_INTERP)
    self.assertEqual(other.pid._k_i[1],[KI])
    self.assertAlmostEqual(other.torque_params.friction,.210548,places=6)

  def test_live_learning_cannot_restore_large_friction(self):
    lac,_=self.controller()
    lac.update_torque_parameters(1.8,.02,.19)
    self.assertAlmostEqual(lac.torque_params.latAccelFactor,1.8,places=6)
    self.assertAlmostEqual(lac.torque_params.latAccelOffset,.02,places=6)
    self.assertAlmostEqual(lac.torque_params.friction,.07,places=6)

  def test_manual_override_is_also_bounded(self):
    lac,vm=self.controller()
    def override(params):
      params.friction=.3
      return True
    lac.extension.update_override_torque_params=override
    self.step(lac,vm)
    self.assertAlmostEqual(lac.torque_params.friction,.07,places=6)

  def test_friction_clamp_does_not_raise_a_smaller_value(self):
    for value in (0.,.02,.05,.07):
      self.assertEqual(p87_torque.friction_limit(value),value)
    for value in (math.nan,math.inf,-math.inf,-.2):
      self.assertEqual(p87_torque.friction_limit(value),0.)

  def test_real_pid_output_and_limits_follow_native_ceiling_in_both_directions(self):
    for speed in (30.,50.,70.,90.,110.,130.,140.):
      for direction in (-1,1):
        lac,vm=self.controller()
        for _ in range(100):
          torque,_,state=self.step(lac,vm,speed=speed,desired=direction*3.)
          effective=structs.CarState(vEgoRaw=speed/3.6).vEgoRaw*3.6
          self.assertLessEqual(abs(torque),p87_torque_limit(effective)/150.+1e-9)
        self.assertEqual(state.version,p87_torque.LOG_VERSION)
        self.assertAlmostEqual(lac.pid.pos_limit,lac.steer_max*lac.torque_params.latAccelFactor,places=6)

  def test_acceleration_cannot_leave_stale_pid_limits(self):
    lac,vm=self.controller()
    for speed in (50.,90.,110.,130.,50.):
      self.step(lac,vm,speed=speed,desired=3.)
      effective=structs.CarState(vEgoRaw=speed/3.6).vEgoRaw*3.6
      self.assertEqual(lac.steer_max,p87_torque.normalized_limit(effective))

  def test_nonfinite_wheel_speed_zeroes_the_available_range(self):
    self.assertEqual(p87_torque.normalized_limit(math.nan),0.)

  def test_small_error_has_less_feedback_and_friction_without_losing_feedforward(self):
    p87,vm=self.controller();other,other_vm=self.controller(CAR.PSA_PEUGEOT_3008)
    for _ in range(100):
      torque,_,q=self.step(p87,vm)
      old,_,baseline=self.step(other,other_vm)
    self.assertGreater(abs(torque),0.)
    self.assertLess(abs(torque),abs(old)*.7)
    self.assertAlmostEqual(q.desiredLateralAccel,baseline.desiredLateralAccel)

  def test_inactive_stays_zero(self):
    lac,vm=self.controller()
    torque,_,state=self.step(lac,vm,desired=2.,active=False)
    self.assertEqual(torque,0.)
    self.assertFalse(state.active)

  def test_real_extension_with_recorded_optional_modes_disabled(self):
    cp=CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_5008_P87).as_reader()
    sp=CarInterface.get_non_essential_params_sp(cp,CAR.PSA_PEUGEOT_5008_P87)
    ci=CarInterface(cp,sp)
    class NoOptionalModes:
      def get_bool(self,key):return False
    with patch('openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc.Params',NoOptionalModes), \
         patch('openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_jerk_aware.Params',NoOptionalModes), \
         patch('openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override.Params',NoOptionalModes):
      lac=LatControlTorque(cp,sp,ci,.01)
    vm=VehicleModel(cp)
    for _ in range(80):
      torque,_,state=self.step(lac,vm)
      self.assertLessEqual(abs(torque),lac.steer_max)
    self.assertFalse(lac.extension._nnlc_enabled)
    self.assertEqual(state.version,p87_torque.LOG_VERSION)

  def test_tuned_pid_commands_pass_actual_controller_and_native_safety(self):
    from opendbc.car.psa.tests import test_5008 as host_fixture
    from opendbc.safety.tests import test_psa_5008 as native_fixture
    for speed in (90.,110.):
      lac,vm=self.controller()
      helper=host_fixture.TestP87Interface();ci=helper.interface(True)
      native=native_fixture.TestP87Safety();native.setUp()
      nonzero=False
      for step in range(70):
        now=1_000_000_000+step*50_000_000
        for _ in range(5):
          torque,_,_=self.step(lac,vm,speed=speed,desired=.4 if step//20%2 else -.4)
        cc=structs.CarControl(enabled=True,latActive=True)
        cc.actuators.torque=float(torque)
        eps=3 if ci.CC.p87_admission.request_ns else 2
        native.safety.set_timer(100_000+step*50_000)
        native.feed(speed=speed,eps=eps)
        state=helper.state(speed_kph=speed,eps_state_lka=eps,eps_active=eps==3,
          p87_lka_timestamp=now,p87_eps_timestamp=now)
        ci.CC.frame=step*5;ci.CC.model_sm.logMonoTime['modelV2']=now
        output,sends=ci.CC.update(cc.as_reader(),structs.CarControlSP(),state,now)
        self.assertEqual(len(sends),1)
        self.assertTrue(native.safety.safety_tx_hook(native_fixture.make_msg(0,0x3F2,dat=sends[0].dat)))
        nonzero|=output.torqueOutputCan!=0
      self.assertTrue(nonzero)


if __name__=='__main__':
  unittest.main()
