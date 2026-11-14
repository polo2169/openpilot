import math
from types import SimpleNamespace
import unittest

from opendbc.car import structs
from opendbc.car.psa.p87_admission import P87Admission
from opendbc.car.psa.p87_driver_resume import P87DriverResumeGate
from opendbc.car.psa.p87_eps_cycle import native_eps_status_ok
from opendbc.car.psa.tests import test_5008 as host_fixture
from opendbc.car.psa.tests.test_p87_eps_cycle import prediction, held
from opendbc.safety.tests import test_psa_5008 as native_fixture
from opendbc.safety.tests.common import make_msg


class TestDriverResumeGate(unittest.TestCase):
  def setUp(self):
    self.gate = P87DriverResumeGate()

  def step(self, ms, **changes):
    now = 1_000_000_000 + ms*1_000_000
    args = dict(pending=True, driver_torque=0., steering_pressed=False, eps_status_ok=True,
                model_valid=True, model_ns=now)
    args.update(changes)
    return self.gate.update(now, **args)

  def test_released_driver_still_needs_three_hundred_ms(self):
    for ms in range(0, 300, 10): self.assertFalse(self.step(ms))
    self.assertTrue(self.step(300))

  def test_eps_limit_and_reboot_flags_are_independent_of_driver_activity(self):
    for activity in (0, 1, None):
      signals = held()
      if activity is None: signals.pop('STEERWHL_HOLD_BY_DRV')
      else: signals['STEERWHL_HOLD_BY_DRV'] = activity
      self.assertTrue(native_eps_status_ok(SimpleNamespace(is_dat_dira=signals)))
      for flag in ('TRQ_LIMIT_STATE', 'STEERING_REBOOT_REQUEST'):
        for value in (1, None):
          changed = dict(signals)
          if value is None: changed.pop(flag)
          else: changed[flag] = value
          self.assertFalse(native_eps_status_ok(SimpleNamespace(is_dat_dira=changed)))
    self.assertFalse(native_eps_status_ok(SimpleNamespace()))
    for ms in range(0, 1501, 50): self.assertFalse(self.step(ms, eps_status_ok=False))
    self.assertEqual(self.gate.reason, 'eps_status_not_ready')

  def test_release_hysteresis_is_lower_than_override_and_uses_steering_pressed(self):
    for torque in (31., 45., -48., math.nan, math.inf):
      self.gate = P87DriverResumeGate()
      for ms in range(0, 1501, 50): self.assertFalse(self.step(ms, driver_torque=torque))
      self.assertFalse(self.gate.released)
    self.assertFalse(self.step(1550, steering_pressed=True))
    self.gate = P87DriverResumeGate()
    for ms in range(0, 1001, 50): result = self.step(ms, driver_torque=-30.)
    self.assertTrue(result)

  def test_repeated_effort_restarts_release_stability(self):
    for ms in range(0, 1001, 50): self.step(ms)
    self.assertFalse(self.step(1050, driver_torque=-48.))
    for ms in range(1100, 1400, 50): self.assertFalse(self.step(ms))
    self.assertTrue(self.step(1400))

  def test_confirmed_release_tolerates_light_contact_but_actual_override_resets_it(self):
    for ms in range(0,301,50):self.step(ms)
    for ms,torque in ((350,33.),(400,-33.),(450,45.),(2500,-45.)):
      self.assertTrue(self.step(ms,driver_torque=torque))
      self.assertTrue(self.gate.released)
    self.assertFalse(self.step(2550,driver_torque=46.))
    self.assertFalse(self.step(2600,driver_torque=33.))
    for ms in range(2650,2950,50):self.assertFalse(self.step(ms))
    self.assertTrue(self.step(2950))

  def test_release_confirmation_cannot_survive_cancellation_or_steering_pressed(self):
    for change in ({'pending':False},{'steering_pressed':True}):
      self.gate=P87DriverResumeGate()
      for ms in range(0,301,50):self.step(ms)
      self.assertFalse(self.step(350,**change))
      self.assertFalse(self.step(400,driver_torque=33.))

  def test_stale_invalid_future_models_and_request_off_withdraw_readiness(self):
    for change in ({'model_valid': False}, {'model_ns': 0}, {'model_ns': 4_000_000_000}, {'pending': False}):
      self.gate = P87DriverResumeGate()
      for ms in range(0, 1001, 50): self.step(ms)
      self.assertFalse(self.step(1050, **change))


class TestDriverResumeAdmission(unittest.TestCase):
  def setUp(self):
    self.admission = P87Admission()

  def step(self, ms, **changes):
    now = 1_000_000_000 + ms*1_000_000
    args = dict(requested=True, reason=None, eps=2 if ms<450 else 3, eps_ns=now,
                model_valid=True, model_ns=now, tick=ms%50==0,
                driver_released=True, driver_resume_ready=True)
    args.update(changes)
    return self.admission.update(now, **args)

  def pause(self):
    for ms in range(0, 451, 50): self.step(ms)
    self.assertEqual(self.step(500, reason='driver_torque_override')[:3], ('driver_resume_pause',2,0))
    self.assertTrue(self.admission.resuming)
    self.assertEqual(self.admission.resume_attempts, 1)

  def request_ack(self):
    self.pause()
    for ms in range(550, 951, 50): self.assertFalse(self.step(ms, eps=2)[3])
    self.assertEqual(self.admission.phase, 'waiting_for_eps_ack')

  def test_pause_only_arms_from_fresh_authorized_active_assistance(self):
    result = self.step(0, reason='driver_torque_override')
    self.assertFalse(self.admission.resuming)
    self.assertEqual(self.admission.resume_attempts, 0)
    self.assertFalse(result[3])
    for eps in (0,1,2,4):
      self.admission = P87Admission()
      for ms in range(0,451,50):self.step(ms)
      self.step(500, eps=eps, reason='driver_torque_override')
      self.assertTrue(self.admission.blocked)
      self.assertEqual(self.admission.resume_attempts, 0)

  def test_observed_eps_three_two_zero_two_waits_at_zero_then_fresh_ack(self):
    self.pause()
    self.assertEqual(self.step(550, eps=2, driver_resume_ready=False)[1:3], (2,0))
    for ms in range(600,1501,50):
      result = self.step(ms, eps=0, reason='eps_not_authorized', driver_resume_ready=False)
      self.assertEqual(result[1:3], (2,0));self.assertFalse(self.admission.blocked)
    for ms in range(1550,1951,50):self.assertFalse(self.step(ms,eps=2)[3])
    self.assertEqual(self.admission.phase,'waiting_for_eps_ack')
    self.assertFalse(self.step(2000, eps_ns=self.admission.request_ns)[3])
    self.assertTrue(self.step(2050)[3])
    self.assertEqual(self.admission.resume_count,1)
    self.assertFalse(self.admission.resuming)

  def test_same_held_model_cannot_satisfy_three_new_lane_samples(self):
    self.pause()
    for ms in range(550,951,50):
      result=self.step(ms,eps=2,model_ns=1_550_000_000)
      self.assertFalse(result[3]);self.assertEqual(result[1:3],(2,0))

  def test_preexisting_eps_three_and_cached_release_never_authorize_resumption(self):
    self.pause()
    self.assertEqual(self.step(550,eps=2,eps_ns=self.admission.resume_since)[1:3],(2,0))
    for ms in range(600,5501,50):
      result=self.step(ms)
      self.assertFalse(result[3]);self.assertEqual(result[1:3],(2,0))
    self.assertEqual(result[0],'driver_resume_timeout')
    self.assertEqual(self.admission.resume_count,0)

  def test_held_override_and_near_threshold_effort_timeout_without_retry(self):
    for reason in ('driver_torque_override',None):
      self.admission=P87Admission();self.pause()
      for ms in range(550,2501,50):
        result=self.step(ms,reason=reason,driver_released=False,driver_resume_ready=False)
      self.assertEqual(result[0],'driver_resume_override_timeout')
      self.assertTrue(self.admission.blocked)
      self.assertFalse(self.step(2550,eps=2)[3])
      self.assertEqual(self.admission.resume_attempts,1)

  def test_missing_readiness_cannot_extend_total_five_seconds(self):
    self.pause()
    for ms in range(550,5501,50):
      result=self.step(ms,eps=2,driver_resume_ready=False)
    self.assertEqual(result[0],'driver_resume_timeout')
    self.assertEqual(self.admission.resume_count,0)

  def test_second_effort_before_request_restarts_validation_but_not_budget(self):
    self.pause()
    self.step(550,eps=2);self.step(600,eps=2)
    self.step(650,reason='driver_torque_override',driver_released=False)
    self.assertEqual(self.admission.resume_since,1_500_000_000)
    self.assertEqual(self.admission.resume_attempts,1)
    for ms in range(700,1001,50):self.assertFalse(self.step(ms,eps=2,driver_resume_ready=False)[3])
    self.assertEqual(self.admission.request_ns,0)

  def test_hard_faults_pedals_indicator_or_can_loss_latch_stop(self):
    for reason in ('driver_brake','driver_accelerator','stock_acc_fault','can_invalid','can_timeout',
                   'eps_feedback_stale','camera_template_stale','driver_indicator','below_lateral_speed',
                   'door_open','seatbelt_unlatched','stock_acc_inactive','driver_resume_panda_rejected'):
      self.admission=P87Admission();self.pause()
      self.step(550,reason=reason)
      self.assertTrue(self.admission.blocked,reason)
      self.assertFalse(self.step(600,eps=2)[3])

  def test_eps_fault_is_not_hidden_by_a_simultaneous_driver_override(self):
    self.pause();self.step(550,eps=4,reason='driver_torque_override',driver_released=False)
    self.assertTrue(self.admission.blocked)
    self.assertEqual(self.admission.phase,'eps_fault_or_unknown')

  def test_after_request_any_gate_loss_or_eps_zero_latches_without_retry(self):
    for changes in ({'driver_resume_ready':False},{'driver_released':False},
                    {'eps':0,'reason':'eps_not_authorized'},{'reason':'driver_torque_override'}):
      self.admission=P87Admission();self.request_ack()
      self.assertFalse(self.step(1000,**changes)[3])
      self.assertTrue(self.admission.blocked)
      self.assertFalse(self.step(1050)[3])

  def test_no_ack_timeout_is_not_reset_by_a_late_ack(self):
    self.request_ack()
    for ms in range(1000,1951,50):result=self.step(ms,eps=2)
    self.assertEqual(result[0],'eps_ack_timeout')
    self.assertFalse(self.step(2000)[3])

  def test_request_off_cancels_background_resumption(self):
    self.pause();self.step(550,requested=False)
    self.assertFalse(self.admission.resuming)
    self.assertFalse(self.step(600,eps=3)[3])
    self.assertEqual(self.admission.resume_count,0)

  def test_clock_gap_and_reversal_cannot_rearm_pause(self):
    for ms in (450,700):
      self.admission=P87Admission();self.pause()
      self.step(ms,eps=2)
      self.assertTrue(self.admission.blocked)

  def test_another_override_soon_after_success_needs_manual_reengagement(self):
    self.request_ack();self.assertTrue(self.step(1000)[3])
    self.assertEqual(self.step(1050,reason='driver_torque_override')[0],'driver_resume_repeated_override')
    self.assertTrue(self.admission.blocked)
    self.assertEqual(self.admission.resume_attempts,1)


class TestDriverResumeControllerNative(unittest.TestCase):
  def run_resume(self, *, speed=70., direction=1, condition=None):
    helper=host_fixture.TestP87Interface();ci=helper.interface(True);ci.CC.model_sm.model=prediction()
    native=native_fixture.TestP87Safety();native.setUp()
    cc=structs.CarControl(latActive=True,enabled=True);cc.actuators.torque=direction*.12
    physical_eps=2;stamp=1_000_000_000;actual_torque=[];pause_torques=[];activity_lost=False;new_request=False
    for ms in range(0,7501,10):
      now=1_000_000_000+ms*1_000_000;a=ci.CC.p87_admission
      raw_driver=direction*48. if 1510<=ms<1530 or (condition=='held' and ms>=1510) else 0.
      if a.resuming and a.phase=='waiting_for_eps_ack':new_request=True
      if (condition=='weak_after_release' and ms>=2200) or (condition=='weak_after_request' and new_request):
        raw_driver=direction*33.
      model_valid=not(condition=='model' and ms>=1500)
      if condition=='activity_lost_after_request' and a.resuming and a.phase=='waiting_for_eps_ack':
        activity_lost=True
      activity=not(condition=='activity_absent' or activity_lost)
      if condition=='activity_intermittent':activity=bool((ms//100)%2)
      signals=dict(held(),STEERWHL_HOLD_BY_DRV=int(activity))
      if condition=='activity_unknown':signals.pop('STEERWHL_HOLD_BY_DRV')
      if condition=='trq_limit' and ms>=1500:signals['TRQ_LIMIT_STATE']=1
      if condition=='reboot' and ms>=1500:signals['STEERING_REBOOT_REQUEST']=1
      ci.CC.model_sm.valid['modelV2']=model_valid
      fault=condition=='fault' and ms>=1700;brake=condition=='brake' and ms>=1700
      request=not(condition=='cancel' and ms>=1700)
      if ms%50==0:
        eps=3 if a.phase=='active' or a.phase=='waiting_for_eps_ack' else 2
        if a.resuming and a.phase!='waiting_for_eps_ack':
          age=(now-a.resume_since)/1e6
          eps=3 if age<150 else 2 if age<250 else 0 if age<1450 else 2
        if condition=='eps0' and ms>=1500:eps=0
        native.clock=now//1000;native.safety.set_timer(native.clock)
        native.feed(speed=speed,eps=eps,driver=raw_driver/3,fault=int(fault),brake=int(brake))
        physical_eps,stamp=eps,now
        native.safety.safety_rx_hook(native.packer.make_can_msg_safety('IS_DAT_DIRA',0,
          dict(signals,EPS_STATE_LKA=eps)))
        ci.CC.model_sm.logMonoTime['modelV2']=now
        if condition=='future_publications':ci.CC.model_sm.logMonoTime['modelV2']+=152_448
      else:
        native.safety.set_timer(now//1000)
        native.safety.safety_rx_hook(native.packer.make_can_msg_safety('STEERING',0,{'DRIVER_TORQUE':raw_driver/3}))
      state=helper.state(speed_kph=speed,vEgo=speed/3.6,vEgoRaw=speed/3.6,
        eps_state_lka=physical_eps,eps_active=physical_eps==3,p87_eps_timestamp=stamp,p87_lka_timestamp=now,
        steeringTorque=raw_driver,p87_driver_torque_raw=raw_driver,
        is_dat_dira=signals,accFaulted=fault,brakePressed=brake)
      if condition=='panda' and ms>=1700:state.p87_lka_rejection_count=1
      cc.latActive=request
      output,sends=ci.CC.update(cc.as_reader(),structs.CarControlSP(),state,now)
      if ms>=1500 and a.resuming:pause_torques.append(output.torqueOutputCan)
      if 1510<=ms<1530:self.assertEqual(output.torqueOutputCan,0)
      for send in sends:
        self.assertEqual((send.address,send.src),(0x3F2,0))
        self.assertTrue(native.safety.safety_tx_hook(make_msg(send.src,send.address,dat=send.dat)),(ms,ci.CC.p87_lateral_status))
      if ms>1500:actual_torque.append(output.torqueOutputCan)
    return ci,pause_torques,actual_torque

  def test_brief_override_recovers_via_actual_can_and_native_ack_both_speeds_and_directions(self):
    for speed in (70.,110.):
      for direction in (-1,1):
        ci,zero,after=self.run_resume(speed=speed,direction=direction)
        self.assertEqual(ci.CC.p87_admission.resume_count,1)
        self.assertEqual(ci.CC.p87_admission.resume_attempts,1)
        self.assertTrue(zero);self.assertTrue(all(t==0 for t in zero))
        self.assertTrue(any(t!=0 for t in after))
        self.assertFalse(ci.CC.p87_admission.blocked)

  def test_resume_without_asserted_or_stable_driver_activity_still_needs_real_ack(self):
    for speed in (70.,110.):
      for direction in (-1,1):
        for condition in ('activity_absent','activity_intermittent','activity_unknown','activity_lost_after_request'):
          ci,zero,after=self.run_resume(speed=speed,direction=direction,condition=condition)
          self.assertEqual(ci.CC.p87_admission.resume_count,1,(speed,direction,condition))
          self.assertTrue(zero);self.assertTrue(all(t==0 for t in zero))
          self.assertTrue(any(t!=0 for t in after))
          self.assertFalse(ci.CC.p87_admission.blocked)

  def test_held_effort_eps_flags_or_missing_models_never_returns_torque(self):
    for condition in ('held','trq_limit','reboot','model'):
      ci,_,after=self.run_resume(condition=condition)
      self.assertEqual(ci.CC.p87_admission.resume_count,0)
      self.assertTrue(ci.CC.p87_admission.blocked)
      self.assertTrue(all(t==0 for t in after))

  def test_light_contact_and_sub_tick_publications_do_not_latch_false_resume_failures(self):
    for direction in (-1,1):
      for condition in ('weak_after_release','weak_after_request','future_publications'):
        ci,zero,after=self.run_resume(direction=direction,condition=condition)
        self.assertEqual(ci.CC.p87_admission.resume_count,1,(direction,condition))
        self.assertFalse(ci.CC.p87_admission.blocked)
        self.assertTrue(zero);self.assertTrue(all(t==0 for t in zero))
        self.assertTrue(any(t!=0 for t in after))
        if condition=='future_publications':self.assertGreater(ci.CC.p87_model_snapshot.deferrals,0)

  def test_fault_brake_rejection_cancel_and_unsolicited_eps_loss_never_resume(self):
    for condition in ('fault','brake','panda','cancel','eps0'):
      ci,_,after=self.run_resume(condition=condition)
      self.assertEqual(ci.CC.p87_admission.resume_count,0)
      self.assertTrue(all(t==0 for t in after))
      if condition!='cancel':self.assertTrue(ci.CC.p87_admission.blocked)


if __name__=='__main__':
  unittest.main()
