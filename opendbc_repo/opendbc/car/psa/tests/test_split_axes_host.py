"""Host admission and latch checks for the explicitly selected bench profile."""
import os
import unittest
from unittest.mock import patch

from opendbc.car.psa import rvv_wire
from opendbc.car.psa.interface import CarInterface, LOCAL_LATERAL_STOP_REASONS
from opendbc.car.psa.lateral_test import enabled as lateral_enabled
from opendbc.car.psa.rvv_following import T9RvvFollowingObserver
from opendbc.car.psa.tests.test_rvv_control import BASE, observer_messages
from opendbc.car.psa.values import CAR


class TestSplitHost(unittest.TestCase):
  def messages(self, now, **changes):
    sm = observer_messages(now, **changes)
    sm['modelV2'] = object()
    sm.valid['modelV2'] = sm.alive['modelV2'] = True
    sm.logMonoTime['modelV2'] = now
    return sm

  def publish(self, observer, now, engaged=True):
    payload = rvv_wire.command(observer.decision,now=now,engaged=engaged,split_axes=True)
    observer.command_published(payload)
    return rvv_wire.split_status(payload,now)

  def test_new_profile_requires_all_three_explicit_flags(self):
    for lateral,rvv,split,expected in (('0','0','1',0),('0','1','1',0x1310),
        ('1','0','1',0x1308),('1','1','0',0x1312),('1','1','1',0x1314)):
      with patch.dict(os.environ,{'PSA_T9_LATERAL_TEST':lateral,'PSA_T9_RVV_TEST':rvv,'PSA_T9_SPLIT_AXES_TEST':split}):
        cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
      self.assertEqual(cp.safetyConfigs[0].safetyParam,expected)
      self.assertEqual(rvv_wire.split(cp),expected==0x1314)
      self.assertFalse(cp.openpilotLongitudinalControl)
      if expected==0x1314:
        self.assertTrue(lateral_enabled(cp))
        self.assertTrue(rvv_wire.enabled(cp))
        self.assertAlmostEqual(cp.minEnableSpeed * 3.6, 40., places=4)
        self.assertAlmostEqual(cp.minSteerSpeed * 3.6, 67.1, places=4)

  def test_no_target_local_stop_commits_only_when_published_engaged(self):
    for engaged in (False,True):
      observer = T9RvvFollowingObserver(split_axes=True)
      observer.update(self.messages(BASE,dRel=10.),BASE)
      self.publish(observer,BASE,engaged)
      self.assertEqual(observer.request_episode_started,engaged)
      now=BASE+10_000_000
      observer.update(self.messages(now),now)
      if engaged:
        self.assertEqual(observer.decision.reason,'fixed_cruise_cannot_handle_critical_lead')
        self.assertEqual(self.publish(observer,now),(False,True,False))
      else:
        self.assertEqual(self.publish(observer,now),(False,False,True))

  def test_local_stop_does_not_hide_new_common_model_car_or_calibration_failure(self):
    for service in ('modelV2','carState','liveCalibration'):
      observer=T9RvvFollowingObserver(split_axes=True)
      observer.update(self.messages(BASE),BASE)
      self.assertEqual(self.publish(observer,BASE),(False,False,True))
      now=BASE+10_000_000
      observer.update(self.messages(now,status=False),now)
      self.assertEqual(self.publish(observer,now),(False,True,False))
      now+=10_000_000
      sm=self.messages(now); sm.alive[service]=False
      observer.update(sm,now)
      self.assertTrue(self.publish(observer,now)[0],service)

  def test_recovered_lead_does_not_clear_published_stop(self):
    observer=T9RvvFollowingObserver(split_axes=True)
    observer.update(self.messages(BASE),BASE); self.publish(observer,BASE)
    now=BASE+10_000_000
    observer.update(self.messages(now,status=False),now); self.publish(observer,now)
    now+=10_000_000
    observer.update(self.messages(now),now)
    self.assertEqual(self.publish(observer,now),(False,True,False))
    now+=10_000_000
    observer.update(self.messages(now,active=False),now); self.publish(observer,now,False)
    now+=10_000_000
    observer.update(self.messages(now),now)
    self.assertEqual(self.publish(observer,now),(False,False,True))

  def test_eps_withdrawal_reason_is_local_but_shared_faults_are_not(self):
    self.assertIn('eps_not_authorized',LOCAL_LATERAL_STOP_REASONS)
    self.assertIn('speed_below_lateral_envelope',LOCAL_LATERAL_STOP_REASONS)
    for reason in ('can_invalid','safety_rx_stale','invalid_clock','control_update_timeout','nonfinite_input'):
      self.assertNotIn(reason,LOCAL_LATERAL_STOP_REASONS)


if __name__ == '__main__':
  unittest.main()
