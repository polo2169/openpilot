import os
import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from cereal import messaging, log
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.car.psa import rvv_wire
from opendbc.car.psa.rvv_following import FollowingDecision
from openpilot.selfdrive.controls.controlsd import Controls, publish_t9_rvv_request, t9_split_common_fault


class TestRvvCerealTransport(unittest.TestCase):
  def test_real_cereal_data_field_roundtrip_without_any_publication(self):
    pm = Mock()
    now = time.monotonic_ns()
    decision = FollowingDecision('vision_following_candidate', target_kph=49, computed_ns=now,
                                 lead_rx_ns=now, model_ns=now, candidate_increase_permitted=True)
    published = publish_t9_rvv_request(pm, decision, now, True)
    service, event = pm.send.call_args.args
    self.assertEqual(service, rvv_wire.SERVICE)
    received = messaging.log_from_bytes(event.to_bytes())
    self.assertTrue(received.valid)
    self.assertEqual(received.which(), rvv_wire.SERVICE)
    self.assertEqual(bytes(received.customReservedRawData0), rvv_wire.command(decision, now=now, engaged=True))
    self.assertEqual(published, bytes(received.customReservedRawData0))

  def test_failed_publication_cannot_return_an_episode_commit(self):
    pm = Mock()
    pm.send.side_effect = RuntimeError('publisher unavailable')
    now = time.monotonic_ns()
    decision = FollowingDecision('vision_following_candidate', target_kph=49, computed_ns=now,
                                 lead_rx_ns=now, model_ns=now, candidate_increase_permitted=True)
    with self.assertRaisesRegex(RuntimeError, 'publisher unavailable'):
      publish_t9_rvv_request(pm, decision, now, True)

  def test_empty_subscriber_data_is_rejected_before_any_publisher_exists(self):
    sm = messaging.SubMaster([rvv_wire.SERVICE])
    # A zero-frequency service is considered valid by SubMaster even before
    # publication. Our payload/freshness checks must reject its empty value.
    self.assertTrue(rvv_wire.blocked(bytes(sm[rvv_wire.SERVICE]), time.monotonic_ns()))

  def test_split_shared_subscriber_failure_during_rvv_local_stop_is_common_fault(self):
    cs = SimpleNamespace(canValid=True,canTimeout=False,brakePressed=False,gasPressed=False,blockPcmEnable=False)
    services = {'selfdriveState','carState','driverMonitoringState','modelV2','liveCalibration'}
    for failed in services:
      sm = Mock()
      sm.all_checks.side_effect = lambda requested: failed not in requested
      self.assertTrue(t9_split_common_fault(sm,cs),failed)
    sm = Mock(); sm.all_checks.return_value = True
    self.assertFalse(t9_split_common_fault(sm,cs))
    for field in ('canTimeout','brakePressed','gasPressed','blockPcmEnable'):
      setattr(cs,field,True)
      self.assertTrue(t9_split_common_fault(sm,cs),field)
      setattr(cs,field,False)

  def test_split_publication_retains_version_and_common_fault(self):
    pm = Mock(); now = time.monotonic_ns()
    decision = FollowingDecision('lead_lost',rearm_required=True,common_fault=True)
    payload = publish_t9_rvv_request(pm,decision,now,False,split_axes=True)
    self.assertEqual(rvv_wire.split_status(payload,now),(True,True,False))
    self.assertEqual(payload[:4],b'RVV2')

  def test_real_controls_state_suppresses_lateral_on_common_fault_after_local_rvv_stop(self):
    with patch.dict(os.environ,{'PSA_T9_RVV_TEST':'1','PSA_T9_LATERAL_TEST':'1','PSA_T9_SPLIT_AXES_TEST':'1'}):
      cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    cs = structs.CarState(vEgo=75/3.6,vEgoRaw=75/3.6,canValid=True)
    cs.cruiseState.enabled = True
    class SM(dict):
      failed = None
      valid = {'lateralManeuverPlan':False}
      def all_checks(self,services):
        return self.failed not in services
    sm = SM(carState=cs,
      selfdriveState=SimpleNamespace(enabled=True,active=True,state=log.SelfdriveState.OpenpilotState.enabled),
      liveParameters=SimpleNamespace(stiffnessFactor=1.,steerRatio=15.,angleOffsetDeg=0.,roll=0.),
      liveTorqueParameters=SimpleNamespace(useParams=False),liveDelay=SimpleNamespace(lateralDelay=.15),
      longitudinalPlan=SimpleNamespace(aTarget=0.,shouldStop=False),onroadEvents=[],
      modelV2=SimpleNamespace(meta=SimpleNamespace(laneChangeState=log.LaneChangeState.off),
                             action=SimpleNamespace(desiredCurvature=0.)))
    controls = Controls.__new__(Controls)
    from opendbc.car.psa.lateral_pause import T9LateralPause
    controls.t9_lateral_pause = T9LateralPause()
    sm.logMonoTime = {'modelV2': time.monotonic_ns()}
    controls.CP=cp; controls.CI=CarInterface(cp); controls.sm=sm; controls.pm=Mock()
    controls.t9_rvv_active=controls.t9_split_axes=True
    controls.t9_rvv_following=Mock()
    controls.t9_rvv_following.update.return_value=None
    controls.t9_rvv_following.decision=FollowingDecision('lead_lost',rearm_required=True)
    controls.VM=Mock(); controls.VM.calc_curvature.return_value=0.
    controls.LoC=Mock(); controls.LoC.long_control_state=0; controls.LoC.update.return_value=0.
    controls.LaC=Mock(); controls.LaC.update.return_value=(0.,0.,None)
    controls.desired_curvature=controls.curvature=0.; controls.steer_limited_by_safety=False
    cc,_=controls.state_control()
    self.assertTrue(cc.latActive)
    self.assertFalse(cc.longActive)
    for failed in ('driverMonitoringState','selfdriveState'):
      sm.failed=failed
      cc,_=controls.state_control()
      self.assertFalse(cc.latActive)
      self.assertFalse(cc.longActive)
      _,event=controls.pm.send.call_args.args
      self.assertTrue(rvv_wire.split_status(bytes(event.customReservedRawData0),time.monotonic_ns())[0])
      self.assertTrue(controls.LaC.reset.called)
    sm.failed=None
    for decision in (FollowingDecision('lead_lost',rearm_required=True,common_fault=True),
                     FollowingDecision('calibration_invalid_or_stale',rearm_required=True),
                     FollowingDecision('unrecognized_failure',rearm_required=True)):
      controls.t9_rvv_following.decision=decision
      cc,_=controls.state_control()
      self.assertFalse(cc.latActive)
      self.assertFalse(cc.longActive)
      _,event=controls.pm.send.call_args.args
      self.assertTrue(rvv_wire.split_status(bytes(event.customReservedRawData0),time.monotonic_ns())[0])
    controls.t9_rvv_following.decision=FollowingDecision('lead_lost',rearm_required=True)
    for state in (log.SelfdriveState.OpenpilotState.softDisabling,
                  log.SelfdriveState.OpenpilotState.overriding,
                  log.SelfdriveState.OpenpilotState.preEnabled):
      sm['selfdriveState'].state=state
      cc,_=controls.state_control()
      self.assertFalse(cc.latActive)
      self.assertFalse(cc.longActive)
      _,event=controls.pm.send.call_args.args
      self.assertTrue(rvv_wire.split_status(bytes(event.customReservedRawData0),time.monotonic_ns())[0])
    sm['selfdriveState'].state=log.SelfdriveState.OpenpilotState.enabled
    cs.steeringDisengage=True
    cc,_=controls.state_control()
    self.assertFalse(cc.latActive)
    cs.steeringDisengage = False
    cs.steeringTorque = 16
    sm['modelV2'] = messaging.new_message('modelV2').modelV2
    sm['modelV2'].laneLineProbs = [.1, .95, .95, .1]
    for line, y in zip(sm['modelV2'].init('laneLines', 4), (-5.4, -1.8, 1.8, 5.4), strict=True):
      line.y = [y]
    start = time.monotonic_ns()
    with patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=start):
      cc, _ = controls.state_control()
    self.assertTrue(cc.psaLateralPause)
    self.assertFalse(cc.latActive)
    self.assertFalse(controls.LaC.update.call_args.args[0])
    cs.steeringTorque = 15
    for ms in range(50, 551, 50):
      now = start + ms * 1_000_000
      sm.logMonoTime['modelV2'] = now
      with patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=now):
        cc, _ = controls.state_control()
      self.assertEqual(cc.psaLateralPause, ms < 550)
      self.assertEqual(cc.latActive, ms == 550)

    from opendbc.car.psa.eps_cycle import T9EpsCycleGate
    controls.CP.safetyConfigs[0].safetyParam = rvv_wire.EPS_CYCLE_SAFETY_PARAM
    controls.t9_eps_cycle = T9EpsCycleGate()
    sm['modelV2'].orientationRate.t = [0., .5, 1., 1.5, 2.]
    sm['modelV2'].orientationRate.z = [0.] * 5
    sm['modelV2'].velocity.x = [25.] * 5
    for ms in range(600, 1151, 50):
      now = start + ms * 1_000_000
      sm.logMonoTime['modelV2'] = now
      with patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=now):
        cc, _ = controls.state_control()
    self.assertTrue(cc.psaEpsCycleReady)
    cs.psaEpsCycling = True
    now = start + 1_200_000_000
    sm.logMonoTime['modelV2'] = now
    controls.LaC.reset.reset_mock()
    with patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=now):
      cc, _ = controls.state_control()
    self.assertFalse(cc.latActive)
    self.assertTrue(cc.psaEpsCycleReady)
    self.assertTrue(controls.LaC.reset.called)
    copied = structs.CarControl.from_bytes(cc.to_bytes())
    with copied as deserialized:
      self.assertTrue(deserialized.psaEpsCycleReady)
    sm.failed = 'driverMonitoringState'
    with patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=now):
      cc, _ = controls.state_control()
    self.assertFalse(cc.psaEpsCycleReady)
    self.assertFalse(cc.latActive)


if __name__ == '__main__':
  unittest.main()
