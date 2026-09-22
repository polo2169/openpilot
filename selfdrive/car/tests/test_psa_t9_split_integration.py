"""Real CAN decoding, lateral lifecycle, compiled Panda and native events."""
import copy
import os
import time
import unittest
from unittest.mock import MagicMock, patch

from cereal import log
from opendbc.car import structs
from opendbc.car.psa import rvv_wire
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.rvv_following import FollowingDecision
from opendbc.car.psa.tests import test_lateral_test as lateral_tests
from opendbc.car.psa.values import CAR
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.selfdrived.state import StateMachine


class TestSplitInterfaceIntegration(unittest.TestCase):
  def setUp(self):
    self.h=lateral_tests.TestT9LateralIntegration(); self.h.setUp(); self.addCleanup(self.h.doCleanups)
    with patch.dict(os.environ,{'PSA_T9_RVV_TEST':'1','PSA_T9_LATERAL_TEST':'1','PSA_T9_SPLIT_AXES_TEST':'1'}):
      cp=CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.h.cp=cp; self.h.ci=CarInterface(cp)
    self.h.safety.set_safety_hooks(31,0x1314); self.h.safety.init_tests()
    self.seq=0
    self.sm=MagicMock(); self.sm.valid={rvv_wire.SERVICE:True}
    with patch('cereal.messaging.SubMaster',return_value=self.sm):
      self.events=CarSpecificEvents(cp)
    self.machine=StateMachine(); self.machine.state=log.SelfdriveState.OpenpilotState.enabled
    for ms in range(0,4001,10):
      self.step(ms)
    self.assertTrue(self.h.safety.get_t9_split_lateral_allowed())
    self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())

  def step(self,ms,**kwargs):
    result=self.h.step(ms,**kwargs)
    if ms>=2200 and kwargs.get('requested',True):
      self.seq+=1
      self.assertEqual(self.h.safety.test_t9_rvv_request(0x2100|74,self.seq),0)
    return result

  def native(self,previous):
    now=time.monotonic_ns()
    decision=FollowingDecision('vision_following_candidate',target_kph=74,computed_ns=now,
      lead_rx_ns=now,model_ns=now,candidate_increase_permitted=True)
    self.sm.logMonoTime={rvv_wire.SERVICE:now}
    self.sm.__getitem__.return_value=rvv_wire.command(decision,now=now,engaged=True,split_axes=True)
    return self.events.update(self.h.ci.CS.out,previous,structs.CarControl(enabled=True))

  def test_observed_eps_withdrawal_stops_lateral_keeps_rvv_and_latches(self):
    previous=copy.deepcopy(self.h.ci.CS.out)
    applied,_=self.step(4001,eps=0,lat_active=False)
    self.assertEqual(applied.torqueOutputCan,0)
    self.assertEqual(self.h.ci.CC.t9_lateral.lateral.reason,'eps_not_authorized')
    self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
    self.assertEqual(self.machine.update(self.native(previous)),(True,True))
    for ms in range(4010,4301,10):
      applied,_=self.step(ms,eps=1,lat_active=False)
      self.assertEqual(applied.torqueOutputCan,0)
      self.assertTrue(self.h.ci.CS.out.steerFaultTemporary)
      self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
      self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
      self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())

  def test_driver_or_blinker_pause_keeps_session_and_can_resume_progressively(self):
    for change in ({'driver': -16}, {'driver': 16}, {'blinker': 1}, {'blinker': 2}, {'blinker': 3}):
      with self.subTest(change=change):
        self.setUp()
        previous = copy.deepcopy(self.h.ci.CS.out)
        applied, _ = self.step(4001, pause=True, lat_active=False, **change)
        self.assertEqual(applied.torqueOutputCan, 0)
        self.assertTrue(self.h.ci.CS.out.psaLateralPaused)
        self.assertFalse(self.h.ci.CS.out.steeringDisengage)
        self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
        self.assertEqual(self.machine.update(self.native(previous)), (True, True))
        for ms in range(4010, 4601, 10):
          applied, _ = self.step(ms, pause=True, lat_active=False)
          self.assertEqual(applied.torqueOutputCan, 0)
          self.assertTrue(self.h.safety.get_t9_split_lateral_allowed())
        previous_torque = 0
        for ms in range(4610, 5501, 10):
          applied, _ = self.step(ms, resume=True)
          self.assertLessEqual(abs(applied.torqueOutputCan - previous_torque), 1)
          previous_torque = applied.torqueOutputCan
        self.assertEqual(applied.torqueOutputCan, 15)
        self.assertFalse(self.h.ci.CS.out.psaLateralPaused)

  def test_eps_withdrawal_during_pause_requires_manual_rearm(self):
    self.step(4001, pause=True, lat_active=False, driver=16)
    self.step(4010, pause=True, lat_active=False, eps=0)
    for ms in range(4020, 4501, 10):
      applied, _ = self.step(ms, eps=1, lat_active=False)
      self.assertEqual(applied.torqueOutputCan, 0)
      self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
      self.assertTrue(self.h.ci.CS.out.steerFaultTemporary)

  def test_initial_axis_refusal_cannot_wake_on_recovery_without_new_physical_edge(self):
    cases = (({'eps':0},'eps_not_authorized'), ({'driver':16},'driver_override'),
             ({'stock_lka':bytes.fromhex('0000120008000000')},'stock_lka_not_authorized'))
    for changes,reason in cases:
      with self.subTest(reason=reason):
        self.h.ci=CarInterface(self.h.cp)
        self.h.safety.set_timer(0)
        self.h.safety.set_safety_hooks(31,0x1314); self.h.safety.init_tests(); self.seq=0
        for ms in range(0,2200,10):
          self.step(ms,requested=False,eps=1)
        self.step(2200,lat_active=False,**({'eps':1}|changes))
        self.assertEqual(self.h.ci.CC.t9_lateral.lateral.reason,reason)
        self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
        self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
        self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())
        for ms in range(2210,2330,10):
          applied,_=self.step(ms,eps=1,driver=0,lat_active=False)
          self.assertEqual(applied.torqueOutputCan,0)
          self.assertEqual(self.h.ci.CC.t9_lateral.lateral.reason,reason)
          self.assertTrue(self.h.ci.CS.out.steeringDisengage or self.h.ci.CS.out.steerFaultTemporary)
          self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
          self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())
        self.step(2330,requested=False,eps=1,lat_active=False)
        self.step(2340,requested=True,eps=1)
        self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
        self.assertFalse(self.h.ci.CS.out.steeringDisengage)
        self.assertFalse(self.h.ci.CS.out.steerFaultTemporary)
        self.assertTrue(self.h.safety.get_t9_split_lateral_allowed())
        self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())

  def test_actual_wrong_bus_topology_fault_remains_common(self):
    previous=copy.deepcopy(self.h.ci.CS.out)
    self.h.step(4001,stock_bus=0,lat_active=False)
    self.h.step(4002,stock_bus=0,lat_active=False)
    self.assertEqual(self.h.ci.CC.t9_lateral.lateral.reason,'can_invalid')
    self.assertTrue(self.h.ci.CS.out.blockPcmEnable)
    self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
    self.assertFalse(self.h.safety.get_t9_split_rvv_allowed())
    self.assertEqual(self.machine.update(self.native(previous)),(False,False))

  def test_real_can_admission_at_forty_keeps_lateral_inactive(self):
    self.h.ci = CarInterface(self.h.cp)
    self.h.safety.set_timer(0)
    self.h.safety.set_safety_hooks(31, 0x1314)
    self.h.safety.init_tests()
    for ms in range(0, 2200, 10):
      self.h.step(ms, requested=False, lat_active=False, speed=40, eps=1)
    previous = copy.deepcopy(self.h.ci.CS.out)
    applied, sends = self.h.step(2200, requested=True, lat_active=False, speed=40, eps=1)
    self.assertEqual((applied.torqueOutputCan, sends), (0, []))
    self.assertEqual(self.h.ci.CC.t9_lateral.lateral.reason, 'speed_below_lateral_envelope')
    self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
    self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
    self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())
    self.assertEqual(self.h.safety.test_t9_rvv_request(0x2100 | 40, 1), 0)
    self.machine.state = log.SelfdriveState.OpenpilotState.disabled
    self.assertEqual(self.machine.update(self.native(previous)), (True, True))
    for ms in range(2210, 2350, 10):
      applied, sends = self.h.step(ms, speed=75, eps=1, lat_active=False)
      self.assertEqual((applied.torqueOutputCan, sends), (0, []))
      self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())

  def test_real_speed_drop_stops_only_lateral_above_rvv_floor(self):
    previous = copy.deepcopy(self.h.ci.CS.out)
    applied, _ = self.step(4001, speed=66, lat_active=False)
    self.assertEqual(applied.torqueOutputCan, 0)
    self.assertEqual(self.h.ci.CC.t9_lateral.lateral.reason, 'speed_below_lateral_envelope')
    self.h.step(4002, speed=66, lat_active=False)
    self.assertFalse(self.h.ci.CS.out.blockPcmEnable)
    self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())
    self.assertEqual(self.machine.update(self.native(previous)), (True, True))


if __name__ == '__main__':
  unittest.main()
