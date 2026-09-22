"""Native state transition tests; no publisher or vehicle I/O."""
import os
import time
import unittest
from dataclasses import replace
from unittest.mock import MagicMock, patch

from cereal import log
from opendbc.car import structs
from opendbc.car.psa import rvv_wire
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.rvv_following import FollowingDecision
from opendbc.car.psa.values import CAR
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.selfdrived.state import StateMachine

EventName = log.OnroadEvent.EventName


class TestSplitAxisEvents(unittest.TestCase):
  def setUp(self):
    with patch.dict(os.environ, {'PSA_T9_RVV_TEST':'1', 'PSA_T9_LATERAL_TEST':'1', 'PSA_T9_SPLIT_AXES_TEST':'1'}):
      self.cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.sm = MagicMock()
    self.sm.valid = {rvv_wire.SERVICE: True}
    with patch('cereal.messaging.SubMaster',return_value=self.sm):
      self.generator = CarSpecificEvents(self.cp)
    self.now = time.monotonic_ns()
    self.sm.logMonoTime = {rvv_wire.SERVICE: self.now}
    self.decision = FollowingDecision('vision_following_candidate',target_kph=74,computed_ns=self.now,
      lead_rx_ns=self.now,model_ns=self.now,candidate_increase_permitted=True)
    self.cs = structs.CarState(vEgo=75/3.6,vEgoRaw=75/3.6,canValid=True,
      gearShifter=structs.CarState.GearShifter.drive)
    self.cs.cruiseState.enabled = self.cs.cruiseState.available = self.cs.cruiseState.nonAdaptive = True
    self.machine = StateMachine()
    self.machine.state = log.SelfdriveState.OpenpilotState.enabled

  def events(self, previous=None, payload=None):
    self.sm.__getitem__.return_value = (rvv_wire.command(self.decision,now=self.now,engaged=True,split_axes=True)
      if payload is None else payload)
    return self.generator.update(self.cs, previous or self.cs, structs.CarControl(enabled=True))

  def test_rvv_stop_preserves_healthy_lateral_and_stock_state(self):
    for reason in ('lead_lost','deceleration_exceeds_unvalidated_engine_brake_prior',
                   'fixed_cruise_cannot_handle_critical_lead'):
      self.decision = replace(self.decision,reason=reason,rearm_required=True)
      events = self.events()
      self.assertIn(EventName.psaRvvAxisUnavailable,events.names)
      self.assertFalse(events.contains(ET.IMMEDIATE_DISABLE))
      self.assertEqual(self.machine.update(events),(True,True))
      self.assertTrue(self.cs.cruiseState.enabled)
      warnings = events.create_alerts([ET.WARNING])
      self.assertTrue(any('Peugeot cruise remains active' in a.alert_text_2 for a in warnings))

  def test_steering_intervention_or_eps_loss_keeps_valid_rvv(self):
    for attribute in ('steeringDisengage','steeringPressed','steerFaultTemporary','steerFaultPermanent'):
      setattr(self.cs,attribute,True)
      events = self.events()
      self.assertIn(EventName.psaLateralAxisUnavailable,events.names)
      self.assertNotIn(EventName.steerDisengage,events.names)
      self.assertNotIn(EventName.steerUnavailable,events.names)
      self.assertEqual(self.machine.update(events),(True,True))
      setattr(self.cs,attribute,False)

  def test_both_axes_stop_and_target_return_does_not_reenable(self):
    self.cs.steerFaultTemporary = True
    self.decision = replace(self.decision,reason='lead_lost',rearm_required=True)
    events = self.events()
    self.assertIn(EventName.psaAxesUnavailable,events.names)
    self.assertEqual(self.machine.update(events),(False,False))
    self.cs.steerFaultTemporary = False
    self.decision = replace(self.decision,reason='vision_following_candidate',rearm_required=False)
    events = self.events()
    self.assertIn(EventName.psaAxesUnavailable,events.names)
    self.assertNotIn(EventName.pcmEnable,events.names)
    self.assertEqual(self.machine.update(events),(False,False))

  def test_lateral_stop_preserves_fresh_enabled_rvv_waiting_for_a_lead(self):
    self.cs.steeringDisengage = True
    self.decision = FollowingDecision('waiting_for_lead')
    self.assertEqual(self.machine.update(self.events()),(True,True))
    disabled = rvv_wire.command(self.decision,now=self.now,engaged=False,split_axes=True)
    self.assertEqual(self.machine.update(self.events(payload=disabled)),(False,False))

  def test_temporary_pause_preserves_session_without_an_rvv_target(self):
    self.cs.psaLateralPaused = self.cs.steeringPressed = True
    self.decision = FollowingDecision('waiting_for_lead')
    events = self.events()
    self.assertIn(EventName.psaLateralPaused, events.names)
    self.assertNotIn(EventName.psaLateralAxisUnavailable, events.names)
    self.assertEqual(self.machine.update(events), (True, True))
    self.cs.steerFaultTemporary = True
    events = self.events()
    self.assertIn(EventName.psaLateralAxisUnavailable, events.names)
    self.assertEqual(self.machine.update(events), (True, True))

  def test_waiting_for_lead_preserves_healthy_lateral(self):
    self.decision = FollowingDecision('waiting_for_lead')
    self.assertEqual(self.machine.update(self.events()),(True,True))

  def test_cycle_is_visible_without_disengagement_and_fault_takes_precedence(self):
    self.cs.psaEpsCycling = True
    events = self.events()
    self.assertIn(EventName.psaEpsCycling, events.names)
    self.assertEqual(self.machine.update(events), (True, True))
    self.assertTrue(any('EPS Test Cycle' in a.alert_text_1 for a in events.create_alerts([ET.WARNING])))
    self.cs.steerFaultTemporary = True
    events = self.events()
    self.assertNotIn(EventName.psaEpsCycling, events.names)
    self.assertIn(EventName.psaLateralAxisUnavailable, events.names)

  def test_stale_malformed_or_common_failed_wire_stops_both(self):
    for payload in (b'',b'RVV1'+bytes(28),rvv_wire.command(self.decision,now=self.now-1_000_000_000,
                   engaged=True,split_axes=True)):
      events = self.events(payload=payload)
      self.assertIn(EventName.psaAxesUnavailable,events.names)
      self.assertTrue(events.contains(ET.IMMEDIATE_DISABLE))
    self.decision = replace(self.decision,common_fault=True)
    self.assertEqual(self.machine.update(self.events()),(False,False))

  def test_physical_edge_can_admit_rvv_alone_from_disabled_native_state(self):
    self.machine.state = log.SelfdriveState.OpenpilotState.disabled
    self.cs.steerFaultTemporary = True
    self.decision = FollowingDecision('waiting_for_lead')
    disabled_wire = rvv_wire.command(self.decision,now=self.now,engaged=False,split_axes=True)
    events = self.events(previous=structs.CarState(),payload=disabled_wire)
    self.assertIn(EventName.pcmEnable,events.names)
    self.assertNotIn(EventName.psaAxesUnavailable,events.names)
    self.assertEqual(self.machine.update(events),(True,True))
    self.now += 10_000_000
    self.sm.logMonoTime[rvv_wire.SERVICE] = self.now
    self.decision = FollowingDecision('vision_following_candidate',target_kph=74,computed_ns=self.now,
      lead_rx_ns=self.now,model_ns=self.now,candidate_increase_permitted=True)
    with patch('openpilot.selfdrive.car.car_specific.time.monotonic_ns',return_value=self.now):
      self.assertEqual(self.machine.update(self.events()),(True,True))
    self.assertEqual(self.generator.t9_split_admission_ns,0)

  def test_initial_rvv_admission_expires_and_later_recovery_cannot_reenable(self):
    self.machine.state = log.SelfdriveState.OpenpilotState.disabled
    self.cs.steerFaultTemporary = True
    self.decision = FollowingDecision('waiting_for_lead')
    start = self.now
    def disabled_wire():
      return rvv_wire.command(self.decision,now=self.now,engaged=False,split_axes=True)
    with patch('openpilot.selfdrive.car.car_specific.time.monotonic_ns',return_value=start):
      self.assertEqual(self.machine.update(self.events(previous=structs.CarState(),payload=disabled_wire())),(True,True))
    self.now = start+250_000_000
    self.sm.logMonoTime[rvv_wire.SERVICE] = self.now
    with patch('openpilot.selfdrive.car.car_specific.time.monotonic_ns',return_value=self.now):
      self.assertEqual(self.machine.update(self.events(payload=disabled_wire())),(True,True))
    self.now += 1
    self.sm.logMonoTime[rvv_wire.SERVICE] = self.now
    with patch('openpilot.selfdrive.car.car_specific.time.monotonic_ns',return_value=self.now):
      self.assertEqual(self.machine.update(self.events(payload=disabled_wire())),(False,False))
      self.cs.steerFaultTemporary = False
      self.decision = FollowingDecision('vision_following_candidate',target_kph=74,computed_ns=self.now,
        lead_rx_ns=self.now,model_ns=self.now,candidate_increase_permitted=True)
      events = self.events()
      self.assertIn(EventName.psaAxesUnavailable,events.names)
      self.assertNotIn(EventName.pcmEnable,events.names)
      self.assertEqual(self.machine.update(events),(False,False))

  def test_low_speed_rvv_waits_without_lateral_or_native_engagement_timeout(self):
    self.cs.vEgo = self.cs.vEgoRaw = 40 / 3.6
    self.cs.steerFaultTemporary = True
    self.machine.state = log.SelfdriveState.OpenpilotState.disabled
    self.decision = FollowingDecision('waiting_for_lead')
    self.assertEqual(self.machine.update(self.events(previous=structs.CarState())), (True, True))
    self.now += 1_000_000_000
    self.sm.logMonoTime[rvv_wire.SERVICE] = self.now
    with patch('openpilot.selfdrive.car.car_specific.time.monotonic_ns',return_value=self.now):
      self.assertEqual(self.machine.update(self.events()), (True, True))
      self.cs.vEgo = self.cs.vEgoRaw = 39.99 / 3.6
      self.assertEqual(self.machine.update(self.events()), (False, False))

  def test_admission_window_never_defers_common_or_local_rvv_failure(self):
    for decision in (FollowingDecision('lead_lost',rearm_required=True),
                     FollowingDecision('waiting_for_lead',common_fault=True)):
      with self.subTest(decision=decision):
        self.setUp()
        self.machine.state = log.SelfdriveState.OpenpilotState.disabled
        self.cs.steerFaultTemporary = True
        self.decision = decision
        events = self.events(previous=structs.CarState())
        self.assertIn(EventName.psaAxesUnavailable,events.names)
        self.assertEqual(self.machine.update(events),(False,False))

  def test_common_stop_wire_latches_even_when_later_wire_is_healthy(self):
    # controlsd uses common_fault when an enabled session becomes
    # softDisabling/overriding; a later state recovery cannot re-admit axes.
    stopped = replace(self.decision,common_fault=True)
    payload = rvv_wire.command(stopped,now=self.now,engaged=False,split_axes=True)
    self.assertEqual(self.machine.update(self.events(payload=payload)),(False,False))
    events = self.events()
    self.assertIn(EventName.psaAxesUnavailable,events.names)
    self.assertNotIn(EventName.pcmEnable,events.names)
    self.assertEqual(self.machine.update(events),(False,False))

  def test_event_timestamp_is_checked_independently_of_fresh_payload(self):
    self.sm.logMonoTime[rvv_wire.SERVICE] = self.now-100_000_000
    with patch('openpilot.selfdrive.car.car_specific.time.monotonic_ns',return_value=self.now):
      self.assertEqual(self.machine.update(self.events()),(True,True))
      self.sm.logMonoTime[rvv_wire.SERVICE] -= 1
      self.assertEqual(self.machine.update(self.events()),(False,False))

  def test_missing_physical_admission_is_common_refusal(self):
    self.cs.blockPcmEnable = True
    events = self.events(previous=structs.CarState())
    self.assertNotIn(EventName.pcmEnable,events.names)
    self.assertTrue(events.contains(ET.NO_ENTRY))
    self.assertEqual(self.machine.update(events),(False,False))

  def test_common_vehicle_inhibitions_are_retained(self):
    self.cs.doorOpen = self.cs.seatbeltUnlatched = True
    events = self.events()
    self.assertIn(EventName.doorOpen,events.names)
    self.assertIn(EventName.seatbeltNotLatched,events.names)
    self.cs.cruiseState.enabled = False
    self.assertEqual(self.machine.update(self.events()),(False,False))

  def test_physical_off_then_on_is_required_after_common_stop(self):
    self.events(payload=b'')
    self.cs.cruiseState.enabled = False
    self.machine.update(self.events())
    previous = structs.CarState()
    self.cs.cruiseState.enabled = True
    events = self.events(previous=previous)
    self.assertNotIn(EventName.psaAxesUnavailable,events.names)
    self.assertIn(EventName.pcmEnable,events.names)


if __name__ == '__main__':
  unittest.main()
