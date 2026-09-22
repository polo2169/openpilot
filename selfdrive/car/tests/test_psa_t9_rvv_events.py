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


class TestRvvNativeEvents(unittest.TestCase):
  combined = False

  def setUp(self):
    with patch.dict(os.environ, {'PSA_T9_RVV_TEST': '1', 'PSA_T9_LATERAL_TEST': str(int(self.combined))}):
      self.cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.sm = MagicMock()
    self.sm.valid = {rvv_wire.SERVICE: True}
    with patch('cereal.messaging.SubMaster', return_value=self.sm):
      self.events = CarSpecificEvents(self.cp)
    self.cs = structs.CarState(vEgo=40/3.6, vEgoRaw=40/3.6, canValid=True,
                              gearShifter=structs.CarState.GearShifter.drive)
    self.cs.cruiseState.available = True
    self.cs.cruiseState.enabled = True
    self.cs.cruiseState.nonAdaptive = True
    self.now = time.monotonic_ns()
    self.decision = FollowingDecision('waiting_for_lead')

  def update(self):
    self.sm.__getitem__.return_value = rvv_wire.command(self.decision, now=self.now, engaged=True)
    return self.events.update(self.cs, structs.CarState(), structs.CarControl())

  def test_profile_speed_gate(self):
    events = self.update()
    self.assertNotIn(log.OnroadEvent.EventName.wrongCruiseMode, events.names)
    self.assertNotIn(log.OnroadEvent.EventName.belowEngageSpeed, events.names)
    self.assertIn(log.OnroadEvent.EventName.pcmEnable, events.names)
    self.cs.vEgoRaw = 39.99 / 3.6
    self.assertIn(log.OnroadEvent.EventName.pcmDisable, self.update().names)

  def test_missing_physical_rearm_prevents_native_enable(self):
    self.cs.blockPcmEnable = True
    events = self.update()
    self.assertNotIn(log.OnroadEvent.EventName.pcmEnable, events.names)
    self.assertIn(log.OnroadEvent.EventName.psaRvvUnavailable, events.names)
    self.assertTrue(events.contains(ET.NO_ENTRY))

  def test_profile_steering_events(self):
    self.cs.steeringPressed = True
    self.cs.steeringDisengage = True
    self.cs.steerFaultTemporary = True
    events = self.update()
    self.assertNotIn(log.OnroadEvent.EventName.steerOverride, events.names)
    self.assertNotIn(log.OnroadEvent.EventName.steerDisengage, events.names)
    self.assertNotIn(log.OnroadEvent.EventName.steerUnavailable, events.names)
    self.cs.doorOpen = True
    self.cs.seatbeltUnlatched = True
    events = self.update()
    self.assertIn(log.OnroadEvent.EventName.doorOpen, events.names)
    self.assertIn(log.OnroadEvent.EventName.seatbeltNotLatched, events.names)

  def test_failed_following_immediately_alerts_without_falsifying_stock_state(self):
    self.decision = replace(self.decision, reason='lead_lost', rearm_required=True)
    events = self.update()
    self.assertIn(log.OnroadEvent.EventName.psaRvvUnavailable, events.names)
    self.assertTrue(events.contains(ET.NO_ENTRY))
    machine = StateMachine()
    machine.state = log.SelfdriveState.OpenpilotState.enabled
    self.assertEqual(machine.update(events), (False, False))
    self.assertTrue(self.cs.cruiseState.enabled)

  def test_stale_command_requires_takeover_and_physical_off_allows_recovery(self):
    self.now -= 1_000_000_000
    self.assertIn(log.OnroadEvent.EventName.psaRvvUnavailable, self.update().names)
    self.cs.cruiseState.enabled = False
    self.assertNotIn(log.OnroadEvent.EventName.psaRvvUnavailable, self.update().names)


class TestCombinedNativeEvents(TestRvvNativeEvents):
  combined = True

  def setUp(self):
    super().setUp()
    self.cs.vEgo = self.cs.vEgoRaw = 75/3.6

  def test_profile_speed_gate(self):
    self.assertEqual(self.cp.safetyConfigs[0].safetyParam, 0x1312)
    self.cs.vEgo = self.cs.vEgoRaw = 40/3.6
    self.assertIn(log.OnroadEvent.EventName.pcmDisable, self.update().names)
    self.cs.vEgo = self.cs.vEgoRaw = 75/3.6
    self.assertNotIn(log.OnroadEvent.EventName.belowEngageSpeed, self.update().names)
    self.assertNotIn(log.OnroadEvent.EventName.wrongCruiseMode, self.update().names)

  def test_profile_steering_events(self):
    # Combined mode retains the steering intervention and EPS events.
    self.cs.vEgo = self.cs.vEgoRaw = 75/3.6
    self.cs.steeringPressed = self.cs.steeringDisengage = True
    events = self.update()
    self.assertIn(log.OnroadEvent.EventName.steerOverride, events.names)
    self.assertIn(log.OnroadEvent.EventName.steerDisengage, events.names)
    machine = StateMachine()
    machine.state = log.SelfdriveState.OpenpilotState.enabled
    self.assertEqual(machine.update(events), (False, False))
    self.cs.steeringPressed = self.cs.steeringDisengage = False
    self.cs.steerFaultTemporary = True
    events = self.update()
    self.assertIn(log.OnroadEvent.EventName.steerUnavailable, events.names)
    self.assertTrue(events.contains(ET.IMMEDIATE_DISABLE))
    machine.state = log.SelfdriveState.OpenpilotState.enabled
    self.assertEqual(machine.update(events), (False, False))


if __name__ == '__main__':
  unittest.main()
