import os
import struct
import unittest
from unittest.mock import patch

from cereal import log
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.selfdrived.events import ET, EVENTS
from openpilot.selfdrive.selfdrived.state import StateMachine

EventName = log.OnroadEvent.EventName


class TestT9ConventionalCruiseEvents(unittest.TestCase):
  def params(self, active=True, candidate=CAR.PSA_PEUGEOT_308_T9):
    with patch.dict(os.environ, {'PSA_T9_LATERAL_TEST': '1' if active else '0'}):
      return CarInterface.get_non_essential_params(candidate)

  def state(self, speed=75):
    cs = structs.CarState(vEgo=speed/3.6, vEgoRaw=speed/3.6, canValid=True,
                          gearShifter=structs.CarState.GearShifter.drive)
    cs.cruiseState.available = True
    cs.cruiseState.enabled = True
    cs.cruiseState.nonAdaptive = True
    return cs

  def events(self, cp, cs=None):
    return CarSpecificEvents(cp).update(cs or self.state(), structs.CarState(), structs.CarControl()).names

  def test_explicit_lateral_profile_accepts_physical_conventional_cruise(self):
    cp = self.params()
    events = self.events(cp)
    self.assertNotIn(EventName.wrongCruiseMode, events)
    self.assertIn(EventName.pcmEnable, events)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertTrue(self.state().cruiseState.nonAdaptive)

  def test_exception_does_not_apply_to_other_profiles_or_longitudinal(self):
    cases = [self.params(active=False), self.params(candidate=CAR.PSA_PEUGEOT_208)]
    longitudinal = self.params(); longitudinal.openpilotLongitudinalControl = True; cases.append(longitudinal)
    different_safety = self.params(); different_safety.safetyConfigs[0].safetyParam = 0; cases.append(different_safety)
    non_pcm = self.params(); non_pcm.pcmCruise = False; cases.append(non_pcm)
    for cp in cases:
      self.assertIn(EventName.wrongCruiseMode, self.events(cp))

  def test_vehicle_inhibitions_are_retained(self):
    cs = self.state(); cs.doorOpen = True; cs.seatbeltUnlatched = True; cs.steerFaultPermanent = True
    events = self.events(self.params(), cs)
    for event in (EventName.doorOpen, EventName.seatbeltNotLatched, EventName.steerUnavailable):
      self.assertIn(event, events)

  def test_speed_envelope_is_visible_before_controller_refusal(self):
    self.assertIn(EventName.belowEngageSpeed, self.events(self.params(), self.state(60)))
    self.assertIn(EventName.speedTooHigh, self.events(self.params(), self.state(140.01)))
    for speed in (75, 90, 100, 110, 130, 140):
      state = self.state(speed)
      state.vEgoRaw = struct.unpack('<f', struct.pack('<f', state.vEgoRaw))[0]
      events = self.events(self.params(), state)
      self.assertNotIn(EventName.belowEngageSpeed, events)
      self.assertNotIn(EventName.speedTooHigh, events)

  def test_t9_driver_cut_immediately_disables_and_blocks_further_engagement(self):
    cp = self.params()
    generator = CarSpecificEvents(cp)
    machine = StateMachine()
    machine.state = log.SelfdriveState.OpenpilotState.enabled
    state = self.state()
    state.steeringDisengage = True
    state.blockPcmEnable = False
    for previous in (self.state(), state):
      events = generator.update(state, previous, structs.CarControl())
      self.assertIn(EventName.steerDisengage, events.names)
      self.assertTrue(events.contains(ET.USER_DISABLE))
      self.assertTrue(events.contains(ET.NO_ENTRY))
      self.assertNotIn(EventName.pcmEnable, events.names)
      self.assertNotIn(EventName.steerTempUnavailable, events.names)
      self.assertNotIn(EventName.steerTempUnavailableSilent, events.names)
      self.assertEqual(machine.update(events), (False, False))
      self.assertEqual(machine.state, log.SelfdriveState.OpenpilotState.disabled)

    # A new physical RVV edge is visible as an attempt, but NO_ENTRY prevents
    # engagement and gives the driver the real "Steering Pressed" reason.
    previous = self.state(); previous.cruiseState.enabled = False
    events = generator.update(state, previous, structs.CarControl())
    self.assertIn(EventName.pcmEnable, events.names)
    self.assertIn(EventName.steerDisengage, events.names)
    self.assertEqual(machine.update(events), (False, False))
    self.assertIn(ET.NO_ENTRY, machine.current_alert_types)

  def test_speed_exit_disengages_instead_of_waiting_for_mismatch(self):
    generator = CarSpecificEvents(self.params())
    for speed, reason in ((67., EventName.belowEngageSpeed), (140.01, EventName.speedTooHigh)):
      machine = StateMachine()
      machine.state = log.SelfdriveState.OpenpilotState.enabled
      state = self.state(speed)
      events = generator.update(state, self.state(75), structs.CarControl(enabled=True))
      self.assertIn(reason, events.names)
      self.assertIn(EventName.pcmDisable, events.names)
      self.assertEqual(machine.update(events), (False, False))
      self.assertTrue(state.cruiseState.enabled)
      previous = self.state(speed); previous.cruiseState.enabled = False
      events = generator.update(state, previous, structs.CarControl())
      self.assertIn(EventName.pcmEnable, events.names)
      self.assertEqual(machine.update(events), (False, False))
      self.assertIn(ET.NO_ENTRY, machine.current_alert_types)
    other = CarSpecificEvents(self.params(active=False))
    self.assertNotIn(EventName.pcmDisable,
                     other.update(self.state(140.01), self.state(75), structs.CarControl()).names)

  def test_non_t9_profiles_keep_existing_edge_behavior(self):
    state = self.state(); state.steeringDisengage = True
    generator = CarSpecificEvents(self.params(active=False))
    self.assertIn(EventName.steerDisengage, generator.update(state, self.state(), structs.CarControl()).names)
    self.assertNotIn(EventName.steerDisengage, generator.update(state, state, structs.CarControl()).names)

  def test_real_eps_fault_is_not_hidden_by_disengagement(self):
    state = self.state(); state.steeringDisengage = True; state.steerFaultTemporary = True
    events = self.events(self.params(), state)
    self.assertIn(EventName.steerDisengage, events)
    self.assertIn(EventName.steerUnavailable, events)

  def test_eps_loss_disables_immediately_even_after_recent_driver_input(self):
    cp = self.params()
    for pressed in (False, True):
      generator = CarSpecificEvents(cp)
      state = self.state(); state.steerFaultTemporary = True; state.blockPcmEnable = True
      state.steeringPressed = pressed
      machine = StateMachine()
      machine.state = log.SelfdriveState.OpenpilotState.enabled
      events = generator.update(state, self.state(), structs.CarControl(enabled=True, latActive=True))
      self.assertIn(EventName.steerUnavailable, events.names)
      self.assertTrue(events.contains(ET.IMMEDIATE_DISABLE))
      self.assertTrue(events.contains(ET.NO_ENTRY))
      self.assertFalse(events.contains(ET.SOFT_DISABLE))
      self.assertEqual(machine.update(events), (False, False))
      self.assertIn(ET.IMMEDIATE_DISABLE, machine.current_alert_types)
      self.assertFalse(state.steerFaultPermanent)
      self.assertTrue(state.cruiseState.enabled)
      wire = next(e for e in events.to_msg() if e.name == EventName.steerUnavailable)
      self.assertTrue(wire.immediateDisable)
      self.assertTrue(wire.noEntry)
      self.assertFalse(wire.softDisable)

      # EPS recovery by itself is not an enable edge or an automatic restart.
      recovered = self.state()
      events = generator.update(recovered, state, structs.CarControl())
      self.assertNotIn(EventName.pcmEnable, events.names)
      self.assertEqual(machine.update(events), (False, False))

  def test_t9_loss_alert_is_truthful_and_legacy_fault_labels_are_preserved(self):
    cp = self.params(); cs = self.state(); cs.steerFaultTemporary = True
    generator = CarSpecificEvents(cp)
    events = generator.update(cs, self.state(), structs.CarControl())
    alerts = events.create_alerts([ET.IMMEDIATE_DISABLE, ET.PERMANENT, ET.NO_ENTRY],
                                 [cp, cs, None, True, 300, 0])
    self.assertEqual(len(alerts), 3)
    self.assertTrue(all('Steering Assist Unavailable' in (a.alert_text_1, a.alert_text_2) for a in alerts))
    self.assertTrue(all('Restart' not in a.alert_text_1+a.alert_text_2 for a in alerts))
    for other_cp, permanent in ((self.params(active=False),False), (self.params(),True)):
      cs.steerFaultPermanent = permanent
      for et in (ET.IMMEDIATE_DISABLE, ET.PERMANENT, ET.NO_ENTRY):
        alert = EVENTS[EventName.steerUnavailable][et](other_cp,cs,None,True,300,0)
        self.assertIn('Restart',alert.alert_text_1+alert.alert_text_2)

  def test_other_profiles_keep_temporary_steering_soft_disable(self):
    generator = CarSpecificEvents(self.params(candidate=CAR.PSA_PEUGEOT_208))
    generator.silent_steer_warning = False
    generator.steering_unpressed = 200
    state = self.state(); state.steerFaultTemporary = True; state.cruiseState.nonAdaptive = False
    events = generator.update(state,self.state(),structs.CarControl())
    self.assertIn(EventName.steerTempUnavailable,events.names)
    self.assertNotIn(EventName.steerUnavailable,events.names)
    self.assertTrue(events.contains(ET.SOFT_DISABLE))


if __name__ == '__main__':
  unittest.main()
