import unittest

from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from openpilot.cereal import log
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.selfdrived.stock_cruise import cruise_mismatch


class TestP87StockCruise(unittest.TestCase):
  def params(self, candidate=CAR.PSA_PEUGEOT_5008_P87):
    return CarInterface.get_non_essential_params(candidate)

  def test_stock_acc_while_disengaged_is_independent(self):
    cp = self.params()
    for enabled in (False, True):
      self.assertFalse(cruise_mismatch(cp, enabled, True))
      self.assertFalse(cruise_mismatch(cp, enabled, False))

  def test_mismatch_check_remains_for_other_ports_and_op_long(self):
    cp = self.params(CAR.PSA_PEUGEOT_3008)
    self.assertTrue(cruise_mismatch(cp, False, True))
    self.assertFalse(cruise_mismatch(cp, True, True))
    cp.pcmCruise = False
    self.assertTrue(cruise_mismatch(cp, True, True))
    cp = self.params()
    cp.openpilotLongitudinalControl = True
    self.assertTrue(cruise_mismatch(cp, False, True))

  def test_throttle_cancels_p87_even_without_generic_accelerator_setting(self):
    cp = self.params()
    cs, prev = structs.CarState(), structs.CarState()
    cs.cruiseState.available = True
    cs.cruiseState.enabled = True
    cs.gearShifter = structs.CarState.GearShifter.drive
    cs.gasPressed = True
    cs.blockPcmEnable = True
    events = CarEvents(cp).update(cs, prev, structs.CarControl())
    self.assertTrue(events.has(log.OnroadEvent.EventName.pedalPressed))
    self.assertFalse(events.has(log.OnroadEvent.EventName.pcmEnable))
    prev.gasPressed = True
    events = CarEvents(cp).update(cs, prev, structs.CarControl())
    self.assertTrue(events.has(log.OnroadEvent.EventName.pedalPressed))

  def test_legacy_psa_accelerator_override_is_unchanged(self):
    cp = self.params(CAR.PSA_PEUGEOT_3008)
    cs = structs.CarState(gasPressed=True)
    events = CarEvents(cp).update(cs, structs.CarState(), structs.CarControl())
    self.assertFalse(events.has(log.OnroadEvent.EventName.pedalPressed))
    self.assertTrue(events.has(log.OnroadEvent.EventName.gasPressedOverride))


if __name__ == '__main__':
  unittest.main()
