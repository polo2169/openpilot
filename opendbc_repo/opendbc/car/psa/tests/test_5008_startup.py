import unittest
from unittest.mock import patch

from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import can_fingerprint, can_fingerprint_p87_startup
from opendbc.car.psa.fingerprints import FINGERPRINTS
from opendbc.car.psa.values import CAR


class TestP87Startup(unittest.TestCase):
  @staticmethod
  def receive(packets):
    iterator = iter(packets)
    return lambda wait_for_one=False: [next(iterator, [])]

  @staticmethod
  def rolling_packets(speed_kph):
    template = FINGERPRINTS[CAR.PSA_PEUGEOT_5008_P87][0]
    frames = [CanData(addr, bytes(length), 0) for addr, length in template.items()]
    # Speed values must never gate the ID/DLC fingerprint.
    frames = [f if f.address != 0x30D else CanData(0x30D, round(speed_kph * 100).to_bytes(2, 'big') * 4, 0) for f in frames]
    return [frames] * 103

  def test_identification_at_zero_and_driving_speeds(self):
    for speed in (0, 50, 67, 83):
      candidate, _ = can_fingerprint_p87_startup(self.receive(self.rolling_packets(speed)))
      self.assertEqual(candidate, CAR.PSA_PEUGEOT_5008_P87)

  def test_bad_initial_can_window_recovers_while_moving(self):
    bad = [[CanData(0x30D, b'\x00', 0), CanData(0x30D, b'\x00', 1)]] * 102
    candidate, _ = can_fingerprint_p87_startup(self.receive(bad + self.rolling_packets(77)))
    self.assertEqual(candidate, CAR.PSA_PEUGEOT_5008_P87)

  def test_incompatible_vehicle_never_forces_p87(self):
    wrong = [[CanData(0x30D, b'\x00', 0), CanData(0x30D, b'\x00', 1)]] * (5 * 102)
    candidate, _ = can_fingerprint_p87_startup(self.receive(wrong))
    self.assertIsNone(candidate)

  def test_silent_bus_has_a_bounded_window(self):
    with patch('opendbc.car.car_helpers.time.monotonic', side_effect=[0, 3]):
      candidate, _ = can_fingerprint(lambda wait_for_one=False: [], timeout=2.5)
    self.assertIsNone(candidate)
