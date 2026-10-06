#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import make_msg


class TestPsaT15Ignition(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  @staticmethod
  def _t15_msg(t15: bool, *, engine_running: bool = False, bus: int = 0, length: int = 8):
    dat = bytearray(8)
    if engine_running:
      dat[5] = 0x6
    if t15:
      dat[6] = 0x40
    return make_msg(bus, 0x348, dat=bytes(dat[:length]))

  def test_t15_controls_ignition(self):
    self.safety.ignition_can_hook(self._t15_msg(True))
    self.assertTrue(self.safety.get_ignition_can())

    self.safety.ignition_can_hook(self._t15_msg(False))
    self.assertFalse(self.safety.get_ignition_can())

  def test_engine_running_bits_do_not_replace_t15(self):
    self.safety.ignition_can_hook(self._t15_msg(False, engine_running=True))
    self.assertFalse(self.safety.get_ignition_can())

  def test_network_state_frame_does_not_override_t15(self):
    self.safety.ignition_can_hook(self._t15_msg(True))
    self.safety.ignition_can_hook(make_msg(0, 0x432, dat=b"\x00" * 8))
    self.assertTrue(self.safety.get_ignition_can())

  def test_wrong_bus_and_length_are_ignored(self):
    self.safety.ignition_can_hook(self._t15_msg(True, bus=1))
    self.assertFalse(self.safety.get_ignition_can())

    self.safety.ignition_can_hook(self._t15_msg(True, length=7))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
