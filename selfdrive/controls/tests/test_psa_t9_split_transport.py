"""Exercise Python publication -> actual C++ parser -> actual MCU C hooks.

Everything runs in desktop processes, with synthetic CAN frames in memory.
No messaging publisher, USB, SPI or CAN device is opened.
"""
from dataclasses import replace
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from opendbc.car.psa import rvv_wire
from opendbc.car.psa.rvv_following import FollowingDecision
from opendbc.safety.tests import test_psa_t9_split as safety_tests


class TestT9SplitTransport(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    compiler = shutil.which('c++')
    if compiler is None:
      raise unittest.SkipTest('A C++ compiler is required for the real transport parser')
    cls.temp = tempfile.TemporaryDirectory(prefix='t9-split-transport-')
    cls.addClassCleanup(cls.temp.cleanup)
    source = Path(cls.temp.name) / 'parse.cc'
    cls.parser = Path(cls.temp.name) / 'parse'
    source.write_text('''#include <cstdint>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include "selfdrive/pandad/psa_t9_rvv_wire.h"
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::vector<uint8_t> bytes(std::istreambuf_iterator<char>(std::cin), {});
  std::cout << t9_rvv_command(bytes.data(), bytes.size(), std::stoull(argv[1]),
                             std::stoull(argv[2]), true, true);
}
''')
    root = Path(__file__).resolve().parents[3]
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(root),
                    str(source), '-o', str(cls.parser)], check=True, capture_output=True)

  def setUp(self):
    self.mcu = safety_tests.TestT9SplitSafety(methodName='runTest')
    self.mcu.setUp()
    self.mcu.engage()
    self.assertTrue(self.mcu.tx(1))

  def candidate(self):
    now = self.mcu.now * 1000
    return FollowingDecision('vision_following_candidate', target_kph=74,
                             candidate_increase_permitted=True, computed_ns=now,
                             lead_rx_ns=now, model_ns=now)

  def transmit(self, decision, *, engaged=True, age_ns=0, mutation=None):
    now = self.mcu.now * 1000
    timestamp = now - age_ns
    payload = rvv_wire.command(decision, now=timestamp, engaged=engaged, split_axes=True)
    if mutation is not None:
      payload = mutation(payload)
    converted = subprocess.run([str(self.parser), str(now), str(timestamp)], input=payload,
                               capture_output=True, check=True)
    command = int(converted.stdout)
    self.mcu.sequence += 1
    status = self.mcu.safety.test_t9_rvv_request(command, self.mcu.sequence)
    return command, status

  def test_lateral_intervention_preserves_real_rvv_request(self):
    self.assertEqual(self.transmit(self.candidate()), (0x2100 | 74, 0))
    self.assertTrue(self.mcu.forwarded()[0])
    self.mcu.feed(driver=9)
    self.assertTrue(self.mcu.tx(0, 2, 0))
    self.assertEqual(self.transmit(self.candidate()), (0x2100 | 74, 0))
    self.assertTrue(self.mcu.forwarded()[0])
    self.mcu.assert_axes(False, True)
    self.mcu.feed(driver=0, eps=1)
    self.assertFalse(self.mcu.tx(0, 3, 0))
    self.mcu.assert_axes(False, True)

  def test_rvv_loss_preserves_lateral_and_cannot_resume_by_new_candidate(self):
    self.assertEqual(self.transmit(self.candidate())[1], 0)
    self.assertTrue(self.mcu.forwarded()[0])
    blocked = FollowingDecision('lead_lost', rearm_required=True, driver_intervention_required=True)
    self.assertEqual(self.transmit(blocked), (0x2400, 0))
    self.mcu.assert_axes(True, False)
    self.mcu.feed()
    self.assertTrue(self.mcu.tx(1))
    self.assertEqual(self.transmit(self.candidate())[1], 4)
    self.mcu.assert_axes(True, False)

  def test_common_monitor_failure_cannot_be_encoded_as_local_release(self):
    self.assertEqual(self.transmit(self.candidate())[1], 0)
    decision = replace(self.candidate(), common_fault=True)
    self.assertEqual(self.transmit(decision), (0x2200, 2))
    self.mcu.assert_axes(False, False)
    self.mcu.feed()
    self.assertEqual(self.transmit(self.candidate())[1], 4)

  def test_stale_or_cross_version_wire_stops_both_axes(self):
    for corruption in ('stale', 'version'):
      with self.subTest(corruption=corruption):
        self.setUp()
        self.assertEqual(self.transmit(self.candidate())[1], 0)
        args = {'age_ns': 100_000_001} if corruption == 'stale' else {
          'mutation': lambda payload: b'RVV1' + payload[4:]}
        self.assertEqual(self.transmit(FollowingDecision('waiting_for_lead'), **args), (0x2200, 2))
        self.mcu.assert_axes(False, False)

  def test_delayed_host_engagement_does_not_consume_physical_admission(self):
    waiting = FollowingDecision('waiting_for_lead')
    self.assertEqual(self.transmit(waiting, engaged=False), (0x2000, 0))
    self.mcu.assert_axes(True, True)
    self.assertEqual(self.transmit(waiting), (0x2100, 0))
    self.assertEqual(self.transmit(self.candidate()), (0x2100 | 74, 0))
    self.assertTrue(self.mcu.forwarded()[0])
    self.assertEqual(self.transmit(waiting, engaged=False), (0x2000, 0))
    self.mcu.assert_axes(True, False)

  def test_first_enabled_block_latches_only_rvv(self):
    blocked = FollowingDecision('fixed_cruise_cannot_handle_critical_lead', rearm_required=True)
    self.assertEqual(self.transmit(blocked, engaged=False), (0x2000, 0))
    self.mcu.assert_axes(True, True)
    self.assertEqual(self.transmit(blocked), (0x2400, 0))
    self.mcu.assert_axes(True, False)
    self.assertEqual(self.transmit(self.candidate())[1], 4)


if __name__ == '__main__':
  unittest.main()
