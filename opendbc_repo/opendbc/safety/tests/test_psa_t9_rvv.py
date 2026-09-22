import unittest

from opendbc.safety.tests import test_psa_t9 as lateral_tests


class TestT9RvvSafety(unittest.TestCase):
  TX_MSGS = []  # only MCU forwarding can reach the car; host CAN allowlist is empty
  def setUp(self):
    self.h = lateral_tests.TestT9LateralSafety()
    self.h.setUp()
    self.s = self.h.safety
    self.s.set_timer(0)
    self.s.set_safety_hooks(31, 0x1310)
    self.s.init_tests()
    self.h.now = 0
    self.seq = 0
    for _ in range(45):
      self.h.feed()

  def request(self, target, enabled=True, seq=None):
    self.seq = (self.seq + 1) & 0xFFFF if seq is None else seq
    return self.s.test_t9_rvv_request(0x1000 | (int(enabled) << 8) | target, self.seq)

  def stock(self, target=75, active=True, counter=0, mode=1):
    parity = ((target >> 4).bit_count() % 2) * 2 + (target & 15).bit_count() % 2
    return self.h.raw(0x50E, [0xC5 | (parity << 4), 17, 22, 33, 44, 55, target,
                            (int(active) << 7) | (mode << 5) | counter], 2)

  def rewrite(self, frame=None):
    frame = self.stock() if frame is None else frame
    # Exact driver order: forwarding (incl. rewriting) before safety RX.
    original = self.h.raw(frame[0].addr, bytes(frame[0].data[0:8]), frame[0].bus)
    destination = self.s.safety_fwd_hook(frame[0].bus, frame[0].addr)
    modified = self.s.test_t9_rvv_rewrite(frame, destination)
    self.s.safety_rx_hook(original)
    return modified, bytes(frame[0].data[0:8])

  def start(self, speed=75):
    self.h.feed(cruise=True, speed=speed)
    self.assertTrue(self.s.get_controls_allowed())
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.rewrite()[0])

  def test_no_host_can_and_other_profiles_cannot_use_mailbox(self):
    self.start()
    for bus in range(4):
      for address in range(0x800):
        self.assertFalse(self.s.safety_tx_hook(self.h.raw(address, bytes(8), bus)))
    for mode, param in [(19, 0), (31, 0x1308), (31, 0x1309), (31, 0x1311), (31, 0)]:
      self.s.set_safety_hooks(mode, param)
      self.assertNotEqual(self.request(74), 0)
      self.assertFalse(self.s.test_t9_rvv_rewrite(self.stock(), 0))

  def test_needs_physical_off_on_and_accepts_40_boundary_without_eps(self):
    self.assertNotEqual(self.request(74), 0)
    self.h.feed(speed=40, eps=0, driver=12)
    self.start(speed=40)
    self.assertFalse(self.s.safety_tx_hook(self.h.command(1)))

  def test_ramp_and_upward_recovery_preserve_all_opaque_bits_and_counter(self):
    self.start()
    values = []
    for step in range(30):
      self.h.feed()
      self.assertEqual(self.request(73 if step < 20 else 75), 0)
      frame = self.stock(counter=(step * 2) % 16)
      original = bytes(frame[0].data[0:8])
      changed, data = self.rewrite(frame)
      self.assertTrue(changed)
      self.assertEqual(data[1:6], original[1:6])
      self.assertEqual(data[7], original[7])
      self.assertEqual(data[0] & 0xCF, original[0] & 0xCF)
      expected_parity = ((data[6] >> 4).bit_count() % 2) * 2 + (data[6] & 15).bit_count() % 2
      self.assertEqual((data[0] >> 4) & 3, expected_parity)
      values.append(data[6])
    self.assertEqual(values[0], 75)
    self.assertEqual(min(values), 73)
    self.assertEqual(values[-1], 74)
    last_change = -1  # start() already rewrote once, 50 ms before values[0].
    for i in range(1, len(values)):
      delta = values[i] - values[i-1]
      if delta:
        self.assertEqual(abs(delta), 1)
        self.assertGreaterEqual((i-last_change)*50, 100 if delta < 0 else 500)
        last_change = i

  def test_large_reduction_still_needs_forty_separate_bounded_steps(self):
    self.h.feed(cruise=True, speed=130, setpoint=130)
    self.assertTrue(self.s.get_controls_allowed())
    values = []
    for step in range(81):
      if step:
        self.h.feed()
      self.assertEqual(self.request(90), 0)
      changed, data = self.rewrite(self.stock(130))
      self.assertTrue(changed)
      values.append(data[6])
      self.assertEqual(data[6], 130 - step // 2)
    self.assertEqual((values[0], values[-1]), (130, 90))

  def test_current_cancel_and_lower_ceiling_precede_rx_update(self):
    self.start()
    changed, data = self.rewrite(self.stock(45))
    self.assertTrue(changed)
    self.assertEqual(data[6], 45)
    frame = self.stock(255, active=False)
    original = bytes(frame[0].data[0:8])
    changed, data = self.rewrite(frame)
    self.assertFalse(changed)
    self.assertEqual(data, original)
    self.assertFalse(self.s.get_controls_allowed())

  def test_brake_gas_door_belt_gear_speed_and_ignition_cut_require_rearm(self):
    for change in ({'brake': True}, {'pedal': 1}, {'doors': True}, {'belt': 1}, {'gear': 9},
                   {'speed': 39.99}, {'speed': 140.01}, {'ignition': False}, {'park': 1}, {'reverse': True}):
      with self.subTest(change=change):
        self.setUp(); self.start()
        old = self.h.settings.copy()
        self.h.feed(**change)
        self.assertFalse(self.rewrite()[0])
        self.h.feed(**old)
        self.assertNotEqual(self.request(74), 0)
        self.h.feed(cruise=False)
        self.h.feed(cruise=True)
        self.assertEqual(self.request(74), 0)

  def test_stale_request_restores_original_and_cannot_revive(self):
    self.start()
    for _ in range(4):
      self.h.feed()
    changed, data = self.rewrite()
    self.assertFalse(changed)
    self.assertEqual(data[6], 75)
    self.assertNotEqual(self.request(74), 0)
    self.h.feed(cruise=False)
    self.h.feed(cruise=True)
    self.assertEqual(self.request(74), 0)

  def test_late_command_before_next_stock_also_cannot_revive(self):
    self.start()
    for _ in range(4):
      self.h.feed()
    self.assertNotEqual(self.request(74), 0)
    self.assertFalse(self.rewrite()[0])

  def test_spi_duplicate_does_not_renew_lease_or_overwrite_target(self):
    self.start()
    sequence = self.seq
    for _ in range(4):
      self.h.feed()
      self.assertEqual(self.request(75, seq=sequence), 3)
    self.assertFalse(self.rewrite()[0])

  def test_invalid_target_parity_mode_length_and_protocol_cannot_modify(self):
    for value in (39, 76, 141, 255):
      self.setUp(); self.start()
      self.assertNotEqual(self.request(value), 0)
      self.assertFalse(self.rewrite()[0])
    for change in ('parity', 'limiter', 'fd', 'extended', 'length'):
      self.setUp(); self.start()
      frame = self.stock(mode=2 if change == 'limiter' else 1)
      if change == 'parity': frame[0].data[0] ^= 0x10
      if change == 'fd': frame[0].fd = 1
      if change == 'extended': frame[0].extended = 1
      if change == 'length': frame[0].data_len_code = 7
      original = bytes(frame[0].data[0:8])
      self.assertFalse(self.s.test_t9_rvv_rewrite(frame, 0))
      self.assertEqual(bytes(frame[0].data[0:8]), original)
    self.setUp(); self.start()
    self.assertNotEqual(self.s.test_t9_rvv_request(0x214A, 99), 0)
    self.assertFalse(self.rewrite()[0])

  def test_waiting_for_lead_does_not_race_native_engagement(self):
    self.h.feed(cruise=True)
    self.assertEqual(self.request(0, enabled=False), 0)
    self.assertTrue(self.s.get_controls_allowed())
    self.assertFalse(self.rewrite()[0])
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.rewrite()[0])
    self.assertEqual(self.request(0), 0)
    self.assertFalse(self.s.get_controls_allowed())
    self.assertNotEqual(self.request(74), 0)

  def test_stale_vehicle_rx_and_wrong_topology_block(self):
    self.start()
    self.s.set_timer(self.h.now + 250001)
    self.assertFalse(self.rewrite()[0])
    self.setUp(); self.start()
    self.s.safety_fwd_hook(0, 0x50E)
    self.assertNotEqual(self.request(74), 0)
    self.assertFalse(self.rewrite()[0])

  def test_microsecond_timer_wrap_does_not_disable_a_fresh_engagement(self):
    self.h.now = 0xFFFFFFFF - 400000
    self.h.feed(cruise=False)
    self.start()
    for _ in range(20):
      if self.h.now + 50000 > 0xFFFFFFFF:
        self.h.now -= 1 << 32
      self.h.feed()
      self.assertEqual(self.request(74), 0)
      self.assertTrue(self.rewrite()[0])
      self.assertTrue(self.s.get_controls_allowed())

  def test_counter_wrap_and_duplicate_sequence_rejected(self):
    self.h.feed(cruise=True)
    self.assertEqual(self.request(74, seq=65535), 0)
    self.assertEqual(self.request(74, seq=0), 0)
    self.assertEqual(self.request(74, seq=0), 3)
    self.assertEqual(self.request(74, seq=65535), 3)


if __name__ == '__main__':
  unittest.main()
