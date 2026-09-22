import unittest

from opendbc.safety.tests import test_psa_t9 as lateral_tests


class TestT9SplitSafety(unittest.TestCase):
  """The real C hooks, independent permissions and shared actuator limits."""
  # The T9 variants share a frame identity; their allowlists are tested here.
  TX_MSGS = lateral_tests.TestT9LateralSafety.TX_MSGS
  raw = lateral_tests.TestT9LateralSafety.raw
  packed = lateral_tests.TestT9LateralSafety.packed
  feed = lateral_tests.TestT9LateralSafety.feed
  stock_rvv = lateral_tests.TestT9LateralSafety.stock_rvv
  command = lateral_tests.TestT9LateralSafety.command
  tx = lateral_tests.TestT9LateralSafety.tx
  engage = lateral_tests.TestT9LateralSafety.engage

  def setUp(self):
    lateral_tests.TestT9LateralSafety.setUp(self)
    self.safety.set_safety_hooks(31, 0x1314)
    self.safety.init_tests()
    for _ in range(45):
      self.feed()
    self.sequence = 0

  def request(self, target, enabled=True):
    self.sequence += 1
    return self.safety.test_t9_rvv_request(0x2000 | (0x100 if enabled else 0) | target, self.sequence)

  def forwarded(self):
    frame = self.stock_rvv(self.settings['cruise'], setpoint=self.settings['setpoint'])
    destination = self.safety.safety_fwd_hook(2, 0x50E)
    rewritten = self.safety.test_t9_rvv_rewrite(frame, destination)
    return bool(rewritten), bytes(frame[0].data[0:8])

  def assert_axes(self, lateral, rvv):
    self.assertEqual(bool(self.safety.get_t9_split_lateral_allowed()), lateral)
    self.assertEqual(bool(self.safety.get_t9_split_rvv_allowed()), rvv)
    self.assertEqual(bool(self.safety.get_controls_allowed()), lateral or rvv)
    expected = int(lateral or rvv) | (int(lateral) << 1) | (int(rvv) << 2)
    self.assertEqual(self.safety.test_t9_rvv_permission_bits(), expected)

  def start_both(self):
    self.engage()
    self.assertTrue(self.tx(1))
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.forwarded()[0])
    self.assert_axes(True, True)

  def test_physical_edge_is_required_and_waiting_for_lead_preserves_admission(self):
    self.assert_axes(False, False)
    self.assertNotEqual(self.request(74), 0)
    self.engage()
    self.assertEqual(self.request(0), 0)
    self.assert_axes(True, True)
    self.assertFalse(self.forwarded()[0])
    self.assertTrue(self.tx(1))
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.forwarded()[0])

  def test_driver_and_blinker_pause_require_zero_within_existing_session(self):
    for change in ({'driver': -16}, {'driver': 16}, {'blinker': 1}, {'blinker': 2}, {'blinker': 3}):
      with self.subTest(change=change):
        self.setUp(); self.start_both()
        self.feed(**change)
        self.assert_axes(True, True)
        self.assertFalse(self.tx(1))
        self.assertTrue(self.tx(0))
        self.assertEqual(self.request(74), 0)
        self.assertTrue(self.forwarded()[0])
        self.feed(driver=0, blinker=0)
        self.assertTrue(self.tx(1))
        self.assert_axes(True, True)

  def test_fifteen_remains_allowed_and_eps_loss_during_pause_is_latched(self):
    for sign in (-1, 1):
      self.setUp(); self.start_both()
      self.feed(driver=sign * 15)
      self.assertTrue(self.tx(1))
      self.feed(driver=sign * 16)
      self.assertTrue(self.tx(0))
      self.feed(driver=0, eps=0)
      self.assert_axes(False, True)
      self.assertTrue(self.tx(0, 2, 0))
      self.feed(eps=1)
      self.assertFalse(self.tx(0, 3, 0))

  def test_admission_survives_native_engagement_delay_but_not_a_later_disable(self):
    self.engage()
    self.assertEqual(self.request(0, enabled=False), 0)
    self.assert_axes(True, True)
    self.assertEqual(self.request(0), 0)
    self.assert_axes(True, True)
    self.assertEqual(self.request(0, enabled=False), 0)
    self.assert_axes(True, False)
    self.assertNotEqual(self.request(74), 0)
    self.assertTrue(self.tx(0, 2, 0))
    self.feed(cruise=False, eps=1)
    self.engage()
    self.assertEqual(self.request(0, enabled=False), 0)
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.forwarded()[0])

  def test_eps_withdrawal_is_local_but_freshness_failure_is_common(self):
    for eps in (0, 1, 2, 4, 7):
      with self.subTest(eps=eps):
        self.setUp(); self.start_both()
        self.feed(eps=eps)
        self.assert_axes(False, True)
        self.assertFalse(self.tx(1))
        self.assertTrue(self.tx(0, 2, 0))
        self.assertEqual(self.request(74), 0)
        self.assertTrue(self.forwarded()[0])
        self.feed(eps=1)
        self.assert_axes(False, True)
        self.assertFalse(self.tx(0, 3, 0))
    self.now += 300000
    self.safety.set_timer(self.now)
    self.assertNotEqual(self.request(74), 0)
    self.assert_axes(False, False)
    self.feed(eps=1)
    self.assertNotEqual(self.request(74), 0)

  def test_manual_admission_does_not_grant_lateral_while_eps_is_unavailable(self):
    self.feed(cruise=True, eps=0)
    self.assert_axes(False, True)
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.forwarded()[0])
    self.assertFalse(self.tx(0, 3, 0))
    self.feed(eps=1)
    self.assert_axes(False, True)
    self.assertFalse(self.tx(0, 3, 0))
    self.feed(cruise=False)
    self.engage()
    self.assert_axes(True, True)

  def test_eps_ack_deadline_remains_local_and_cannot_be_retried(self):
    self.feed(cruise=True)
    self.assertTrue(self.tx(0, 4, 1))
    self.assertEqual(self.request(74), 0)
    for _ in range(9):
      self.feed(eps=1)
      self.assertTrue(self.tx(0, 4, 1))
      self.assertEqual(self.request(74), 0)
    self.feed(eps=1)
    self.assert_axes(False, True)
    self.assertFalse(self.tx(0, 4, 1))
    self.assertTrue(self.tx(0, 2, 0))
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.forwarded()[0])
    self.feed(eps=3)
    self.assertFalse(self.tx(1))
    self.assert_axes(False, True)

  def test_explicit_rvv_axis_fault_before_first_target_is_local_and_latched(self):
    self.engage()
    self.sequence += 1
    self.assertEqual(self.safety.test_t9_rvv_request(0x2400, self.sequence), 0)
    self.assert_axes(True, False)
    self.assertTrue(self.tx(1))
    self.feed()
    self.assertNotEqual(self.request(74), 0)
    self.assert_axes(True, False)

  def test_axis_stop_during_pending_can_admission_cannot_be_undone(self):
    for axis in ('lat', 'rvv'):
      with self.subTest(axis=axis):
        self.setUp()
        # The manual edge is fresh, but the rest of the batch is still stale.
        self.now += 300000; self.safety.set_timer(self.now)
        self.safety.safety_rx_hook(self.stock_rvv(True))
        self.safety.safety_rx_hook(self.packed('T9_ENGINE_DYNAMICS_208', {'CruiseStateCandidate': 2}))
        self.assert_axes(False, False)
        if axis == 'rvv':
          self.sequence += 1
          self.assertEqual(self.safety.test_t9_rvv_request(0x2400, self.sequence), 0)
        else:
          self.safety.safety_rx_hook(self.packed('T9_STEERING_TORQUE_2F5', {'DriverTorqueRaw': 16}))
        self.feed(cruise=True, driver=0)
        self.assert_axes(axis != 'lat', axis != 'rvv')
        self.feed()
        self.assert_axes(axis != 'lat', axis != 'rvv')

  def test_local_rvv_release_preserves_lateral_but_latches_longitudinal_off(self):
    for enabled in (False, True):
      with self.subTest(enabled=enabled):
        self.setUp(); self.start_both()
        self.assertEqual(self.request(0, enabled=enabled), 0)
        self.assert_axes(True, False)
        self.assertFalse(self.forwarded()[0])
        for _ in range(8):
          self.feed()
          self.assertTrue(self.tx(1))
          self.assertNotEqual(self.request(74), 0)
          self.assert_axes(True, False)
        self.assertTrue(self.tx(0, 2, 0))
        self.feed(cruise=False, eps=1)
        self.engage()
        self.assertEqual(self.request(74), 0)
        self.assert_axes(True, True)

  def test_host_lateral_release_is_local_and_does_not_allow_new_takeover(self):
    self.start_both()
    self.assertTrue(self.tx(0, 2, 0))
    self.assert_axes(False, True)
    self.feed(eps=1)
    self.assertEqual(self.request(74), 0)
    self.assertTrue(self.forwarded()[0])
    self.assertFalse(self.tx(0, 3, 0))

  def test_common_vehicle_interruptions_stop_both_without_recovery(self):
    cases = ({'brake': True}, {'pedal': 1}, {'speed': 39.99}, {'speed': 140.01},
             {'doors': True}, {'belt': 1}, {'gear': 9}, {'park': 1},
             {'reverse': True}, {'ignition': False}, {'mode': 2})
    for change in cases:
      with self.subTest(change=change):
        self.setUp(); self.start_both()
        before = self.settings.copy()
        self.feed(**change)
        self.assert_axes(False, False)
        self.assertFalse(self.forwarded()[0])
        self.assertTrue(self.tx(0, 2, 0))
        self.feed(**(before | {'eps': 1}))
        self.assertNotEqual(self.request(74), 0)
        self.assertFalse(self.tx(0, 3, 0))
        self.assert_axes(False, False)

  def test_rvv_admits_from_forty_without_any_lateral_command(self):
    for speed in (39.99, 40., 50., 67.09):
      with self.subTest(speed=speed):
        self.setUp()
        self.feed(speed=speed, cruise=False)
        self.feed(cruise=True)
        available = speed >= 40.
        self.assert_axes(False, available)
        self.assertFalse(self.tx(1))
        self.assertFalse(self.tx(0, 4, 1))
        self.assertEqual(self.request(40) == 0, available)
        self.assertEqual(self.forwarded()[0], available)

  def test_crossing_lateral_speed_floor_keeps_rvv_and_requires_lateral_rearm(self):
    self.start_both()
    self.feed(speed=67.09)
    self.assert_axes(False, True)
    self.assertFalse(self.tx(1))
    self.assertTrue(self.tx(0, 2, 0))
    self.assertEqual(self.request(40), 0)
    self.assertTrue(self.forwarded()[0])
    self.feed(speed=75, eps=1)
    self.assert_axes(False, True)
    self.assertFalse(self.tx(0, 3, 0))
    self.feed(speed=39.99)
    self.assert_axes(False, False)
    self.feed(speed=40)
    self.assert_axes(False, False)

  def test_common_stop_stays_latched_when_only_one_axis_remains(self):
    for first_axis in ('lat', 'rvv'):
      with self.subTest(first_axis=first_axis):
        self.setUp(); self.start_both()
        if first_axis == 'lat':
          self.feed(driver=16)
          self.assertTrue(self.tx(0, 2, 0))
        else:
          self.assertEqual(self.request(0), 0)
        self.safety.set_controls_allowed(False)  # heartbeat/transport cut
        self.feed(driver=0)
        self.assert_axes(False, False)
        self.assertNotEqual(self.request(74), 0)
        self.assertFalse(self.tx(1))

  def test_global_cut_is_consumed_by_each_entry_point_before_it_can_be_undone(self):
    for entry in ('rx', 'tx', 'fwd', 'request', 'rewrite', 'tick'):
      with self.subTest(entry=entry):
        self.setUp(); self.start_both()
        self.safety.set_controls_allowed(False)
        if entry == 'rx': self.feed()
        elif entry == 'tx': self.assertFalse(self.tx(1))
        elif entry == 'fwd': self.safety.safety_fwd_hook(2, 0x50E)
        elif entry == 'request': self.assertNotEqual(self.request(74), 0)
        elif entry == 'rewrite': self.safety.test_t9_rvv_rewrite(self.stock_rvv(True), 0)
        else: self.safety.safety_tick_current_safety_config()
        self.assert_axes(False, False)
        self.feed()
        self.assert_axes(False, False)

  def test_malformed_legacy_and_common_fault_wire_stop_both(self):
    for command in (0x1100 | 74, 0x2200, 0x3100 | 74, 0x2000 | 74, 0x2100 | 39, 0x2100 | 141, 0x2100 | 76):
      with self.subTest(command=command):
        self.setUp(); self.start_both()
        self.sequence += 1
        self.assertNotEqual(self.safety.test_t9_rvv_request(command, self.sequence), 0)
        self.assert_axes(False, False)
        self.assertFalse(self.tx(1))
        self.assertFalse(self.forwarded()[0])
        self.feed()
        self.assertNotEqual(self.request(74), 0)

  def test_expired_rvv_lease_is_common_even_for_a_late_valid_release(self):
    for target in (0, 74):
      with self.subTest(target=target):
        self.setUp(); self.start_both()
        for _ in range(4):
          self.feed()
          self.assertTrue(self.tx(1))
        self.request(target)
        self.assert_axes(False, False)
        self.feed()
        self.assertNotEqual(self.request(74), 0)

  def test_duplicate_wire_does_not_refresh_the_command_lease(self):
    self.start_both()
    for _ in range(3):
      self.feed(); self.assertTrue(self.tx(1))
      self.assertEqual(self.safety.test_t9_rvv_request(0x214A, self.sequence), 3)
      self.assertTrue(self.forwarded()[0])
    self.feed(); self.assertTrue(self.tx(1))
    self.assertEqual(self.safety.test_t9_rvv_request(0x214A, self.sequence), 3)
    self.assert_axes(False, False)
    self.assertFalse(self.forwarded()[0])

  def test_lateral_lease_expiry_is_common_even_while_rvv_is_renewed(self):
    self.start_both()
    for _ in range(5):
      self.feed()
      self.assertEqual(self.request(74), 0)
    self.feed()
    self.assertNotEqual(self.request(74), 0)
    self.assert_axes(False, False)
    self.assertFalse(self.forwarded()[0])

  def test_invalid_can_and_wiring_faults_are_common(self):
    for kind in ('checksum', 'wrong_eps_side', 'both_sides'):
      with self.subTest(kind=kind):
        self.setUp(); self.start_both()
        if kind == 'checksum':
          msg = self.packed('T9_STEERING_TORQUE_2F5', {'DriverTorqueRaw': 0})
          msg[0].data[1] ^= 16
          self.assertFalse(self.safety.safety_rx_hook(msg))
        elif kind == 'wrong_eps_side':
          self.safety.safety_fwd_hook(2, 0x495)
        else:
          self.safety.safety_fwd_hook(0, 0x123)
          self.safety.safety_fwd_hook(2, 0x123)
        self.assert_axes(False, False)
        self.assertFalse(self.forwarded()[0])
        self.assertFalse(self.tx(1))

  def test_split_probe_never_admits_emits_or_rewrites(self):
    self.safety.set_safety_hooks(31, 0x1315)
    self.feed(cruise=False)
    for _ in range(45):
      self.feed(cruise=True)
    self.assert_axes(False, False)
    self.assertFalse(self.tx(0, 3, 0))
    self.assertNotEqual(self.request(74), 0)
    self.assertFalse(self.forwarded()[0])

  def test_capability_queries_do_not_change_authority_sequence_or_lease(self):
    self.assertEqual(self.safety.test_t9_rvv_control_request(0, 0), 6)
    self.assertEqual(self.safety.test_t9_rvv_control_request(0, 2), 7)
    self.assert_axes(False, False)
    self.start_both()
    for _ in range(4):
      self.feed(); self.assertTrue(self.tx(1))
      self.assertEqual(self.safety.test_t9_rvv_control_request(0, 2), 7)
      self.assert_axes(True, True)
    self.assertFalse(self.forwarded()[0])  # a query must not renew the lease
    self.assert_axes(False, False)
    self.safety.set_safety_hooks(19, 0)  # safe to query before selecting split
    self.assertEqual(self.safety.test_t9_rvv_control_request(0, 2), 7)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_permission_status_is_read_only_and_legacy_keeps_single_bit(self):
    self.start_both()
    self.assertEqual(self.safety.test_t9_rvv_permission_bits(), 7)
    self.safety.set_controls_allowed(False)
    self.assertEqual(self.safety.test_t9_rvv_permission_bits(), 0)
    self.assertTrue(self.safety.get_t9_split_lateral_allowed())  # query did not mutate
    self.assertTrue(self.safety.get_t9_split_rvv_allowed())
    self.feed()
    self.assert_axes(False, False)
    for param in (0x1308, 0x1310, 0x1312):
      self.safety.set_safety_hooks(31, param)
      self.safety.set_controls_allowed(True)
      self.assertEqual(self.safety.test_t9_rvv_permission_bits(), 1)

  def test_torque_rate_and_speed_limits_remain_bounded(self):
    self.feed(speed=140, setpoint=140)
    self.engage()
    self.assertEqual(self.request(140), 0)
    for torque in range(1, 16):
      self.assertTrue(self.tx(torque))
      self.feed()
      self.assertEqual(self.request(140), 0)
    self.assertFalse(self.tx(16))
    self.assertTrue(self.tx(0, 2, 0))
    self.feed(speed=140.01)
    self.assert_axes(False, False)


if __name__ == '__main__':
  unittest.main()
