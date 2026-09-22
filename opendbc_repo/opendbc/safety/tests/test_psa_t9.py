import unittest

from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py


class TestT9LateralSafety(unittest.TestCase):
  # Same PSA frame identity as the angle profile: the generic cross-brand
  # all-zero-payload test is not a discriminator between these two variants.
  # The full T9 bus/address allowlist is checked explicitly below.
  TX_MSGS = None
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)
    self.assertEqual(self.safety.set_safety_hooks(31, 0x1308), 0)
    self.safety.init_tests()
    self.packer = CANPackerSafety('psa_308_t9_2018')
    self.now = 0
    self.settings = dict(cruise=False, driver=0, pedal=0, brake=False, blinker=0, speed=75, eps=1,
                         gear=4, park=0, belt=2, doors=False, reverse=False, ignition=True, mode=1, setpoint=75)
    self.template = bytes.fromhex('000012000c000000')
    for _ in range(42):
      self.feed()

  def raw(self, address, data, bus=0):
    return libsafety_py.make_CANPacket(address, bus, bytes(data))

  def packed(self, name, values, bus=0):
    return self.packer.make_can_msg_safety(name, bus, values)

  def feed(self, **changes):
    self.settings.update(changes); s = self.settings
    self.now += 50000; self.safety.set_timer(self.now)
    frames = [
      self.raw(0x3F2, self.template, 2), self.raw(0x495, [0, 0, (s['eps'] << 2) | (int(s.get('driver_activity', False)) << 1), 0]),
      self.packed('T9_STEERING_TORQUE_2F5', {'DriverTorqueRaw': s['driver']}),
      self.packed('T9_WHEEL_SPEEDS_30D', {k: s['speed'] for k in ['WheelSpeedFrontLeftKph', 'WheelSpeedFrontRightKph', 'WheelSpeedRearLeftKph', 'WheelSpeedRearRightKph']}),
      self.raw(0x348, [s['gear'] << 4, 0, 0, 0, 0, 0, 64 if s['ignition'] else 0, 0]),
      self.packed('T9_BODY_STATUS_412', {'BrakePedalActive': s['brake'], 'DriverDoorOpen': s['doors'], 'ReverseGearActive': s['reverse']}, 2),
      self.packed('T9_EASY_MOVE_3AD', {'ParkingBrakeState': s['park']}),
      self.packed('T9_RESTRAINTS_572', {'DriverSeatbeltState': s['belt']}, 2),
      self.packed('T9_ACCELERATOR_PEDAL_228', {'AcceleratorPedalPct': s['pedal']}),
      self.packed('T9_DRIVER_CRUISE_COMMAND_452', {'TurnSignalStatus': s['blinker']}, 2),
      self.stock_rvv(s['cruise'], s['mode'], s['setpoint']),
      self.packed('T9_ENGINE_DYNAMICS_208', {'CruiseStateCandidate': 2 if s['cruise'] else 0}),
    ]
    for frame in frames:
      self.assertTrue(self.safety.safety_rx_hook(frame))

  def stock_rvv(self, active, mode=1, setpoint=75):
    setpoint = setpoint if active else 255
    parity = (((setpoint >> 4).bit_count() % 2) << 1) | ((setpoint & 15).bit_count() % 2)
    return self.raw(0x50E, [parity << 4, 0, 0, 0, 0, 0, setpoint, (int(active) << 7) | (mode << 5)], 2)

  def command(self, torque=0, state=4, factor=100, bus=0):
    data = bytearray(self.template); raw = torque & 0x7FF
    data[3] = raw >> 3; data[4] = ((raw & 7) << 5) | (state << 2) | (data[4] & 3)
    data[5] = factor << 1
    return self.raw(0x3F2, data, bus)

  def tx(self, torque=0, state=4, factor=100, bus=0):
    return bool(self.safety.safety_tx_hook(self.command(torque, state, factor, bus)))

  def engage(self):
    self.feed(cruise=True)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.tx(0, 3, 0))
    self.feed()
    self.assertTrue(self.tx(0, 4, 1))
    self.feed(eps=3)
    self.assertTrue(self.tx(0))
    self.feed()

  def test_requires_real_engagement_and_new_eps_ack(self):
    self.assertFalse(self.tx(1))
    self.feed(cruise=True)
    self.assertFalse(self.tx(1))
    self.assertTrue(self.tx(0, 4, 1))
    self.feed()
    self.assertFalse(self.tx(1))
    self.feed(eps=3)
    self.assertTrue(self.tx(1))

  def test_bsi_and_engine_enable_order_does_not_lose_the_manual_request(self):
    for engine_first in (False, True):
      with self.subTest(engine_first=engine_first):
        self.setUp()
        engine = self.packed('T9_ENGINE_DYNAMICS_208', {'CruiseStateCandidate': 2})
        stock = self.stock_rvv(True)
        first, second = (engine, stock) if engine_first else (stock, engine)
        self.assertTrue(self.safety.safety_rx_hook(first))
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertTrue(self.safety.safety_rx_hook(second))
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertTrue(self.tx(0, 3, 0))
        self.assertFalse(self.tx(1))  # still needs the new EPS handshake

  def test_pending_manual_request_waits_briefly_for_eps_but_cannot_retry_later(self):
    for expire in (False, True):
      with self.subTest(expire=expire):
        self.setUp()
        self.feed(cruise=True, eps=0)
        self.assertFalse(self.safety.get_controls_allowed())
        for _ in range(6 if expire else 1):
          self.feed(eps=0)
        self.feed(eps=1)
        self.assertEqual(self.safety.get_controls_allowed(), not expire)
        # An admitted request is consumed. An expired one stays expired.
        self.feed(eps=0)
        self.feed(eps=1)
        self.assertFalse(self.safety.get_controls_allowed())

  def test_accelerator_override_engine_edge_does_not_rearm_panda(self):
    self.engage(); self.assertTrue(self.tx(1))
    self.feed(pedal=1)
    engine_override = self.packed('T9_ENGINE_DYNAMICS_208', {'CruiseStateCandidate': 1})
    self.assertTrue(self.safety.safety_rx_hook(engine_override))
    self.assertTrue(self.tx(0, 2, 0))
    self.feed(pedal=0, eps=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.tx(0, 3, 0))

  def test_invalid_stock_off_parity_cannot_reset_the_episode(self):
    self.engage(); self.assertTrue(self.tx(1))
    self.feed(driver=16)
    self.assertTrue(self.tx(0, 2, 0))
    self.feed(driver=0, eps=1)
    self.safety.safety_rx_hook(self.packed('T9_ENGINE_DYNAMICS_208', {'CruiseStateCandidate': 0}))
    corrupted = self.stock_rvv(False)
    corrupted[0].data[0] ^= 0x10
    self.safety.safety_rx_hook(corrupted)
    self.feed(cruise=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.tx(0, 3, 0))

  def test_bounded_signed_torque_can_continue_after_first_ack_window(self):
    self.engage()
    for torque in range(1, 16):
      self.assertTrue(self.tx(torque)); self.feed()
    for _ in range(20):
      self.assertTrue(self.tx(15)); self.feed()
    self.assertFalse(self.tx(16))
    self.assertTrue(self.tx(0, 2, 0))

  def test_negative_torque_and_slew(self):
    self.engage()
    self.assertFalse(self.tx(-2))
    self.assertTrue(self.tx(-1)); self.feed()
    self.assertFalse(self.tx(-3))

  def test_driver_boundary_in_both_directions_and_no_automatic_resume(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        self.setUp(); self.engage()
        # Hold the boundary beyond the driver-sample window, including
        # opposing maximum torque: admission and TX limits must agree.
        for torque in range(1, 16):
          self.feed(driver=sign * 15)
          self.assertTrue(self.safety.get_controls_allowed())
          self.assertTrue(self.tx(-sign * torque))
        self.feed(driver=sign * 16)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.tx(-sign * 15))
        self.assertTrue(self.tx(0, 2, 0))
        self.feed(driver=0, eps=1)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.tx(0, 3, 0))

  def test_transport_delay_can_compress_nominal_twenty_hz_and_cascade_rejections(self):
    self.engage()
    self.assertTrue(self.tx(1))
    self.feed()
    # Python publishes at +50 ms, but SPI delivers this command 6 ms late.
    self.safety.set_timer(self.now + 6000)
    self.assertTrue(self.tx(2))
    self.feed()
    # The next publication is on time: only 44 ms since actual reception.
    self.assertFalse(self.tx(3))
    self.feed()
    self.assertFalse(self.tx(4))  # host ramp now differs from accepted torque
    # Retain the immediate zero release even following a timing rejection.
    self.assertTrue(self.tx(0, 2, 0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_spacing_after_completed_transport_preserves_compiled_safety_limits(self):
    self.engage()
    self.assertTrue(self.tx(1))
    for torque in range(2, 11):
      self.feed()
      self.safety.set_timer(self.now + 6000)
      self.assertTrue(self.tx(torque))
      self.feed()
      # Sender waits until >=46 ms after the preceding completed write.
      self.safety.set_timer(self.now + 2000)
      self.assertTrue(self.tx(torque))
    self.assertFalse(self.tx(11))  # bounds are unchanged
    self.assertTrue(self.tx(0, 2, 0))

  def test_user_and_vehicle_interruptions_require_cruise_rearm(self):
    cases = [{'brake': True}, {'pedal': 1}, {'driver': 16}, {'driver': -16}, {'speed': 66},
             {'speed': 140.01}, {'gear': 9}, {'park': 1}, {'park': 3}, {'belt': 1}, {'doors': True},
             {'reverse': True}, {'ignition': False}, {'mode': 2}, {'eps': 4}, {'eps': 1}, {'cruise': False}]
    for change in cases:
      with self.subTest(change=change):
        self.setUp(); self.engage(); self.assertTrue(self.tx(1))
        old = self.settings.copy(); self.feed(**change)
        self.assertFalse(self.tx(1)); self.assertTrue(self.tx(0, 2, 0))
        self.feed(**old)
        if change != {'cruise': False}:
          self.assertFalse(self.safety.get_controls_allowed())

  def test_rejects_wrong_transport_and_every_opaque_bit_change(self):
    self.engage()
    for bus in (1, 2, 3): self.assertFalse(self.tx(0, bus=bus))
    for address in (0x50E, 0x228, 0x2F5, 0x7DF, 0x495):
      self.assertFalse(self.safety.safety_tx_hook(self.raw(address, bytes(8))))
    for index, mask in enumerate([255, 255, 255, 0, 3, 1, 255, 255]):
      for bit in range(8):
        if mask & (1 << bit):
          msg = self.command(); msg[0].data[index] ^= 1 << bit
          self.assertFalse(self.safety.safety_tx_hook(msg))
    msg = self.command(); msg[0].extended = 1
    self.assertFalse(self.safety.safety_tx_hook(msg))

  def test_complete_standard_id_allowlist(self):
    self.engage()
    for bus in range(4):
      for address in range(0x800):
        if (address, bus) != (0x3F2, 0):
          self.assertFalse(self.safety.safety_tx_hook(self.raw(address, bytes(8), bus)))

  def test_stale_signals_and_host_gap_cannot_resume_torque(self):
    self.engage(); self.assertTrue(self.tx(1))
    self.now += 300000; self.safety.set_timer(self.now)
    self.assertFalse(self.tx(1))
    self.feed()
    self.assertFalse(self.tx(1))

  def test_corrupt_driver_frame_blocks_torque(self):
    self.engage()
    msg = self.packed('T9_STEERING_TORQUE_2F5', {'DriverTorqueRaw': 0})
    msg[0].data[1] ^= 16
    self.assertFalse(self.safety.safety_rx_hook(msg))
    self.assertFalse(self.tx(1))

  def test_late_eps_ack_is_rejected(self):
    self.feed(cruise=True); self.assertTrue(self.tx(0, 4, 1))
    for _ in range(11): self.feed()
    self.feed(eps=3)
    self.assertFalse(self.tx(1)); self.assertFalse(self.safety.get_controls_allowed())

  def test_duplicate_command_or_wrong_eps_side_latches_fault(self):
    for bus, address in [(0, 0x3F2), (2, 0x495)]:
      self.setUp(); self.engage()
      self.safety.safety_fwd_hook(bus, address)
      self.assertFalse(self.tx(1))
      self.feed(cruise=False); self.feed(cruise=True)
      self.assertFalse(self.tx(1))

  def test_forwarding_and_probe_have_no_longitudinal_output(self):
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x50E), 0)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x495), 2)
    self.safety.set_safety_hooks(31, 0x1309)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    self.assertFalse(self.tx(0))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_factory_stream_survives_settle_and_rejected_host_frames(self):
    self.safety.set_timer(0)
    self.safety.set_safety_hooks(31, 0x1308)
    self.safety.init_tests()
    self.now = 0
    for _ in range(45):
      self.feed()
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
      self.assertFalse(self.tx(1))
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    # Idle commands cannot interrupt the factory stream, even after settling.
    self.assertFalse(self.tx(0, 2, 0))
    self.assertFalse(self.tx(0, 3, 0))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    self.feed(cruise=True)
    self.assertTrue(self.tx(0, 3, 0))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x50E), 0)

  def test_handover_keeps_independent_relay_fault_detection(self):
    self.feed(cruise=True)
    self.assertTrue(self.tx(0, 3, 0))
    self.safety.set_timer(self.now+2_100_000)
    self.safety.safety_rx_hook(self.raw(0x3F2, self.template, 0))
    self.assertTrue(self.safety.get_relay_malfunction())
    self.assertFalse(self.tx(0, 2, 0))

  def test_zero_only_stream_also_expires_and_cannot_revive_the_lease(self):
    self.feed(cruise=True)
    self.assertTrue(self.tx(0, 3, 0))
    self.now += 200000
    self.feed()  # exactly 250 ms, fresh inputs
    self.assertTrue(self.tx(0, 3, 0))
    self.now += 200001
    self.feed()  # 250 ms + 1 us, no state-4 request ever sent
    self.assertFalse(self.tx(0, 2, 0))
    self.feed()
    self.assertFalse(self.tx(0, 2, 0))

  def test_release_restores_factory_and_next_takeover_needs_a_new_cruise_edge(self):
    self.engage()
    self.assertTrue(self.tx(1))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), -1)
    self.assertTrue(self.tx(0, 2, 0))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    for _ in range(30):
      self.feed(eps=1)
      self.assertFalse(self.tx(0, 3, 0))
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    self.feed(cruise=False)
    self.feed(cruise=True)
    self.assertTrue(self.tx(0, 3, 0))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), -1)

  def test_initial_interception_requires_all_vehicle_gates_even_at_zero_torque(self):
    for change in ({'brake': True}, {'speed': 0}, {'speed': 140.01}, {'pedal': 1}, {'driver': 16},
                   {'belt': 1}, {'doors': True}, {'gear': 9}, {'eps': 4}, {'mode': 2}):
      with self.subTest(change=change):
        self.setUp()
        self.feed(cruise=True, **change)
        self.assertFalse(self.tx(0, 3, 0))
        self.assertFalse(self.tx(0, 4, 1))
        self.assertFalse(self.tx(0, 2, 0))
        self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)

  def test_140_kph_boundary_cuts_and_requires_physical_rearm(self):
    self.feed(speed=140)
    self.engage()
    self.assertTrue(self.tx(1))
    self.feed(speed=140.01)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.tx(1))
    self.assertTrue(self.tx(0, 2, 0))
    self.feed(speed=140)
    self.assertFalse(self.tx(0, 3, 0))
    self.feed(cruise=False, eps=1)
    self.engage()
    self.assertTrue(self.tx(1))


if __name__ == '__main__': unittest.main()
