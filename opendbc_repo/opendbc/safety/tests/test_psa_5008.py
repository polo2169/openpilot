import unittest

from opendbc.car.structs import CarParams
from opendbc.car.psa.psacan import create_p87_lka
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py


class TestP87Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0x5008), 0)
    self.safety.init_tests()
    self.safety.set_timer(100_000)
    self.packer = CANPackerSafety('psa_p87_generated')

  def feed(self, *, pedal=True, **changes):
    values = dict(gas=0, engine_demand=0, brake=0, t15=True, engine=3, speed=60, driver=0, eps=3, acc=4, fault=0, request=1)
    values.update(changes)
    for _ in range(8):
      frames = [
        self.packer.make_can_msg_safety('Dyn_CMM', 0, {'P002_Com_rAPP': values['engine_demand']}),
        make_msg(0, 0x348, dat=bytes([0, 0, 0, 0, 0, values['engine'], 0x40 if values['t15'] else 0, 0])),
        self.packer.make_can_msg_safety('Dyn4_FRE', 0, {name: values['speed'] for name in (
          'P263_VehV_VPsvValWhlFrtL', 'P264_VehV_VPsvValWhlFrtR', 'P265_VehV_VPsvValWhlBckL', 'P266_VehV_VPsvValWhlBckR')}),
        self.packer.make_can_msg_safety('STEERING', 0, {'DRIVER_TORQUE': values['driver']}),
        self.packer.make_can_msg_safety('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': values['eps']}),
        self.packer.make_can_msg_safety('Dat_BSI', 2, {'P013_MainBrake': values['brake']}),
        self.packer.make_can_msg_safety('HS2_DAT_MDD_CMD_452', 1, {'RVV_ACC_ACTIVATION_REQ': values['request']}),
        self.packer.make_can_msg_safety('HS2_DYN1_MDD_ETAT_2B6', 1, {'ACC_STATUS': values['acc']}),
        self.packer.make_can_msg_safety('HS2_DYN_UCF_MDD_32D', 1, {'DEFAUT_ACC_FREIN': values['fault']}),
      ]
      if pedal:
        frames.insert(1, self.packer.make_can_msg_safety('Dyn5_CMM', 0, {'P334_ACCPed_Position': values['gas']}))
      for frame in frames:
        self.safety.safety_rx_hook(frame)
    if pedal:
      self.assertTrue(self.safety.safety_config_valid())

  def command(self, torque=8, factor=100, status=4):
    msg = create_p87_lka(bytes.fromhex('00000d0008000000'), torque, factor, status)
    return make_msg(msg.src, msg.address, dat=msg.dat)

  def test_valid_first_torque_step(self):
    self.feed()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.safety_tx_hook(self.command()))

  def test_request_before_active_status_does_not_lose_engagement(self):
    self.feed(acc=3)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(acc=4)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.safety_tx_hook(self.command()))

  def test_active_before_request_and_initial_feedback_arrival(self):
    self.feed(request=0)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(request=1)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.safety_tx_hook(self.command()))

  def test_stock_acc_suspend_and_resume_follow_actual_status(self):
    self.feed()
    for acc in (3, 5, 7, 2, 15):
      with self.subTest(acc=acc):
        self.feed(acc=acc)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.safety.safety_tx_hook(self.command()))
        self.feed(acc=4)
        self.assertTrue(self.safety.get_controls_allowed())

  def test_fault_recovery_alone_does_not_rearm_active_cruise(self):
    self.feed()
    self.feed(fault=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(fault=0)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(request=0)
    self.feed(request=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_throttle_held_on_acc_resume_needs_another_cruise_edge(self):
    self.feed(acc=5, gas=10)
    self.feed(acc=4, gas=10)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(acc=4, gas=0)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(request=0)
    self.feed(request=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_low_speed_acc_authority_never_authorizes_lateral_torque(self):
    self.feed(speed=30, eps=0)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.command()))
    self.assertFalse(self.safety.safety_tx_hook(self.command(0, 100, 3)))
    self.assertTrue(self.safety.safety_tx_hook(self.command(0, 0, 2)))
    self.feed(speed=60, eps=2)
    self.assertTrue(self.safety.safety_tx_hook(self.command(0, 100, 3)))
    self.feed(eps=3)
    self.assertTrue(self.safety.safety_tx_hook(self.command()))

  def test_engagement_persists_over_the_two_second_mismatch_window(self):
    self.feed()
    for frame in range(1, 301):
      self.safety.set_timer(100_000 + frame * 10_000)
      self.feed()
      self.assertTrue(self.safety.get_controls_allowed())

  def test_brake_recovery_and_bad_checksum_do_not_rearm(self):
    self.feed()
    self.feed(brake=1)
    self.feed(brake=0)
    self.assertFalse(self.safety.get_controls_allowed())
    self.feed(request=0)
    self.feed(request=1)
    self.assertTrue(self.safety.get_controls_allowed())
    msg = self.packer.make_can_msg_safety('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0})
    msg[0].data[3] ^= 1
    self.assertFalse(self.safety.safety_rx_hook(msg))
    self.feed()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stock_acc_engine_demand_is_not_a_driver_override(self):
    self.feed(engine_demand=19.5)
    # Actual released-pedal frame captured while stock ACC demanded 19.5%.
    self.assertTrue(self.safety.safety_rx_hook(make_msg(0, 0x228, dat=bytes.fromhex('4936004100c52032'))))
    self.assertTrue(self.safety.safety_rx_hook(self.packer.make_can_msg_safety('Dyn_CMM', 0, {'P002_Com_rAPP': 19.5})))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.safety_tx_hook(self.command()))

  def test_driver_press_cuts_authority_and_release_does_not_reengage(self):
    self.feed(engine_demand=0)
    pedal = self.packer.make_can_msg_safety('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0.5})
    self.assertTrue(self.safety.safety_rx_hook(pedal))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.command()))
    self.assertTrue(self.safety.safety_tx_hook(self.command(0, 0, 2)))
    self.assertTrue(self.safety.safety_rx_hook(self.packer.make_can_msg_safety('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0})))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_bad_pedal_checksum_or_frozen_counter_blocks_torque(self):
    self.feed()
    pedal = self.packer.make_can_msg_safety('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0})
    pedal[0].data[3] ^= 1
    self.assertFalse(self.safety.safety_rx_hook(pedal))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.command()))
    self.setUp()
    self.feed()
    pedal = self.packer.make_can_msg_safety('Dyn5_CMM', 0, {'P334_ACCPed_Position': 0})
    for _ in range(6):
      self.safety.safety_rx_hook(pedal)
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_missing_wrong_bus_or_wrong_length_pedal_cannot_grant_authority(self):
    for bus, data in ((0, None), (2, bytes.fromhex('4936004100c52032')), (0, bytes.fromhex('4936004100'))):
      with self.subTest(bus=bus, data=data):
        self.setUp()
        self.feed(pedal=False)
        if data is not None:
          self.safety.safety_rx_hook(make_msg(bus, 0x228, dat=data))
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_only_pedal_expiring_blocks_torque(self):
    self.feed()
    self.safety.set_timer(350_001)
    self.feed(pedal=False)  # All other prerequisites are fresh.
    self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_reserved_pedal_values_inhibit_torque(self):
    self.feed(gas=127.5)
    self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_neutral_only_without_authority(self):
    self.assertTrue(self.safety.safety_tx_hook(self.command(0, 0, 2)))
    self.assertFalse(self.safety.safety_tx_hook(self.command(0, 100, 3)))
    self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_fault_pedals_speed_ignition_driver_and_eps_gates(self):
    for change in ({'gas': 1}, {'brake': 1}, {'t15': False}, {'engine': 1}, {'speed': 29.99}, {'speed': 141},
                   {'driver': 16}, {'driver': -16}, {'eps': 4}, {'acc': 15}, {'fault': 1}):
      with self.subTest(change=change):
        self.setUp()
        self.feed(**change)
        self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_exact_thirty_kph_boundary_and_real_eps_permission(self):
    for speed in (29.99,30.,40.,51.99,52.,140.,140.01):
      for eps in (0,1,2,3,4,7):
        with self.subTest(speed=speed,eps=eps):
          self.setUp();self.feed(speed=speed,eps=eps)
          self.assertEqual(bool(self.safety.safety_tx_hook(self.command(0,100,3))),
            30. <= speed <= 140. and eps in (1,2,3))
          self.assertEqual(bool(self.safety.safety_tx_hook(self.command())),
            30. <= speed <= 140. and eps==3)

  def test_torque_waits_for_eps_ack(self):
    self.feed(eps=2)
    self.assertTrue(self.safety.safety_tx_hook(self.command(0, 100, 3)))
    self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_lost_or_invalid_feedback_blocks_torque(self):
    self.feed()
    self.safety.set_timer(350_001)
    self.assertFalse(self.safety.safety_tx_hook(self.command()))
    self.setUp()
    self.feed()
    frame = self.packer.make_can_msg_safety('STEERING', 0, {'DRIVER_TORQUE': 0})
    frame[0].data[0] ^= 0x10
    self.assertFalse(self.safety.safety_rx_hook(frame))
    self.assertFalse(self.safety.safety_tx_hook(self.command()))

  def test_rate_limit_is_enforced(self):
    self.feed()
    self.assertFalse(self.safety.safety_tx_hook(self.command(9)))

  def test_absolute_limit_is_enforced(self):
    self.feed()
    msg = self.command(150)
    msg[0].data[3] = 151 >> 3
    msg[0].data[4] = (msg[0].data[4] & 0x1F) | ((151 & 7) << 5)
    self.safety.set_desired_torque_last(150)
    self.safety.set_rt_torque_last(150)
    self.assertFalse(self.safety.safety_tx_hook(msg))

  def test_realtime_limit_is_enforced(self):
    self.feed()
    self.safety.set_desired_torque_last(75)
    self.safety.set_rt_torque_last(0)
    self.assertFalse(self.safety.safety_tx_hook(self.command(76)))

  def test_no_radar_acc_or_spoof_messages(self):
    self.feed()
    for bus, addr, length in ((1, 0x6B6, 8), (1, 0x452, 6), (1, 0x2B6, 8),
                              (1, 0x2F6, 8), (1, 0x4F6, 5), (1, 0x796, 8), (0, 0x2F5, 7), (2, 0x495, 4)):
      self.assertFalse(self.safety.safety_tx_hook(make_msg(bus, addr, length=length)))

  def test_wrong_bus_length_angle_and_factor(self):
    self.feed()
    msg = self.command()
    msg[0].bus = 2
    self.assertFalse(self.safety.safety_tx_hook(msg))
    msg = self.command()
    msg[0].data_len_code = 7
    self.assertFalse(self.safety.safety_tx_hook(msg))
    for byte, value in ((5, 202), (6, 1)):
      msg = self.command()
      msg[0].data[byte] = value
      self.assertFalse(self.safety.safety_tx_hook(msg))

  def test_stock_forwarding_and_relay_failure(self):
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), -1)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x208), 2)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x412), 0)
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self.safety.safety_tx_hook(self.command(0, 0, 2)))


if __name__ == '__main__':
  unittest.main()
