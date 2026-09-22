import os
import json
import unittest
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.car.psa.lateral_test import T9LateralTestController, steering_frame
from opendbc.car.psa.rvv_control import setpoint_parity
from opendbc.safety.tests.libsafety import libsafety_py


class TestT9LateralIntegration(unittest.TestCase):
  def setUp(self):
    log_patch = patch('opendbc.car.psa.lateral_test.carlog.info')
    self.debug_log = log_patch.start()
    self.addCleanup(log_patch.stop)
    with patch.dict(os.environ, {'PSA_T9_LATERAL_TEST': '1'}):
      self.cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.ci = CarInterface(self.cp)
    self.packer = CANPacker('psa_308_t9_2018')
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0); self.safety.set_safety_hooks(31, 0x1308); self.safety.init_tests()
    self.output = []
    self.sent_nanos = []

  def step(self, ms, *, requested=None, lat_active=None, brake=False, pedal=0, driver=0, stock_bus=2, eps_bus=0, eps=None,
           jitter_ns=0, speed=75, driver_activity=False, stock_active=None, engine_state=None, stock_setpoint=75,
           stock_lka=None, feedback_frames=(), blinker=0, pause=False, resume=False, cycle_ready=False):
    now = 1_000_000_000 + ms * 1_000_000 + jitter_ns
    if requested is None: requested = ms >= 2200
    if stock_active is None: stock_active = requested
    if engine_state is None: engine_state = 2 if requested else 0
    if eps is None: eps = 3 if ms >= 2350 else 1
    values = {
      'T9_ENGINE_DYNAMICS_208': {'CruiseStateCandidate': engine_state},
      'T9_ACCELERATOR_PEDAL_228': {'AcceleratorPedalPct': pedal},
      'T9_STEERING_TORQUE_2F5': {'DriverTorqueRaw': driver},
      'T9_STEERING_DYNAMICS_305': {},
      'T9_WHEEL_SPEEDS_30D': {f'WheelSpeed{w}Kph': speed for w in ('FrontLeft', 'FrontRight', 'RearLeft', 'RearRight')},
      'T9_ENGINE_GEAR_348': {'CurrentGear': 4}, 'T9_GEARBOX_TARGET_349': {'TargetGear': 4},
      'T9_EASY_MOVE_3AD': {}, 'T9_BRAKE_DYNAMICS_3CD': {},
      'T9_BODY_STATUS_412': {'BrakePedalActive': brake}, 'T9_DRIVER_CRUISE_COMMAND_452': {'TurnSignalStatus': blinker},
      'T9_CRUISE_SETPOINT_50E': {'CruiseMode': 1, 'CruiseSetpointKph': stock_setpoint if stock_active else 255},
      'T9_RESTRAINTS_572': {'DriverSeatbeltState': 2},
    }
    frames = []
    for name, signals in values.items():
      bus = 2 if name in ('T9_BODY_STATUS_412', 'T9_CRUISE_SETPOINT_50E', 'T9_RESTRAINTS_572', 'T9_DRIVER_CRUISE_COMMAND_452') else 0
      address, data, bus = self.packer.make_can_msg(name, bus, signals)
      if address == 0x348:
        data = bytearray(data); data[6] |= 64; data = bytes(data)
      if address == 0x50E:
        data = bytearray(data)
        data[0] = (data[0] & 0xCF) | (setpoint_parity(data[6]) << 4)
        self.stock_counter = (getattr(self, 'stock_counter', 0) + 1) & 15
        data[7] = (int(stock_active) << 7) | 0x20 | self.stock_counter
        data = bytes(data)
      frames.append((address, data, bus))
    frames += [(0x3F2, stock_lka if stock_lka is not None else bytes.fromhex('000012000c000000'), stock_bus),
               (0x495, bytes([0, 0, (eps << 2) | (int(driver_activity) << 1), 0]), eps_bus)]
    self.safety.set_timer((now-1_000_000_000)//1000)
    for address, data, bus in frames:
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
      self.safety.safety_fwd_hook(bus, address)
    self.ci.update([(now, frames + list(feedback_frames))])
    control = structs.CarControl(enabled=requested, latActive=requested if lat_active is None else lat_active, longActive=True)
    control.psaLateralPause = pause
    control.psaLateralResume = resume
    control.psaEpsCycleReady = cycle_ready
    control.actuators.torque = 1.
    control.actuators.accel = 1.
    applied, sends = self.ci.apply(control.as_reader(), now)
    self.assertEqual(applied.accel, 0.)
    for address, data, bus in sends:
      self.assertEqual((address, bus, len(data)), (0x3F2, 0, 8))
      accepted = self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
      self.assertTrue(accepted, (ms, data.hex(), self.ci.CC.t9_lateral.status))
      self.output.append((ms, applied.torqueOutputCan))
      self.sent_nanos.append(now)
    return applied, sends

  def active(self):
    for ms in range(0, 4001, 10): self.step(ms)
    self.assertFalse(self.cp.dashcamOnly)
    self.assertFalse(self.cp.openpilotLongitudinalControl)
    self.assertEqual(self.cp.safetyConfigs[0].safetyParam, 0x1308)
    self.assertEqual(self.output[-1][1], 15)

  def test_native_controller_frames_pass_compiled_panda_safety(self):
    self.active()
    self.assertTrue(all(abs(value) <= 15 for _, value in self.output))
    self.assertTrue(all(value == 0 for ms, value in self.output if ms < 2350))

  def test_panda_rejection_releases_with_fresh_template_and_requires_manual_restart(self):
    for combined in (False, True):
      with self.subTest(combined=combined):
        self.setUp()
        with patch.dict(os.environ, {'PSA_T9_LATERAL_TEST': '1', 'PSA_T9_RVV_TEST': str(int(combined))}):
          self.cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
        self.ci = CarInterface(self.cp)
        self.safety.set_timer(0)
        self.safety.set_safety_hooks(31, 0x1312 if combined else 0x1308)
        self.safety.init_tests()
        for ms in range(0, 4001, 10):
          self.step(ms)
        controller = self.ci.CC.t9_lateral
        old_command = controller.queued_commands[-1][1]
        stock_rvv = bytes([setpoint_parity(75) << 4, 0, 0, 0, 0, 0, 75, 0xA0])
        if combined:
          self.assertEqual(self.safety.test_t9_rvv_request(0x1100 | 74, 1), 0)
          self.assertTrue(self.safety.test_t9_rvv_rewrite(libsafety_py.make_CANPacket(0x50E, 2, stock_rvv), 0))
        changed_template = bytes.fromhex('000012000e000000')
        # As in route 28, the BSI changes a preserved field before MCU TX.
        self.safety.set_timer(4_050_000)
        self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x3F2, 2, changed_template)))
        self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x3F2, 0, old_command)))
        self.assertTrue(self.safety.get_controls_allowed())
        applied, sends = self.step(4051, stock_lka=changed_template,
                                   feedback_frames=[(0x3F2, old_command, 192)])
        self.assertEqual(controller.status['reason'], 'panda_steering_tx_rejected')
        self.assertEqual(applied.torqueOutputCan, 0)
        self.assertEqual(len(sends), 1)  # step also checks compiled Panda acceptance
        self.assertEqual((sends[0][1][4] >> 2) & 7, 2)
        self.assertEqual(sends[0][1][4] & 3, 2)  # current opaque BSI fields retained
        self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
        self.assertFalse(self.safety.get_controls_allowed())
        if combined:
          restored = libsafety_py.make_CANPacket(0x50E, 2, stock_rvv)
          self.assertFalse(self.safety.test_t9_rvv_rewrite(restored, 0))
          self.assertEqual(restored[0].data[6], 75)
          self.assertNotEqual(self.safety.test_t9_rvv_request(0x1100 | 74, 2), 0)
        self.assertTrue(self.ci.CS.out.steerFaultTemporary)
        self.assertTrue(self.ci.CS.out.blockPcmEnable)
        for ms in range(4060, 4401, 10):
          applied, sends = self.step(ms, eps=1, stock_lka=changed_template)
          self.assertEqual((applied.torqueOutputCan, sends), (0, []))
        self.step(4410, requested=False, eps=1)
        self.assertEqual(controller.previous_stop_reason, 'panda_steering_tx_rejected')
        for ms in range(4420, 6101, 10):
          self.step(ms, eps=3 if ms >= 4570 else 1)
        self.assertEqual(controller.status['phase'], 'active')
        self.assertTrue(self.safety.get_controls_allowed())

  def test_only_recent_own_rejection_ends_current_episode(self):
    self.active()
    controller = self.ci.CC.t9_lateral
    sent, command = controller.queued_commands[-1]
    other = bytearray(command)
    other[0] ^= 1
    for timestamp, address, data, bus in (
        (sent + 1, 0x3F2, command, 128),  # accepted TX echo
        (sent + 1, 0x3F2, command, 194),  # rejected on another destination
        (sent + 1, 0x50E, command, 192),
        (sent + 1, 0x3F2, bytes(other), 192),
        (sent + 1, 0x3F2, command[:7], 192),
        (sent - 1_000_000_000, 0x3F2, command, 192),
        (sent + 150_000_001, 0x3F2, command, 192)):
      controller.observe([(timestamp, [(address, data, bus)])])
      self.assertEqual(controller.lateral.phase, 'active')
    controller.observe([(sent + 1, [(0x3F2, command, 192)])])
    self.assertEqual(controller.lateral.reason, 'panda_steering_tx_rejected')

  def test_rejection_releases_between_ticks_even_during_zero_torque_preparation(self):
    for stop_ms in (2200, 4000):
      with self.subTest(stop_ms=stop_ms):
        self.setUp()
        for ms in range(0, stop_ms + 1, 10):
          self.step(ms)
        controller = self.ci.CC.t9_lateral
        sent, command = controller.queued_commands[-1]
        self.assertLess((1_000_000_000 + (stop_ms + 1) * 1_000_000) - sent, 45_000_000)
        applied, sends = self.step(stop_ms + 1, feedback_frames=[(0x3F2, command, 192)])
        self.assertEqual(applied.torqueOutputCan, 0)
        self.assertEqual(len(sends), 1)
        self.assertEqual((sends[0][1][4] >> 2) & 7, 2)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertEqual(controller.status['reason'], 'panda_steering_tx_rejected')

  def test_late_rejection_does_not_replace_first_stop_reason_or_new_episode(self):
    self.active()
    controller = self.ci.CC.t9_lateral
    old_command = controller.queued_commands[-1][1]
    self.step(4001, driver=16)
    controller.observe([(5_002_000_000, [(0x3F2, old_command, 192)])])
    self.assertEqual(controller.lateral.reason, 'driver_override')
    self.step(4010, requested=False, eps=1)
    self.assertFalse(controller.queued_commands)
    controller.observe([(5_011_000_000, [(0x3F2, old_command, 192)])])
    self.assertEqual(controller.lateral.phase, 'disabled')

  def test_host_and_panda_agree_at_140_kph(self):
    for ms in range(0, 4001, 10):
      self.step(ms, speed=140)
    self.assertEqual(self.output[-1][1], 15)
    self.assertTrue(self.safety.get_controls_allowed())
    applied, _ = self.step(4001, speed=140.01)
    self.assertEqual(applied.torqueOutputCan, 0)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.ci.CC.t9_lateral.status['reason'], 'speed_outside_candidate_envelope')
    for ms in range(4010, 4301, 10):
      applied, sends = self.step(ms, speed=140)
      self.assertEqual(applied.torqueOutputCan, 0)
      self.assertEqual(sends, [])

  def test_brake_driver_pedal_release_immediately_and_cannot_auto_resume(self):
    for change in ({'brake': True}, {'driver': 16}, {'pedal': 1}):
      self.setUp(); self.active()
      applied, sends = self.step(4001, **change)
      self.assertEqual(applied.torqueOutputCan, 0)
      self.assertEqual(len(sends), 1)
      for ms in range(4050, 4401, 50):
        applied, _ = self.step(ms)
        self.assertEqual(applied.torqueOutputCan, 0)

  def test_unseparated_bus_does_not_queue_commands(self):
    for ms in range(0, 4001, 10):
      _, sends = self.step(ms, stock_bus=0, eps_bus=0)
      self.assertEqual(sends, [])

  def test_driver_override_is_immediate_disengagement_without_false_eps_fault(self):
    self.active()
    self.step(4001, driver=16)
    state = self.ci.CS.out
    self.assertTrue(state.steeringDisengage)
    self.assertFalse(state.blockPcmEnable)
    self.assertFalse(state.steerFaultTemporary)
    self.assertEqual(self.ci.CC.t9_lateral.status['reason'], 'driver_override')
    for ms in range(4050, 4201, 50):
      self.step(ms, driver=0, eps=1)
      self.assertTrue(self.ci.CS.out.steeringDisengage)
      self.assertFalse(self.ci.CS.out.blockPcmEnable)
      self.assertFalse(self.ci.CS.out.steerFaultTemporary)
    self.step(4250, requested=False, driver=0, eps=1)
    self.step(4300, requested=False, driver=0, eps=1)
    self.assertFalse(self.ci.CS.out.steeringDisengage)
    self.assertFalse(self.ci.CS.out.blockPcmEnable)

  def test_actual_eps_fault_is_retained_during_driver_intervention(self):
    self.active()
    self.step(4001, driver=16, eps=4)
    self.assertTrue(self.ci.CS.out.steeringDisengage)
    self.assertTrue(self.ci.CS.out.steerFaultTemporary)

  def test_manual_rvv_restart_after_each_interruption_and_eps_recovery(self):
    for change in ({'driver': 16}, {'brake': True}, {'pedal': 1}, {'eps': 0}, {'eps': 4}, {'eps': 7}):
      with self.subTest(change=change):
        self.setUp(); self.active()
        self.step(4001, **change)
        previous_reason = self.ci.CC.t9_lateral.status['reason']
        # EPS remains unavailable for the entire physical RVV-off interval.
        for ms in range(4010, 4110, 10):
          applied, _ = self.step(ms, requested=False, eps=0)
          self.assertEqual(applied.torqueOutputCan, 0)
          self.assertTrue(self.ci.CS.out.steerFaultTemporary)  # truthful current fault
          self.assertFalse(self.ci.CS.out.blockPcmEnable)  # old episode cleared
        self.assertEqual(self.ci.CC.t9_lateral.status['previous_stop_reason'], previous_reason)
        self.assertEqual(self.ci.CC.t9_lateral.status['physical_rearm_count'], 1)
        self.step(4110, requested=True, eps=1)
        self.assertFalse(self.ci.CS.out.steerFaultTemporary)
        self.assertFalse(self.ci.CS.out.steeringDisengage)
        self.assertFalse(self.ci.CS.out.blockPcmEnable)
        self.assertEqual(self.ci.CC.t9_lateral.status['phase'], 'preparing')
        for ms in range(4120, 6001, 10):
          self.step(ms, eps=3 if ms >= 4260 else 1)
        self.assertEqual(self.ci.CC.t9_lateral.status['phase'], 'active')
        self.assertEqual(self.output[-1][1], 15)
        self.assertTrue(self.safety.get_controls_allowed())

  def test_manual_restart_cannot_override_a_current_eps_fault(self):
    self.active(); self.step(4001, eps=4)
    for ms in range(4010, 4110, 10):
      self.step(ms, requested=False, eps=4)
    applied, _ = self.step(4110, requested=True, eps=4)
    self.assertEqual(applied.torqueOutputCan, 0)
    self.assertTrue(self.ci.CS.out.steerFaultTemporary)
    self.assertFalse(self.safety.get_controls_allowed())
    for ms in range(4120, 4301, 10):
      applied, sends = self.step(ms, eps=1)
      self.assertEqual(applied.torqueOutputCan, 0)
      self.assertEqual(sends, [])

  def test_split_host_updates_wait_for_both_physical_cruise_sources(self):
    for bsi_first in (False, True):
      with self.subTest(bsi_first=bsi_first):
        self.setUp()
        for ms in range(0, 2200, 10):
          self.step(ms, requested=False, eps=1)
        applied, sends = self.step(2200, requested=True, stock_active=bsi_first,
                                   engine_state=0 if bsi_first else 2, eps=1)
        self.assertFalse(self.ci.CS.out.cruiseState.enabled)
        self.assertEqual(sends, [])
        self.assertEqual(applied.torqueOutputCan, 0)
        self.assertFalse(self.safety.get_controls_allowed())
        self.step(2210, requested=True, stock_active=True, engine_state=2, eps=1)
        self.assertTrue(self.ci.CS.out.cruiseState.enabled)
        self.assertEqual(self.ci.CC.t9_lateral.status['phase'], 'preparing')
        self.assertTrue(self.safety.get_controls_allowed())

  def test_accelerator_recovery_is_not_a_manual_cruise_restart(self):
    self.active(); self.step(4001, pedal=1)
    for ms in range(4010, 4110, 10):
      self.step(ms, requested=False, stock_active=True, engine_state=1, pedal=1, eps=1)
    self.assertEqual(self.ci.CC.t9_lateral.status['physical_rearm_count'], 0)
    for ms in range(4110, 4301, 10):
      applied, sends = self.step(ms, eps=1)
      self.assertEqual(applied.torqueOutputCan, 0)
      self.assertEqual(sends, [])

  def test_eps_withdrawal_releases_once_and_never_rearms_on_recovery(self):
    for eps, reason in ((0, 'eps_not_authorized'), (4, 'eps_fault'), (7, 'eps_feedback_invalid')):
      with self.subTest(eps=eps):
        self.setUp(); self.active()
        applied, sends = self.step(4001, eps=eps)
        self.assertTrue(self.ci.CS.out.steerFaultTemporary)
        self.assertFalse(self.ci.CS.out.steerFaultPermanent)
        self.assertEqual(applied.torqueOutputCan, 0)
        self.assertEqual(len(sends), 1)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertEqual(self.ci.CC.t9_lateral.status['reason'], reason)
        for ms in range(4010, 4401, 10):
          applied, sends = self.step(ms, eps=1)
          self.assertEqual(sends, [])
          self.assertEqual(applied.torqueOutputCan, 0)
          self.assertTrue(self.ci.CS.out.steerFaultTemporary)
          self.assertTrue(self.ci.CS.out.blockPcmEnable)
          self.assertFalse(self.safety.get_controls_allowed())

  def test_admission_uses_same_raw_driver_threshold_as_panda(self):
    for raw in (-16, -15, -14, -11, -10, -5, 0, 5, 10, 11, 14, 15, 16):
      self.setUp()
      self.step(0, requested=False, driver=raw, eps=1)
      self.assertEqual(self.ci.CS.out.steeringDisengage, abs(raw) > 15)
      self.assertFalse(self.ci.CS.out.blockPcmEnable)

  def test_soft_disable_cannot_clear_latched_interruption(self):
    self.active()
    self.step(4001, driver=16)
    for ms in range(4050, 4401, 50):
      applied, _ = self.step(ms, lat_active=False, eps=1)
      self.assertEqual(applied.torqueOutputCan, 0)
    for ms in range(4450, 4801, 50):
      applied, _ = self.step(ms, lat_active=True, eps=1)
      self.assertEqual(applied.torqueOutputCan, 0)
    self.assertEqual(self.ci.CC.t9_lateral.lateral.phase, 'blocked')

  def test_merge_removes_mirror_echo_without_discarding_new_payload(self):
    controller = T9LateralTestController()
    frames = [(0x2F5, b'1234567', 0), (0x2F5, b'1234567', 2)]
    self.assertEqual(controller.merge_for_carstate([(100, frames)]), [(100, frames[:1])])
    changed = (0x2F5, b'abcdefg', 2)
    self.assertEqual(controller.merge_for_carstate([(200, [changed])]), [(200, [(0x2F5, b'abcdefg', 0)])])

  def test_packet_conserves_opaque_bits_and_signed_command(self):
    template = bytes.fromhex('125614000d000001')
    for value in range(-15, 16):
      _, data, _ = steering_frame(template, 4, 100, value)
      raw = (data[3] << 3) | (data[4] >> 5)
      self.assertEqual(raw - 2048 if raw & 1024 else raw, value)
      self.assertEqual(data[:3], template[:3]); self.assertEqual(data[4] & 3, template[4] & 3)
      self.assertEqual(data[6:], template[6:])

  def test_debug_records_transitions_and_immediate_cut_without_100hz_spam(self):
    self.active()
    def lateral_entries():
      return [json.loads(call.args[0].removeprefix('psa_t9_lateral ')) for call in self.debug_log.call_args_list
              if call.args[0].startswith('psa_t9_lateral ')]
    entries = lateral_entries()
    phases = {entry['phase'] for entry in entries}
    self.assertTrue({'disabled', 'preparing', 'waiting_eps', 'active'} <= phases)
    self.assertLess(len(entries), 20)
    count = len(entries)
    self.step(4001, brake=True)
    self.assertEqual(len(lateral_entries()), count+1)
    cut = lateral_entries()[-1]
    self.assertEqual(cut['reason'], 'brake_pressed')
    self.assertEqual(cut['torque_raw_queued'], 0)
    self.assertTrue(cut['rearm_required'])
    self.assertTrue(cut['brake_pressed'])
    self.assertEqual(cut['mono_ns'], 5_001_000_000)

  def test_logging_exception_cannot_prevent_steering_release(self):
    self.debug_log.side_effect = OSError('simulated logger failure')
    self.active()
    applied, sends = self.step(4001, brake=True)
    self.assertEqual(applied.torqueOutputCan, 0)
    self.assertEqual(len(sends), 1)

  def test_eps_driver_activity_is_logged_without_changing_control(self):
    self.active()
    applied, _ = self.step(4010, driver_activity=True)
    self.assertEqual(applied.torqueOutputCan, 15)
    self.assertTrue(self.safety.get_controls_allowed())
    snapshots = [json.loads(call.args[0].removeprefix('psa_t9_lateral ')) for call in self.debug_log.call_args_list
                 if call.args[0].startswith('psa_t9_lateral ')]
    self.assertTrue(snapshots[-1]['eps_driver_activity_candidate'])
    self.assertEqual(snapshots[-1]['eps_feedback_hex'], '00000e00')
    self.step(4020, driver_activity=False)
    snapshots = [json.loads(call.args[0].removeprefix('psa_t9_lateral ')) for call in self.debug_log.call_args_list
                 if call.args[0].startswith('psa_t9_lateral ')]
    self.assertFalse(snapshots[-1]['eps_driver_activity_candidate'])

  def test_eps_activity_diagnostic_uses_only_physical_destination_rx(self):
    observer = T9LateralTestController()
    now = 1_000_000_000
    observer.observe([(now, [(0x495, bytes.fromhex('00000c00'), 0)])])
    self.assertFalse(observer.driver_activity(now))
    for bus in (2, 128, 130, 192):
      observer.observe([(now+1, [(0x495, bytes.fromhex('00000e00'), bus)])])
      self.assertFalse(observer.driver_activity(now+1))
    observer.observe([(now+2, [(0x495, bytes.fromhex('00000e00'), 0)])])
    self.assertTrue(observer.driver_activity(now+2))

  def test_eps_activity_diagnostic_never_reports_stale_or_invalid_as_present(self):
    observer = T9LateralTestController()
    now = 1_000_000_000
    valid = bytes.fromhex('00000e00')
    observer.observe([(now, [(0x495, valid, 0)])])
    self.assertIsNone(observer.driver_activity(now-1))
    self.assertIsNone(observer.driver_activity(now+250_000_001))
    observer.observe([(now-1, [(0x495, valid, 0)])])
    self.assertIsNone(observer.driver_activity(now))
    observer.observe([(now+1, [(0x495, valid, 0)])])
    observer.observe([(now+2, [(0x495, valid[:3], 0)])])
    self.assertIsNone(observer.driver_activity(now+2))
    self.assertIsNone(observer.eps_feedback_hex)

  def test_disabled_for_a_minute_preserves_factory_without_idle_transmission(self):
    for ms in range(0, 60001, 10):
      _, sends = self.step(ms, requested=False, eps=0)
      self.assertEqual(sends, [])
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    self.assertFalse(self.ci.CC.t9_lateral.status['replacement_queued'])

  def test_jittered_control_loop_keeps_twenty_hz_and_compiled_safety_limits(self):
    for ms in range(0, 16001, 10):
      # Repeatable sub-millisecond scheduler jitter, as seen in the rlog.
      self.step(ms, jitter_ns=((ms//10*73)%200)*1000)
    times=[t for t in self.sent_nanos if t >= 5_000_000_000]
    gaps=[b-a for a,b in zip(times,times[1:])]
    self.assertGreaterEqual(min(gaps), 45_000_000)
    self.assertLess(max(gaps), 61_000_000)
    hz=(len(times)-1)*1e9/(times[-1]-times[0])
    self.assertGreater(hz, 19.9)
    self.assertLess(hz, 20.1)

  def test_cut_restores_factory_and_stops_host_stream_without_auto_resume(self):
    self.active()
    _, sends = self.step(4001, brake=True)
    self.assertEqual(len(sends), 1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)
    for ms in range(4050, 7001, 50):
      _, sends = self.step(ms, eps=1)
      self.assertEqual(sends, [])
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x3F2), 0)


if __name__ == '__main__': unittest.main()
