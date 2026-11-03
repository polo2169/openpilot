import unittest

from opendbc.car.psa.p87_admission import P87Admission, lanes_ready
from opendbc.car.psa.tests.test_5008 import ModelSource
from opendbc.car.psa.tests import test_5008 as host_fixture
from opendbc.car import structs
from opendbc.safety.tests import test_psa_5008 as native_fixture


class TestAdmission(unittest.TestCase):
  def setUp(self):
    self.admission = P87Admission()

  def step(self, ms, **changes):
    now = 1_000_000_000+ms*1_000_000
    inputs = dict(requested=True, reason=None, eps=2, eps_ns=now,
                  model_valid=True, model_ns=now, tick=True)
    inputs.update(changes)
    return self.admission.update(now, **inputs)

  def wait_for_ack(self):
    outputs = [self.step(ms) for ms in range(0, 401, 50)]
    self.assertTrue(all(not o[3] for o in outputs))
    self.assertEqual(outputs[-1][:2], ('waiting_for_eps_ack', 4))
    return outputs

  def activate(self):
    self.wait_for_ack()
    self.assertTrue(self.step(450, eps=3)[3])

  def test_three_models_and_stability_before_zero_torque_activation_request(self):
    outputs = self.wait_for_ack()
    self.assertEqual([o[1] for o in outputs], [2]*6+[3, 3, 4])
    self.assertTrue(self.step(450, eps=3)[3])

  def test_one_held_model_cannot_satisfy_three_distinct_samples(self):
    for ms in range(0, 501, 50):
      self.assertEqual(self.step(ms, model_ns=1_000_000_000)[1], 2)
    self.assertEqual(self.admission.models, 1)

  def test_two_models_cannot_satisfy_three(self):
    for ms in range(0, 451, 50):
      self.assertEqual(self.step(ms, model_ns=1_000_000_000+(50_000_000 if ms else 0))[1], 2)
    self.assertEqual(self.admission.models, 2)

  def test_future_stale_invalid_and_reversed_models_restart_stability(self):
    for changes in ({'model_ns': 0}, {'model_ns': 2_000_000_000}, {'model_valid': False}):
      self.admission = P87Admission()
      self.step(0)
      self.assertEqual(self.step(50, **changes)[:3], ('waiting_valid_lanes', 2, 0))
    self.admission = P87Admission()
    self.step(0)
    self.step(50)
    self.assertEqual(self.step(100, model_ns=1_000_000_000)[0], 'waiting_valid_lanes')

  def test_preexisting_eps_active_needs_release_and_new_ack(self):
    for ms in range(0, 451, 50):
      self.assertFalse(self.step(ms, eps=3)[3])
    self.assertEqual(self.step(500, eps=3)[0], 'waiting_eps_release')
    self.assertEqual(self.step(550)[0], 'preparing_eps')
    self.step(600)
    self.assertEqual(self.step(650)[1], 4)
    self.assertFalse(self.step(700, eps=3, eps_ns=1_650_000_000)[3])
    self.assertTrue(self.step(750, eps=3)[3])

  def test_no_ack_times_out_and_requires_request_off(self):
    self.wait_for_ack()
    for ms in range(450, 1401, 50):
      result = self.step(ms)
    self.assertEqual(result[:3], ('eps_ack_timeout', 2, 0))
    self.assertFalse(self.step(1450, eps=3)[3])
    self.step(1500, requested=False)
    self.assertEqual(self.step(1550)[0], 'validating_lanes')

  def test_driver_blinker_and_low_speed_resume_require_validation_again(self):
    for reason in ('driver_torque_override', 'driver_indicator', 'below_lateral_speed'):
      self.admission = P87Admission()
      self.activate()
      self.assertEqual(self.step(500, reason=reason)[:3], (reason, 2, 0))
      self.assertFalse(self.step(550, eps=3)[3])
      for ms in range(600, 851, 50):
        self.assertFalse(self.step(ms)[3])
      self.step(900)
      self.assertEqual(self.step(950)[1], 4)
      self.assertTrue(self.step(1000, eps=3)[3])

  def test_faults_and_pedals_never_resume_without_a_new_request(self):
    for reason in ('can_invalid', 'driver_brake', 'driver_accelerator', 'stock_acc_fault', 'eps_feedback_stale'):
      self.admission = P87Admission()
      self.activate()
      self.step(500, reason=reason)
      for ms in range(550, 1001, 50):
        self.assertFalse(self.step(ms)[3])
        self.assertTrue(self.admission.blocked)

  def test_eps_loss_clock_gap_and_clock_reversal_block(self):
    for ms, changes in ((500, {'eps': 1}), (700, {}), (400, {})):
      self.admission = P87Admission()
      self.activate()
      self.assertFalse(self.step(ms, **changes)[3])
      self.assertTrue(self.admission.blocked)

  def test_invalid_lanes_withdraw_unacknowledged_request(self):
    self.wait_for_ack()
    self.assertEqual(self.step(450, model_valid=False)[1:3], (2, 0))
    self.assertFalse(self.step(500, eps=3)[3])

  def test_lane_geometry_and_lane_change(self):
    source = ModelSource()
    self.assertTrue(lanes_ready(source.model))
    for left, right in ((0., 1.7), (-.3, 1.), (-3., 3.), (float('nan'), 1.7)):
      source.model.laneLines[1].y = [left]
      source.model.laneLines[2].y = [right]
      self.assertFalse(lanes_ready(source.model))
    source = ModelSource()
    source.model.meta.laneChangeState = 'laneChangeStarting'
    self.assertFalse(lanes_ready(source.model))


class TestNativeAck(unittest.TestCase):
  def test_old_eps_three_cannot_start_torque_even_if_host_requests_it(self):
    native = native_fixture.TestP87Safety()
    native.setUp()
    native.feed()
    self.assertFalse(native.safety.safety_tx_hook(native.command()))
    self.assertFalse(native.safety.safety_tx_hook(native.command(0, 100, 4)))
    native.ack()
    self.assertTrue(native.safety.safety_tx_hook(native.command()))

  def test_ack_must_follow_request_and_neutral_withdraws_it(self):
    native = native_fixture.TestP87Safety()
    native.setUp()
    native.feed(eps=2)
    self.assertTrue(native.safety.safety_tx_hook(native.command(0, 10, 4)))
    native.feed(eps=3)  # Same timer: not a later ACK.
    self.assertFalse(native.safety.safety_tx_hook(native.command()))
    native.ack()
    self.assertTrue(native.safety.safety_tx_hook(native.command()))
    self.assertTrue(native.safety.safety_tx_hook(native.command(0, 0, 2)))
    self.assertFalse(native.safety.safety_tx_hook(native.command()))

  def test_native_ack_timeout_and_eps_withdrawal(self):
    native = native_fixture.TestP87Safety()
    native.setUp()
    native.feed(eps=2)
    self.assertTrue(native.safety.safety_tx_hook(native.command(0, 10, 4)))
    native.safety.set_timer(1_100_000)
    native.feed(eps=2)
    self.assertFalse(native.safety.safety_tx_hook(native.command(0, 20, 4)))
    native.clock = 1_100_000
    native.feed(request=0)
    native.feed(request=1)
    native.authorize()
    native.feed(eps=1)
    self.assertFalse(native.safety.safety_tx_hook(native.command()))

  def test_native_first_ack_must_arrive_before_timeout(self):
    for delay in (999_999, 1_000_000, 1_000_001):
      native = native_fixture.TestP87Safety()
      native.setUp()
      native.feed(eps=2)
      self.assertTrue(native.safety.safety_tx_hook(native.command(0, 10, 4)))
      for elapsed in (200_000, 400_000, 600_000, 800_000):
        native.safety.set_timer(100_000+elapsed)
        native.feed(eps=2)
      native.safety.set_timer(100_000+delay)
      native.feed(eps=3)
      self.assertEqual(bool(native.safety.safety_tx_hook(native.command())), delay < 1_000_000)

  def test_unfiltered_driver_override_inhibits_host_immediately(self):
    helper = host_fixture.TestP87Interface()
    ci = helper.interface(True)
    ci.CC.model_sm.logMonoTime['modelV2'] = 1_000_000_000
    state = helper.state(p87_driver_torque_raw=48., steeringTorque=40.)
    output, sends = ci.CC.update(helper.control(), structs.CarControlSP(), state, 1_000_000_000)
    self.assertEqual(output.torqueOutputCan, 0)
    self.assertEqual(ci.CC.p87_lateral_status['phase'], 'driver_torque_override')
    self.assertEqual(sends[0].dat[5] >> 1, 0)

  def test_late_rejection_echo_cannot_clear_a_manual_rearm_latch(self):
    helper = host_fixture.TestP87Interface()
    ci = helper.interface(True)
    ci.CC.p87_admission.block('eps_ack_timeout')
    ci.CC.model_sm.logMonoTime['modelV2'] = 1_000_000_000
    output, _ = ci.CC.update(helper.control(), structs.CarControlSP(),
      helper.state(p87_lka_rejection_count=1), 1_000_000_000)
    self.assertEqual(output.torqueOutputCan, 0)
    self.assertTrue(ci.CC.p87_admission.blocked)
    self.assertEqual(ci.CC.p87_lateral_status['phase'], 'eps_ack_timeout')


if __name__ == '__main__':
  unittest.main()
