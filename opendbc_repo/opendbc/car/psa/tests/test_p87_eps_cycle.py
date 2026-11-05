import math
from types import SimpleNamespace as NS
import unittest

from opendbc.car import structs
from opendbc.car.psa.p87_admission import P87Admission
from opendbc.car.psa.p87_eps_cycle import P87EpsCycleGate, native_hands_ok, straight_corridor
from opendbc.car.psa.tests import test_5008 as host_fixture
from opendbc.safety.tests import test_psa_5008 as native_fixture
from opendbc.safety.tests.common import make_msg


def prediction():
  model = host_fixture.ModelSource().model
  model.action = NS(desiredCurvature=0.)
  model.orientationRate = NS(t=[i*.5 for i in range(11)], z=[0.]*11)
  model.velocity = NS(x=[25.]*11)
  return model


def held():
  return dict(STEERWHL_HOLD_BY_DRV=1, TRQ_LIMIT_STATE=0, STEERING_REBOOT_REQUEST=0)


class TestCycleGate(unittest.TestCase):
  def setUp(self):
    self.gate = P87EpsCycleGate()
    self.car = structs.CarState(vEgo=25., vEgoRaw=25., yawRate=0.)
    self.model = prediction()

  def step(self, ms, **changes):
    now = 1_000_000_000 + ms*1_000_000
    args = dict(requested=True, active=True, eligible=True, hands_ok=True,
                car=self.car, model=self.model, model_valid=True, model_ns=now)
    args.update(changes)
    return self.gate.update(now, **args)

  def test_straight_needs_stability_and_distinct_predictions(self):
    for ms in range(0, 1300, 50):
      self.assertFalse(self.step(ms))
    self.assertTrue(self.step(1300))
    self.gate = P87EpsCycleGate()
    for ms in range(0, 1351, 50):
      self.assertFalse(self.step(ms, model_ns=1_000_000_000))

  def test_missing_native_hands_cannot_be_inferred_from_light_driver_torque(self):
    self.assertFalse(native_hands_ok(NS()))
    self.assertFalse(native_hands_ok(NS(is_dat_dira={'STEERWHL_HOLD_BY_DRV': 1})))
    self.assertTrue(native_hands_ok(NS(is_dat_dira=held())))
    for key in held():
      data = held(); data[key] = 0 if key == 'STEERWHL_HOLD_BY_DRV' else 1
      self.assertFalse(native_hands_ok(NS(is_dat_dira=data)))

  def test_hands_loss_withdraws_readiness_and_return_needs_full_new_stability(self):
    for ms in range(0, 1301, 50): result = self.step(ms)
    self.assertTrue(result)
    self.assertFalse(self.step(1350, hands_ok=False))
    self.assertEqual(self.gate.reason, 'hands_not_confirmed')
    for ms in range(1400, 2700, 50):
      self.assertFalse(self.step(ms))
    self.assertTrue(self.step(2700))

  def test_curves_and_missing_or_nonfinite_prediction_never_renew(self):
    self.model.orientationRate.z[-1] = .03
    self.assertFalse(straight_corridor(self.car, self.model))
    self.model = prediction(); self.car.yawRate = .03
    self.assertFalse(straight_corridor(self.car, self.model))
    self.car.yawRate = 0.
    for change in ('shape', 'future', 'nan', 'gap', 'short', 'negative'):
      model = prediction()
      if change == 'shape': model.velocity.x.pop()
      elif change == 'future': model.orientationRate.t[0] = .2
      elif change == 'nan': model.orientationRate.z[3] = math.nan
      elif change == 'gap': model.orientationRate.t[3] += .2
      elif change == 'short': model.orientationRate.t = model.orientationRate.t[:-1]; model.orientationRate.z.pop(); model.velocity.x.pop()
      else: model.velocity.x[1] = -1.
      self.assertFalse(straight_corridor(self.car, model), change)
    self.assertFalse(straight_corridor(self.car, NS()))

  def test_stale_future_reversed_and_invalid_models_withdraw_readiness(self):
    for changes in ({'model_ns': 0}, {'model_ns': 4_000_000_000}, {'model_valid': False}, {'model_ns': 1_000_000_000}):
      self.gate = P87EpsCycleGate()
      for ms in range(0, 1301, 50): result = self.step(ms)
      self.assertTrue(result)
      self.assertFalse(self.step(1350, **changes))

  def test_no_planned_release_below_fifty_even_if_eps_remains_active(self):
    self.car.vEgoRaw = 40./3.6
    for ms in range(0, 1501, 50):
      self.assertFalse(self.step(ms))
    self.assertEqual(self.gate.reason, 'below_cycle_speed')


class TestCycleAdmission(unittest.TestCase):
  def setUp(self):
    self.admission = P87Admission()

  def step(self, ms, **changes):
    now = 1_000_000_000 + ms*1_000_000
    args = dict(requested=True, reason=None, eps=3 if ms >= 450 else 2, eps_ns=now,
                model_valid=True, model_ns=now, tick=ms % 50 == 0, cycle_ready=False)
    args.update(changes)
    return self.admission.update(now, **args)

  def activate(self):
    for ms in range(0, 10401, 50): result = self.step(ms)
    self.assertEqual(result[0], 'active')

  def release(self):
    self.activate()
    self.assertEqual(self.step(10450, cycle_ready=True), ('cycle_releasing', 2, 0, False))

  def test_no_forced_deadline_or_early_cycle(self):
    self.activate()
    self.assertEqual(self.step(10440, cycle_ready=True)[0], 'active')
    for ms in range(10450, 20451, 50):
      self.assertEqual(self.step(ms)[0], 'active')
    self.assertFalse(self.admission.cycling)

  def test_release_new_models_preparation_and_fresh_ack_before_torque(self):
    self.release()
    self.assertEqual(self.step(10500, eps=0, reason='eps_not_authorized')[1:3], (2, 0))
    self.assertEqual(self.step(10550, eps=2)[0], 'validating_lanes')
    for ms in range(10600, 10951, 50):
      self.assertFalse(self.step(ms, eps=2)[3])
    self.assertEqual(self.admission.phase, 'waiting_for_eps_ack')
    self.assertFalse(self.step(11000, eps=3, eps_ns=self.admission.request_ns)[3])
    self.assertTrue(self.step(11050)[3])
    self.assertEqual(self.admission.cycle_count, 1)
    self.assertEqual(self.admission.active_since, 12_050_000_000)
    self.assertFalse(self.admission.cycling)

  def test_cached_release_cannot_prepare(self):
    self.release()
    self.assertEqual(self.step(10500, eps=2, eps_ns=11_450_000_000)[0], 'cycle_releasing')

  def test_release_timeout_latches_and_late_release_cannot_retry(self):
    self.release()
    for ms in range(10500, 11451, 50): result = self.step(ms)
    self.assertEqual(result[:3], ('eps_cycle_release_timeout', 2, 0))
    self.assertFalse(self.step(11500, eps=2)[3])
    self.assertTrue(self.admission.blocked)

  def test_missing_ack_latches_and_late_ack_cannot_retry(self):
    self.release()
    for ms in range(10500, 12001, 50): result = self.step(ms, eps=2)
    self.assertEqual(result[:3], ('eps_ack_timeout', 2, 0))
    self.assertFalse(self.step(12050)[3])
    self.assertEqual(self.admission.cycle_count, 0)

  def test_fault_pedal_override_indicator_and_gate_loss_abort_without_auto_resume(self):
    for reason in ('driver_brake', 'driver_accelerator', 'driver_torque_override', 'driver_indicator',
                   'eps_fault_or_unknown', 'eps_cycle_gate_lost', 'camera_template_stale', 'eps_cycle_panda_rejected'):
      self.admission = P87Admission(); self.release()
      self.assertEqual(self.step(10500, reason=reason)[1:3], (2, 0))
      self.assertTrue(self.admission.blocked, reason)
      self.assertFalse(self.step(10550, eps=2)[3])
      self.step(10600, requested=False)
      self.assertEqual(self.step(10650, eps=2)[0], 'validating_lanes')

  def test_unsolicited_eps_loss_is_never_a_cycle(self):
    self.activate()
    self.assertFalse(self.step(10450, eps=0, reason='eps_not_authorized', cycle_ready=True)[3])
    self.assertTrue(self.admission.blocked)
    self.assertEqual(self.admission.cycle_count, 0)

  def test_invalid_models_and_clock_gap_abort_during_cycle(self):
    for changes in ({'model_valid': False, 'eps': 2}, {'eps': 2}):
      self.admission = P87Admission(); self.release()
      ms = 10500 if 'model_valid' in changes else 10700
      self.assertFalse(self.step(ms, **changes)[3])
      self.assertTrue(self.admission.blocked)

  def test_total_cycle_timeout_cannot_be_extended_by_returning_eps_three(self):
    self.release()
    self.step(10500, eps=2)
    for ms in range(10550, 13451, 50): result = self.step(ms)
    self.assertEqual(result[:3], ('eps_cycle_timeout', 2, 0))
    self.assertEqual(self.admission.cycle_count, 0)

  def test_first_abort_reason_survives_subsequent_withdrawal(self):
    self.release()
    self.admission.block('eps_cycle_panda_rejected')
    self.assertEqual(self.step(10500, eps=0, reason='eps_not_authorized')[0], 'eps_cycle_panda_rejected')


class TestCycleControllerNative(unittest.TestCase):
  def run_cycle(self, interrupt=None, *, speed=90., direction=1, duration_ms=12500):
    helper = host_fixture.TestP87Interface(); ci = helper.interface(True)
    ci.CC.model_sm.model = prediction()
    native = native_fixture.TestP87Safety(); native.setUp()
    cc = structs.CarControl(latActive=True, enabled=True); cc.actuators.torque = direction*.06
    statuses = []; previous = 0; cycle_zeros = 0; release_tick = None; max_reduction = 0
    interrupted = False
    physical_eps, physical_hands, eps_stamp = 2, held(), 1_000_000_000
    for ms in range(0, duration_ms+1, 10):
      now = 1_000_000_000 + ms*1_000_000
      a = ci.CC.p87_admission
      if a.phase != 'cycle_releasing': release_tick = None
      eps = 2
      if a.phase == 'active': eps = 3
      elif a.phase == 'waiting_for_eps_ack' and now > a.request_ns: eps = 3
      elif a.phase == 'cycle_releasing':
        if release_tick is None: release_tick = ms
        eps = 0 if ms-release_tick < 100 else 2
      changes = {}
      if interrupt and a.cycling: interrupted = True
      if interrupted:
        if interrupt == 'hands': changes['is_dat_dira'] = dict(held(), STEERWHL_HOLD_BY_DRV=0)
        elif interrupt == 'panda': changes['p87_lka_rejection_count'] = 1
        elif interrupt == 'curve': ci.CC.model_sm.model.action.desiredCurvature = .01
      native.clock = now//1000; native.safety.set_timer(native.clock)
      if ms % 50 == 0:
        native.feed(eps=eps)
        physical_eps, eps_stamp = eps, now
        physical_hands = changes.get('is_dat_dira', held())
        native.safety.safety_rx_hook(native.packer.make_can_msg_safety('IS_DAT_DIRA', 0,
          dict(physical_hands, EPS_STATE_LKA=physical_eps)))
        ci.CC.model_sm.logMonoTime['modelV2'] = now
      eps = physical_eps
      state = helper.state(speed_kph=speed, vEgo=speed/3.6, vEgoRaw=speed/3.6, eps_state_lka=eps, eps_active=eps==3,
        p87_eps_timestamp=eps_stamp, p87_lka_timestamp=now, is_dat_dira=physical_hands)
      for key, value in changes.items():
        if key != 'is_dat_dira': setattr(state, key, value)
      output, sends = ci.CC.update(cc.as_reader(), structs.CarControlSP(), state, now)
      for send in sends:
        self.assertEqual((send.address, send.src), (0x3F2, 0))
        self.assertTrue(native.safety.safety_tx_hook(make_msg(send.src, send.address, dat=send.dat)), (ms, ci.CC.p87_lateral_status))
        status = (send.dat[4] >> 2) & 7; statuses.append(status)
        if ci.CC.p87_lateral_status['eps_cycle_ramping_zero']:
          max_reduction = max(max_reduction, abs(previous)-abs(output.torqueOutputCan))
          self.assertLessEqual(abs(previous)-abs(output.torqueOutputCan), 4)
        previous = output.torqueOutputCan
      if a.cycling:
        self.assertEqual(output.torqueOutputCan, 0); cycle_zeros += 1
    return ci, statuses, cycle_zeros, max_reduction

  def test_complete_host_cycle_is_accepted_by_actual_panda_and_ramps_from_zero(self):
    ci, statuses, zeros, reduction = self.run_cycle()
    self.assertEqual(ci.CC.p87_admission.cycle_count, 1)
    self.assertEqual(ci.CC.p87_admission.phase, 'active')
    self.assertGreater(zeros, 30)
    self.assertGreater(reduction, 0)
    self.assertEqual(statuses[-1], 4)

  def test_hands_loss_curve_and_panda_rejection_during_cycle_latch_neutral(self):
    for interrupt in ('hands', 'curve', 'panda'):
      ci, _, _, _ = self.run_cycle(interrupt)
      self.assertTrue(ci.CC.p87_admission.blocked, interrupt)
      self.assertEqual(ci.CC.apply_torque_scaled_last, 0)
      self.assertEqual(ci.CC.p87_admission.cycle_count, 0)

  def test_repeated_cycles_at_110_in_both_directions_restart_the_period_after_ack(self):
    for direction in (-1, 1):
      ci, _, _, _ = self.run_cycle(speed=110., direction=direction, duration_ms=24500)
      self.assertEqual(ci.CC.p87_admission.cycle_count, 2)
      self.assertEqual(ci.CC.p87_admission.phase, 'active')


if __name__ == '__main__':
  unittest.main()
