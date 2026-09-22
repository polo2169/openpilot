from dataclasses import replace
import math
import random
import unittest

from opendbc.car.psa.lka import LkaDecision, LkaFeedback, LkaInputs, LkaPhase, T9LkaLifecycle
from opendbc.car.psa.lka_feedback import T9LkaCanObserver


def nanos(ms):
  return 1_000_000_000 + ms * 1_000_000


def inputs(ms, *, eps=1, stock_state=3, **changes):
  return replace(LkaInputs(
    requested=True, torque=1.0, speed_kph=72.0, vehicle_ready=True, can_valid=True,
    safety_rx_nanos=nanos(ms),
    feedback=LkaFeedback(stock_state=stock_state, stock_nanos=nanos(ms), eps_state=eps, eps_nanos=nanos(ms)),
  ), **changes)


def active_machine():
  machine = T9LkaLifecycle()
  for ms in range(0, 851, 50):
    decision = machine.update(nanos(ms), inputs(ms, eps=3 if ms >= 150 else 1))
  assert decision.phase == LkaPhase.ACTIVE
  assert decision.torque_raw == 1
  return machine


class TestT9LkaLifecycle(unittest.TestCase):
  def test_activation_requires_new_eps_response_and_full_blend(self):
    machine = T9LkaLifecycle()
    initial = machine.update(nanos(0), inputs(0, requested=False))
    self.assertEqual((initial.phase, initial.state), (LkaPhase.DISABLED, 2))
    rows = []
    for ms in range(50, 901, 50):
      rows.append(machine.update(nanos(ms), inputs(ms, eps=3 if ms >= 200 else 1)))
    self.assertEqual(rows[0].state, 3)
    self.assertEqual(rows[2].phase, LkaPhase.WAITING_EPS)
    self.assertEqual(rows[2].state, 4)
    self.assertEqual(rows[2].torque_raw, 0)
    self.assertEqual(rows[3].phase, LkaPhase.ACTIVE)
    self.assertEqual(rows[3].torque_raw, 0)
    self.assertEqual(rows[-1].torque_raw, 1)
    for row in rows:
      self.assertFalse(row.tx_allowed)
      if row.torque_raw:
        self.assertEqual((row.state, row.factor, row.eps_confirmed), (4, 100, True))

  def test_state_four_on_stock_bus_is_not_eps_confirmation(self):
    machine = T9LkaLifecycle()
    for ms in range(0, 651, 50):
      decision = machine.update(nanos(ms), inputs(ms, stock_state=4))
      self.assertEqual(decision.torque_raw, 0)
    self.assertEqual(decision.reason, "eps_activation_timeout")

  def test_old_eps_active_cannot_acknowledge_new_request(self):
    machine = T9LkaLifecycle()
    for ms in (0, 50, 100):
      machine.update(nanos(ms), inputs(ms))
    snapshot = inputs(150, eps=3)
    snapshot = replace(snapshot, feedback=replace(snapshot.feedback, eps_nanos=nanos(100)))
    result = machine.update(nanos(150), snapshot)
    self.assertEqual(result.phase, LkaPhase.WAITING_EPS)
    self.assertFalse(result.eps_confirmed)
    self.assertEqual(result.torque_raw, 0)

  def test_late_response_is_not_accepted_and_true_request_does_not_retry(self):
    machine = T9LkaLifecycle()
    for ms in range(0, 801, 50):
      result = machine.update(nanos(ms), inputs(ms, eps=3 if ms >= 600 else 1))
    self.assertEqual(result.phase, LkaPhase.BLOCKED)
    self.assertEqual(result.reason, "eps_activation_timeout")
    self.assertTrue(result.rearm_required)
    self.assertEqual(result.torque_raw, 0)

  def test_eps_must_be_released_before_starting_a_cycle(self):
    machine = T9LkaLifecycle()
    result = machine.update(nanos(0), inputs(0, eps=3))
    self.assertEqual(result.reason, "eps_not_released")
    machine.update(nanos(50), inputs(50, eps=3, requested=False))
    result = machine.update(nanos(100), inputs(100))
    self.assertEqual(result.phase, LkaPhase.BLOCKED)
    machine.update(nanos(150), inputs(150, requested=False))
    result = machine.update(nanos(200), inputs(200))
    self.assertEqual(result.phase, LkaPhase.PREPARING)

  def test_eps_activation_before_state_four_is_rejected(self):
    machine = T9LkaLifecycle()
    machine.update(nanos(0), inputs(0))
    result = machine.update(nanos(50), inputs(50, eps=3))
    self.assertEqual(result.reason, "eps_active_before_request")

  def test_faults_cut_torque_between_proposal_ticks(self):
    cases = [
      ({"brake_pressed": True}, "brake_pressed"),
      ({"driver_override": True}, "driver_override"),
      ({"driver_torque_raw": 16}, "driver_override"),
      ({"driver_torque_raw": -16}, "driver_override"),
      ({"vehicle_ready": False}, "vehicle_not_ready"),
      ({"can_valid": False}, "can_invalid"),
      ({"speed_kph": 67.0}, "speed_outside_candidate_envelope"),
      ({"speed_kph": 140.01}, "speed_outside_candidate_envelope"),
      ({"torque": math.nan}, "nonfinite_input"),
      ({"speed_kph": math.inf}, "nonfinite_input"),
      ({"driver_torque_raw": math.nan}, "nonfinite_input"),
      ({"safety_rx_nanos": nanos(0)}, "safety_rx_stale"),
      ({"safety_rx_nanos": nanos(900)}, "safety_rx_stale"),
    ]
    for changes, reason in cases:
      with self.subTest(reason=reason, changes=changes):
        result = active_machine().update(nanos(851), inputs(851, eps=3, **changes))
        self.assertEqual(result.reason, reason)
        self.assertEqual((result.state, result.factor, result.torque_raw), (2, 0, 0))
        self.assertTrue(result.rearm_required)

  def test_140_kph_boundary_and_rearm(self):
    machine = active_machine()
    self.assertEqual(machine.update(nanos(851), inputs(851, eps=3, speed_kph=140.)).phase, LkaPhase.ACTIVE)
    result = machine.update(nanos(861), inputs(861, eps=3, speed_kph=140.01))
    self.assertEqual(result.reason, "speed_outside_candidate_envelope")
    self.assertEqual(result.torque_raw, 0)
    self.assertEqual(machine.update(nanos(871), inputs(871, speed_kph=140.)).phase, LkaPhase.BLOCKED)
    machine.update(nanos(881), inputs(881, requested=False))
    self.assertEqual(machine.update(nanos(891), inputs(891, speed_kph=140.)).phase, LkaPhase.PREPARING)

  def test_feedback_failures_are_distinct_from_requested_state(self):
    cases = [
      ({"eps_state": 1}, "eps_activation_lost"),
      ({"eps_state": 0}, "eps_not_authorized"),
      ({"eps_state": 4}, "eps_fault"),
      ({"eps_state": 7}, "eps_feedback_invalid"),
      ({"eps_state": None}, "eps_feedback_invalid"),
      ({"eps_nanos": nanos(0)}, "eps_feedback_stale"),
      ({"eps_nanos": nanos(900)}, "eps_feedback_stale"),
      ({"stock_nanos": nanos(0)}, "stock_lka_stale"),
      ({"stock_state": 2}, "stock_lka_not_authorized"),
      ({"stock_state": 5}, "stock_lka_unavailable"),
      ({"stock_state": None}, "stock_lka_unavailable"),
      ({"stock_factor": 101}, "stock_torque_api_invalid"),
      ({"stock_angle_raw": 1}, "stock_torque_api_invalid"),
      ({"stock_lxa": 1}, "stock_torque_api_invalid"),
    ]
    for changes, reason in cases:
      with self.subTest(changes=changes):
        snapshot = inputs(851, eps=3)
        snapshot = replace(snapshot, feedback=replace(snapshot.feedback, **changes))
        result = active_machine().update(nanos(851), snapshot)
        self.assertEqual(result.reason, reason)
        self.assertEqual(result.torque_raw, 0)

  def test_initial_inhibition_does_not_auto_engage_when_it_clears(self):
    machine = T9LkaLifecycle()
    machine.update(nanos(0), inputs(0, stock_state=2))
    result = machine.update(nanos(50), inputs(50))
    self.assertEqual(result.phase, LkaPhase.BLOCKED)
    machine.update(nanos(100), inputs(100, requested=False))
    result = machine.update(nanos(150), inputs(150))
    self.assertEqual(result.phase, LkaPhase.PREPARING)

  def test_cancel_zeros_torque_immediately_while_eps_release_is_observed(self):
    machine = active_machine()
    result = machine.update(nanos(851), inputs(851, eps=3, requested=False))
    self.assertEqual((result.phase, result.state, result.torque_raw), (LkaPhase.RELEASING, 3, 0))
    self.assertEqual(result.factor, 100)
    for ms in range(901, 1902, 50):
      result = machine.update(nanos(ms), inputs(ms, eps=1 if ms >= 1901 else 3, requested=False))
      self.assertEqual(result.torque_raw, 0)
    self.assertEqual(result.phase, LkaPhase.DISABLED)
    self.assertEqual(result.factor, 0)
    self.assertFalse(result.rearm_required)

  def test_request_during_release_needs_another_explicit_off_on_cycle(self):
    machine = active_machine()
    machine.update(nanos(851), inputs(851, eps=3, requested=False))
    for ms in range(901, 1952, 50):
      result = machine.update(nanos(ms), inputs(ms, eps=1))
    self.assertEqual(result.phase, LkaPhase.BLOCKED)
    machine.update(nanos(2001), inputs(2001, requested=False))
    self.assertEqual(machine.update(nanos(2051), inputs(2051)).phase, LkaPhase.PREPARING)

  def test_eps_that_never_releases_is_reported_and_cannot_reengage(self):
    machine = active_machine()
    for ms in range(900, 2451, 50):
      result = machine.update(nanos(ms), inputs(ms, eps=3, requested=False))
    self.assertEqual(result.phase, LkaPhase.BLOCKED)
    self.assertEqual(result.reason, "eps_release_timeout")

  def test_release_does_not_keep_proposing_authorized_after_stock_withdrawal(self):
    machine = active_machine()
    machine.update(nanos(851), inputs(851, eps=3, requested=False))
    result = machine.update(nanos(852), inputs(852, eps=3, stock_state=2, requested=False))
    self.assertEqual((result.state, result.factor, result.torque_raw), (2, 0, 0))
    self.assertEqual(result.reason, "stock_lka_not_authorized")

  def test_time_faults_reset_torque_and_require_rearming(self):
    for timestamp, reason in ((nanos(850) + 150_000_001, "control_update_timeout"),
                              (nanos(849), "clock_reversed"), (0, "invalid_clock"), (math.nan, "invalid_clock")):
      with self.subTest(reason=reason):
        result = active_machine().update(timestamp, inputs(1001, eps=3))
        self.assertEqual(result.reason, reason)
        self.assertEqual((result.factor, result.torque_raw), (0, 0))

  def test_duplicate_and_fast_updates_cannot_accelerate_torque_changes(self):
    machine = active_machine()
    for ms in range(851, 900):
      result = machine.update(nanos(ms), inputs(ms, eps=3, torque=-1))
      self.assertEqual(result.torque_raw, 1)
    self.assertEqual(machine.update(nanos(900), inputs(900, eps=3, torque=-1)).torque_raw, 0)
    self.assertEqual(machine.update(nanos(900), inputs(900, eps=3, torque=-1)).torque_raw, 0)
    self.assertEqual(machine.update(nanos(950), inputs(950, eps=3, torque=-1)).torque_raw, -1)

  def test_existing_normalized_torque_scale_is_preserved_before_stage_one_clamp(self):
    machine = active_machine()
    self.assertEqual(machine.update(nanos(900), inputs(900, eps=3, torque=0.1)).torque_raw, 1)
    self.assertEqual(machine.update(nanos(950), inputs(950, eps=3, torque=0.0)).torque_raw, 0)
    self.assertEqual(machine.update(nanos(1000), inputs(1000, eps=3, torque=-0.1)).torque_raw, -1)

  def test_continuous_feedback_does_not_invent_a_ten_second_rearm(self):
    machine = active_machine()
    for ms in range(900, 13001, 50):
      result = machine.update(nanos(ms), inputs(ms, eps=3, torque=999))
      self.assertEqual(result.phase, LkaPhase.ACTIVE)
      self.assertEqual(result.torque_raw, 1)
      self.assertFalse(result.tx_allowed)

  def test_varied_inputs_never_produce_can_authorization(self):
    rng = random.Random(308)
    machine = T9LkaLifecycle()
    for ms in range(0, 50000, 50):
      result = machine.update(nanos(ms), inputs(ms, eps=rng.randrange(8), requested=rng.choice((True, False)),
                                             stock_state=rng.randrange(8), torque=rng.uniform(-10, 10)))
      self.assertFalse(result.tx_allowed)
      self.assertLessEqual(abs(result.torque_raw), 1)
      if result.torque_raw:
        self.assertEqual((result.phase, result.state, result.factor), (LkaPhase.ACTIVE, 4, 100))
    with self.assertRaises(AttributeError):
      LkaDecision(LkaPhase.ACTIVE, 4, 100, 1, "test", False, True).tx_allowed = True


class TestT9LkaCanObserver(unittest.TestCase):
  def test_decodes_received_command_and_eps_as_separate_fields(self):
    observer = T9LkaCanObserver()
    observer.update([(nanos(0), [(0x3F2, bytes.fromhex("0000140010C80000"), 0), (0x495, bytes.fromhex("00000C00"), 0)])])
    self.assertEqual(observer.feedback.stock_state, 4)
    self.assertEqual(observer.feedback.stock_factor, 100)
    self.assertEqual(observer.feedback.eps_state, 3)
    self.assertEqual(observer.feedback.eps_nanos, nanos(0))

  def test_wrong_bus_and_other_ids_do_not_refresh_feedback(self):
    observer = T9LkaCanObserver()
    observer.update([(nanos(0), [(0x495, bytes.fromhex("00000C00"), 1), (0x1495, bytes(4), 0)])])
    self.assertEqual(observer.feedback, LkaFeedback())

  def test_truncated_oversized_or_reordered_frames_invalidate_previous_feedback(self):
    for address, valid_frame in ((0x495, bytes.fromhex("00000C00")), (0x3F2, bytes.fromhex("0000140010C80000"))):
      for timestamp, bad_frame in ((nanos(50), valid_frame[:-1]), (nanos(50), valid_frame + b"\x00"), (nanos(0) - 1, valid_frame)):
        with self.subTest(address=address, timestamp=timestamp, bad_frame=bad_frame):
          observer = T9LkaCanObserver()
          observer.update([(nanos(0), [(address, valid_frame, 0)])])
          observer.update([(timestamp, [(address, bad_frame, 0)])])
          self.assertIsNone(observer.feedback.eps_state if address == 0x495 else observer.feedback.stock_state)


if __name__ == "__main__":
  unittest.main()
