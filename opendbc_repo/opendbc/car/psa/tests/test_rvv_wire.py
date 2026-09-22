import unittest
from dataclasses import replace

from opendbc.car.psa.rvv_following import FollowingDecision
from opendbc.car.psa.rvv_wire import WIRE, blocked, command, split_status


class TestRvvWire(unittest.TestCase):
  def setUp(self):
    self.now = 1_000_000_000
    self.decision = FollowingDecision('vision_following_candidate', target_kph=74, computed_ns=self.now,
                                     lead_rx_ns=self.now, model_ns=self.now, candidate_increase_permitted=True)

  def test_binary_contract_and_timestamp_layout(self):
    payload = command(self.decision, now=self.now, engaged=True)
    self.assertEqual(len(payload), 32)
    self.assertEqual(payload[:8], b'RVV1\x4a\x01\x00\x00')
    self.assertEqual(WIRE.unpack(payload)[4:], (self.now,) * 3)
    self.assertFalse(blocked(payload, self.now))
    self.assertTrue(blocked(payload, self.now + 150_000_001))

  def test_upper_setpoint_boundary_survives_binary_transport(self):
    payload = command(replace(self.decision, target_kph=140), now=self.now, engaged=True)
    self.assertEqual(WIRE.unpack(payload)[1], 140)
    self.assertFalse(blocked(payload, self.now))

  def test_lost_stale_invalid_and_disabled_decisions_release(self):
    cases = [dict(reason='lead_lost'), dict(target_kph=39), dict(target_kph=141), dict(target_kph=None),
             dict(target_kph=74.), dict(computed_ns=0), dict(computed_ns=self.now + 1),
             dict(lead_rx_ns=self.now-250_000_001), dict(model_ns=self.now-500_000_001),
             dict(rearm_required=True), dict(driver_intervention_required=True), dict(candidate_increase_permitted=False)]
    for fields in cases:
      with self.subTest(fields=fields):
        payload = command(replace(self.decision, **fields), now=self.now, engaged=True)
        self.assertEqual(payload[4], 0)
    self.assertEqual(command(self.decision, now=self.now, engaged=False)[4], 0)

  def test_blocked_signal_survives_even_without_target_and_rejects_bad_wire(self):
    payload = command(replace(self.decision, reason='lead_lost', rearm_required=True), now=self.now, engaged=True)
    self.assertTrue(blocked(payload, self.now))
    for payload in [b'', b'0'*32, payload[:-1], payload+b'0']:
      self.assertTrue(blocked(payload, self.now))

  def test_split_wire_version_and_axis_local_vs_shared_failure(self):
    payload = command(self.decision, now=self.now, engaged=True, split_axes=True)
    self.assertEqual(payload[:8], b'RVV2\x4a\x01\x00\x00')
    self.assertEqual(split_status(payload, self.now), (False, False, True))
    self.assertTrue(blocked(payload, self.now))  # Legacy consumer rejects it.
    for reason in ('lead_lost', 'fixed_cruise_cannot_handle_critical_lead',
                   'deceleration_exceeds_unvalidated_engine_brake_prior'):
      local = replace(self.decision, reason=reason, rearm_required=True)
      self.assertEqual(split_status(command(local, now=self.now, engaged=True, split_axes=True), self.now),
                       (False, True, False))
    for reason in ('invalid_clock', 'car_state_invalid_or_stale', 'calibration_invalid_or_stale',
                   'lead_invalid_or_stale', 'following_update_timeout', 'unrecognized_failure'):
      shared = replace(self.decision, reason=reason, rearm_required=True)
      self.assertEqual(split_status(command(shared, now=self.now, engaged=True, split_axes=True), self.now),
                       (True, True, False))

  def test_new_shared_fault_overrides_already_latched_local_reason(self):
    decision = replace(self.decision, reason='lead_lost', rearm_required=True, common_fault=True)
    payload = command(decision, now=self.now, engaged=True, split_axes=True)
    self.assertEqual(payload[4:6], bytes((0, 7)))
    self.assertEqual(split_status(payload, self.now), (True, True, False))

  def test_split_malformed_candidates_are_common_faults(self):
    valid = list(WIRE.unpack(command(self.decision, now=self.now, engaged=True, split_axes=True)))
    for index, value in ((0,b'RVV1'), (1,39), (1,141), (2,3), (2,9), (3,1),
                         (4,0), (4,self.now+1), (5,0), (5,self.now-250_000_001),
                         (6,0), (6,self.now-500_000_001)):
      fields = valid.copy(); fields[index] = value
      self.assertTrue(split_status(WIRE.pack(*fields), self.now)[0], (index,value))
    self.assertFalse(split_status(WIRE.pack(*valid), self.now+100_000_000)[0])
    self.assertTrue(split_status(WIRE.pack(*valid), self.now+100_000_001)[0])
    self.assertTrue(split_status(b'', self.now)[0])

  def test_split_waiting_and_disabled_are_truthful_zero_requests(self):
    for engaged in (False,True):
      payload = command(FollowingDecision('waiting_for_lead'), now=self.now, engaged=engaged, split_axes=True)
      self.assertEqual(split_status(payload,self.now), (False,False,False))


if __name__ == '__main__':
  unittest.main()
