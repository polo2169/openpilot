from dataclasses import replace
import math
import unittest

from opendbc.car.psa.rvv import RvvInputs, T9RvvAdapter


def sample(ms, **changes):
  now = 1_000_000_000 + ms * 1_000_000
  return now, replace(RvvInputs(
    requested=True, requested_accel_ms2=-0.2, speed_kph=80., stock_setpoint_kph=90.,
    cruise_active=True, cruise_available=True, vehicle_ready=True, can_valid=True,
    safety_rx_nanos=now, stock_rx_nanos=now,
  ), **changes)


class TestRvvAdapter(unittest.TestCase):
  def test_explicit_setpoint_without_acceleration_has_bounded_faster_reduction(self):
    adapter = T9RvvAdapter()
    for ms in range(0, 3001, 10):
      decision = adapter.update(*sample(ms, requested_accel_ms2=None, target_setpoint_kph=84.))
      self.assertEqual(decision.phase, 'tracking')
      self.assertEqual(decision.target_setpoint_kph, 84)
      self.assertEqual(decision.proposed_setpoint_kph, max(84, 90 - ms // 100))
      self.assertFalse(decision.tx_allowed)

  def test_legacy_acceleration_proposal_keeps_half_second_steps(self):
    adapter = T9RvvAdapter()
    for ms in range(0, 2001, 10):
      decision = adapter.update(*sample(ms))
      self.assertFalse(decision.tx_allowed)
      self.assertEqual(decision.proposed_setpoint_kph, 90 - ms // 500)
      self.assertEqual(decision.target_setpoint_kph, 78)

  def test_large_timing_gap_never_catches_up(self):
    adapter = T9RvvAdapter()
    adapter.update(*sample(0))
    decision = adapter.update(*sample(1000))
    self.assertEqual(decision.reason, 'control_update_timeout')
    self.assertIsNone(decision.proposed_setpoint_kph)

  def test_driver_lower_ceiling_immediate_and_return_is_gradual(self):
    adapter = T9RvvAdapter()
    adapter.update(*sample(0))
    self.assertEqual(adapter.update(*sample(10, stock_setpoint_kph=75.)).proposed_setpoint_kph, 75)
    for ms in range(20, 501, 10):
      decision = adapter.update(*sample(ms, stock_setpoint_kph=90., requested_accel_ms2=1.))
    self.assertEqual(decision.proposed_setpoint_kph, 76)
    self.assertEqual(decision.target_setpoint_kph, 90)

  def test_brake_gas_cancel_cruise_loss_latch_and_need_new_request(self):
    for change, reason in (({'brake_pressed': True}, 'brake_pressed'),
                           ({'gas_pressed': True}, 'driver_accelerator'),
                           ({'cancel': True}, 'driver_cancel'),
                           ({'cruise_active': False}, 'rvv_not_active'),
                           ({'cruise_available': False}, 'rvv_mode_not_selected')):
      with self.subTest(change=change):
        adapter = T9RvvAdapter()
        adapter.update(*sample(0))
        decision = adapter.update(*sample(10, **change))
        self.assertEqual(decision.reason, reason)
        self.assertTrue(decision.rearm_required)
        self.assertIsNone(adapter.update(*sample(20)).proposed_setpoint_kph)
        self.assertEqual(adapter.update(*sample(30, requested=False)).phase, 'disabled')
        self.assertEqual(adapter.update(*sample(40)).phase, 'tracking')

  def test_service_braking_is_reported_without_a_setpoint(self):
    for accel in (-0.30001, -1.2, -10.):
      decision = T9RvvAdapter().update(*sample(0, requested_accel_ms2=accel))
      self.assertTrue(decision.service_brake_required)
      self.assertIsNone(decision.proposed_setpoint_kph)
      self.assertFalse(decision.tx_allowed)

  def test_limits_invalid_values_stale_and_future_inputs(self):
    cases = [
      ({'stock_setpoint_kph': 255.}, 'setpoint_out_of_range'),
      ({'stock_setpoint_kph': 39.}, 'setpoint_out_of_range'),
      ({'stock_setpoint_kph': 141.}, 'setpoint_out_of_range'),
      ({'speed_kph': 39.}, 'speed_below_rvv_envelope'),
      ({'speed_kph': 40.}, 'target_below_rvv_envelope'),
      ({'requested_accel_ms2': math.nan}, 'nonfinite_input'),
      ({'requested_accel_ms2': None}, 'missing_acceleration_or_target'),
      ({'stock_setpoint_kph': math.inf}, 'nonfinite_input'),
      ({'vehicle_ready': False}, 'vehicle_not_ready'),
      ({'can_valid': False}, 'can_invalid'),
      ({'safety_rx_nanos': 1}, 'safety_rx_stale'),
      ({'stock_rx_nanos': 1_000_000_001}, 'rvv_rx_stale'),
    ]
    for changes, reason in cases:
      with self.subTest(changes=changes):
        self.assertEqual(T9RvvAdapter().update(*sample(0, **changes)).reason, reason)

  def test_clock_reversal_and_invalid_time(self):
    for bad in (0, -1, True, 1.5):
      _, inputs = sample(0)
      self.assertEqual(T9RvvAdapter().update(bad, inputs).reason, 'invalid_clock')
    adapter = T9RvvAdapter()
    adapter.update(*sample(10))
    self.assertEqual(adapter.update(*sample(0)).reason, 'clock_reversed')

  def test_disabled_never_activates_stock_cruise(self):
    decision = T9RvvAdapter().update(*sample(0, requested=False))
    self.assertEqual(decision.phase, 'disabled')
    self.assertIsNone(decision.proposed_setpoint_kph)
