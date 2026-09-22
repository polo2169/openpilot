from dataclasses import replace
import json
import math
import os
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.psa import rvv_wire
from opendbc.car.psa.rvv import RvvInputs, T9RvvAdapter
from opendbc.car.psa.rvv_control import T9RvvControl, decode_stock, replacement_payload, setpoint_parity
from opendbc.car.psa.rvv_following import RvvLead, T9RvvFollowing, T9RvvFollowingObserver


BASE = 1_000_000_000


def stock_frame(setpoint=90, active=True, mode=1, counter=0):
  data = bytearray.fromhex('32000014224282af')  # Recorded route e, 130 km/h.
  data[6] = setpoint
  data[0] = (data[0] & 0xCF) | (setpoint_parity(setpoint) << 4)
  data[7] = (int(active) << 7) | (mode << 5) | counter
  return bytes(data)


def inputs(now, **changes):
  return replace(RvvInputs(requested=True, requested_accel_ms2=-.2,
    speed_kph=80., stock_setpoint_kph=90., cruise_active=True, cruise_available=True,
    vehicle_ready=True, can_valid=True, safety_rx_nanos=now, stock_rx_nanos=now), **changes)


def tick(control, ms, *, unseparated=False, observed_active=None, payload=None, engine_state=None, **changes):
  now = BASE + ms * 1_000_000
  current = inputs(now, **changes)
  active = current.cruise_active if observed_active is None else observed_active
  raw_state = (2 if active else 0) if engine_state is None else engine_state
  engine = bytes([0, 0, 0, 0, raw_state << 2, 0, 0, 0])
  frames = [(0x208, engine, 0)]
  if ms % 100 == 0:
    data = payload if payload is not None else stock_frame(int(current.stock_setpoint_kph) if active else 255,
                                                        active, counter=(ms // 50) & 15)
    frames += [(0x50E, data, 2), (0x50E, data, 128)]
    if unseparated:
      frames += [(0x50E, data, 0)]
  control.observe([(now, frames)])
  return control.update(now, current)


def armed():
  control = T9RvvControl()
  for ms in range(0, 2200, 10):
    tick(control, ms, requested=False, cruise_active=False)
  return control


def lead(now, **changes):
  return replace(RvvLead(True, 5. + 2 * (80 / 3.6) - 1., 80 / 3.6, 0., .95,
                         now, now, True, False), **changes)


def follow(planner, ms, **changes):
  now = BASE + ms * 1_000_000
  values = dict(active=True, car_valid=True, car_ns=now, speed_kph=80., stock_setpoint_kph=90.,
                calibrated=True, calibration_ns=now, lead=lead(now))
  values.update(changes)
  return planner.update(now, **values)


def following_tick(control, planner, ms, *, lead_changes=None, **changes):
  now = BASE + ms * 1_000_000
  current = inputs(now, **({'stock_setpoint_kph': 82.} | changes))
  frames = [(0x208, bytes.fromhex('0000000008000000'), 0)]
  if ms % 100 == 0:
    frames.append((0x50E, stock_frame(int(current.stock_setpoint_kph), counter=(ms // 50) & 15), 2))
  control.observe([(now, frames)])
  decision = follow(planner, ms, stock_setpoint_kph=current.stock_setpoint_kph,
                    lead=lead(now, **(lead_changes or {})))
  return control.update_following(now, current, decision)


def observer_messages(now, *, active=True, speed_kph=80., stock_setpoint_kph=90., **lead_changes):
  car = structs.CarState(canValid=True, vEgoRaw=speed_kph/3.6)
  car.cruiseState.enabled = active
  car.cruiseState.speed = stock_setpoint_kph/3.6
  native = SimpleNamespace(**(dict(status=True, dRel=48.4, vLead=speed_kph/3.6, vRel=0.,
                                   modelProb=.95, radar=False) | lead_changes))
  messages = {'carState': car, 'radarState': SimpleNamespace(leadOne=native, mdMonoTime=now),
              'liveCalibration': SimpleNamespace(calStatus='calibrated')}
  class SM(dict):
    valid = {key: True for key in messages}
    alive = valid.copy()
    logMonoTime = {key: now for key in messages}
  return SM(messages)


class TestRvvPayload(unittest.TestCase):
  def test_integer_can_setpoints_survive_float32_carstate(self):
    for stock in range(40, 141):
      with self.subTest(stock=stock):
        converted = struct.unpack('f', struct.pack('f', stock / 3.6))[0] * 3.6
        proposal = follow(T9RvvFollowing(), 0, speed_kph=float(stock), stock_setpoint_kph=converted,
                          lead=lead(BASE, speed_ms=stock/3.6, distance_m=100.))
        self.assertEqual(proposal.reason, 'stock_setpoint_sufficient')
        self.assertIsNone(proposal.target_kph)
        self.assertFalse(proposal.tx_allowed)
        adapted = T9RvvAdapter().update(BASE, inputs(BASE, stock_setpoint_kph=converted,
          speed_kph=float(stock), requested_accel_ms2=0., target_setpoint_kph=float(stock)))
        self.assertEqual((adapted.proposed_setpoint_kph, adapted.target_setpoint_kph), (stock, stock))
        self.assertFalse(adapted.tx_allowed)

  def test_recorded_78_kph_does_not_prepare_77_from_float_error(self):
    status = tick(armed(), 2200, stock_setpoint_kph=77.99999771118165,
                  requested_accel_ms2=0., target_setpoint_kph=78., payload=stock_frame(78))
    self.assertEqual(status['candidate_setpoint_kph'], 78)
    self.assertEqual(decode_stock(bytes.fromhex(status['candidate']['data_hex'])).setpoint_kph, 78)
    self.assertFalse(status['tx_allowed'])

  def test_setpoint_precision_does_not_raise_measured_speed_or_real_targets(self):
    below = T9RvvAdapter().update(BASE, inputs(BASE, speed_kph=39.99999))
    self.assertEqual(below.reason, 'speed_below_rvv_envelope')
    target = T9RvvAdapter().update(BASE, inputs(BASE, requested_accel_ms2=0., target_setpoint_kph=77.99999))
    self.assertEqual(target.target_setpoint_kph, 77)
    for stock in (39.999, 140.001):
      with self.subTest(stock=stock):
        self.assertEqual(T9RvvAdapter().update(BASE, inputs(BASE, stock_setpoint_kph=stock)).reason,
                         'setpoint_out_of_range')
        self.assertEqual(follow(T9RvvFollowing(), 0, stock_setpoint_kph=stock).reason, 'outside_rvv_envelope')

  def test_140_kph_ceiling_for_adapter_and_following_observer(self):
    accepted = T9RvvAdapter().update(BASE, inputs(BASE, stock_setpoint_kph=140.,
      speed_kph=140., requested_accel_ms2=0.))
    self.assertEqual((accepted.proposed_setpoint_kph, accepted.target_setpoint_kph), (140, 140))
    rejected = T9RvvAdapter().update(BASE, inputs(BASE, stock_setpoint_kph=140.01))
    self.assertEqual(rejected.reason, 'setpoint_out_of_range')
    proposed = follow(T9RvvFollowing(), 0, speed_kph=140., stock_setpoint_kph=140.,
      lead=lead(BASE, speed_ms=140/3.6, distance_m=100.))
    self.assertEqual(proposed.reason, 'stock_setpoint_sufficient')
    self.assertIsNone(proposed.target_kph)
    self.assertFalse(proposed.tx_allowed)

    reduced = follow(T9RvvFollowing(), 0, speed_kph=140., stock_setpoint_kph=140.,
      lead=lead(BASE, speed_ms=140/3.6, distance_m=5. + 2 * (140/3.6) - 1.))
    self.assertEqual(reduced.reason, 'vision_following_candidate')
    self.assertEqual(reduced.target_kph, 139)

  def test_recorded_frame_and_all_bounded_replacements(self):
    recorded = bytes.fromhex('32000014224282af')
    self.assertEqual(decode_stock(recorded).setpoint_kph, 130)
    for counter in range(16):
      original = stock_frame(140, counter=counter)
      for target in range(40, 141):
        output = replacement_payload(original, target)
        decoded = decode_stock(output)
        self.assertEqual((decoded.setpoint_kph, decoded.counter, decoded.mode, decoded.activation), (target, counter, 1, True))
        self.assertEqual(output[1:6] + output[7:], original[1:6] + original[7:])
        self.assertEqual(output[0] & 0xCF, original[0] & 0xCF)

  def test_each_setpoint_bit_error_is_detected(self):
    for value in range(256):
      data = stock_frame(value)
      decode_stock(data)
      for bit in range(8):
        corrupted = bytearray(data); corrupted[6] ^= 1 << bit
        with self.assertRaisesRegex(ValueError, 'parity'):
          decode_stock(corrupted)

  def test_never_activate_resume_limit_or_exceed_driver(self):
    for data in (stock_frame(active=False), stock_frame(mode=2), stock_frame(141), stock_frame(200), stock_frame(255), b''):
      with self.assertRaises(ValueError):
        replacement_payload(data, 80)
    for target in (True, 39, 91, 130, 255, 80.1, math.nan):
      with self.assertRaises(ValueError):
        replacement_payload(stock_frame(90), target)


class TestRvvControl(unittest.TestCase):
  def test_accelerator_override_is_not_physical_rearm(self):
    control = armed()
    self.assertEqual(tick(control, 2500)['phase'], 'prepared')
    self.assertEqual(tick(control, 2510, brake_pressed=True)['reason'], 'brake_pressed')
    status = tick(control, 2600, requested=False, cruise_active=False, gas_pressed=True,
                  observed_active=True, engine_state=1)
    self.assertFalse(status['physical_off_observed'])
    self.assertEqual(status['engine_state_raw'], 1)
    self.assertEqual(tick(control, 2700)['reason'], 'brake_pressed')
    self.assertTrue(tick(control, 2800, requested=False, cruise_active=False)['physical_off_observed'])
    self.assertEqual(tick(control, 2900)['phase'], 'prepared')

  def test_engine_inactive_alone_cannot_arm_on_start(self):
    for raw_state in (0, 1, 3):
      with self.subTest(raw_state=raw_state):
        control = T9RvvControl()
        for ms in range(0, 2401, 10):
          status = tick(control, ms, requested=False, cruise_active=False, gas_pressed=True,
                        observed_active=True, engine_state=raw_state)
          self.assertFalse(status['physical_off_observed'])
        self.assertEqual(tick(control, 2500)['reason'], 'physical_rvv_rearm_required')

  def test_fresh_physical_off_required_on_start_and_rearm(self):
    control = T9RvvControl()
    for ms in range(0, 2401, 10):
      status = tick(control, ms, requested=False)
    status = tick(control, 2500)
    self.assertEqual(status['reason'], 'physical_rvv_rearm_required')
    tick(control, 2510, requested=False)
    self.assertEqual(tick(control, 2520)['phase'], 'blocked')
    tick(control, 2600, requested=False, cruise_active=False)
    self.assertEqual(tick(control, 2700)['phase'], 'prepared')

  def test_one_candidate_per_stock_frame_and_bounded_steps(self):
    control = armed()
    candidates = []
    for ms in range(2200, 4201, 10):
      status = tick(control, ms)
      self.assertFalse(status['tx_allowed'])
      self.assertEqual(status['candidate_setpoint_kph'], 90 - (ms - 2200) // 500)
      if control.candidate:
        candidates.append(control.candidate)
    self.assertEqual(len(candidates), 21)
    self.assertEqual(len({c['stock_rx_ns'] for c in candidates}), 21)
    self.assertTrue(all((c['address'], c['bus']) == (0x50E, 0) for c in candidates))

  def test_driver_lower_ceiling_and_no_automatic_increase(self):
    control = armed()
    tick(control, 2200)
    self.assertEqual(tick(control, 2300, stock_setpoint_kph=75., requested_accel_ms2=0.)['candidate_setpoint_kph'], 75)
    for ms in range(2310, 3301, 10):
      status = tick(control, ms, stock_setpoint_kph=75. if ms < 2400 else 90., requested_accel_ms2=1.)
    self.assertEqual(status['candidate_setpoint_kph'], 75)

  def test_interruptions_latch_without_software_rearm(self):
    for change, reason in (({'brake_pressed': True}, 'brake_pressed'), ({'gas_pressed': True}, 'driver_accelerator'),
                           ({'cancel': True}, 'driver_cancel'), ({'can_valid': False}, 'can_invalid'),
                           ({'requested': False}, 'longitudinal_request_removed'),
                           ({'requested_accel_ms2': -.31}, 'service_brake_required')):
      with self.subTest(change=change):
        control = armed(); tick(control, 2200)
        self.assertEqual(tick(control, 2210, **change)['reason'], reason)
        tick(control, 2220, requested=False)
        self.assertEqual(tick(control, 2230)['reason'], reason)
        self.assertIsNone(control.candidate)

  def test_wrong_bus_and_tx_echo_distinguished(self):
    control = armed()
    self.assertEqual(tick(control, 2200)['phase'], 'prepared')  # Echo 128 ignored.
    self.assertEqual(tick(control, 2300, unseparated=True)['reason'], 'bus_separation_not_observed')
    self.assertIsNone(control.candidate)

  def test_bad_checksum_duplicate_counter_and_stale_template(self):
    for data, reason in ((b'bad', 'stock_dlc_invalid'),
                         (bytes.fromhex('00000014224282af'), 'stock_parity_invalid'),
                         (stock_frame(counter=12), 'stock_counter_not_progressing')):
      control = armed(); tick(control, 2200)
      self.assertEqual(tick(control, 2300, payload=data)['reason'], reason)
      self.assertIsNone(control.candidate)
    control = armed(); tick(control, 2200)
    now = BASE + 2210 * 1_000_000
    self.assertEqual(control.update(now + 300_000_000, inputs(now + 300_000_000))['reason'], 'stock_template_stale')

  def test_old_template_not_queued_and_control_timeout(self):
    control = armed(); tick(control, 2200, requested=False)
    status = tick(control, 2260)
    self.assertEqual(status['reason'], 'stock_frame_too_old_to_replace')
    self.assertIsNone(control.candidate)
    control = armed(); tick(control, 2200)
    self.assertEqual(tick(control, 2400)['reason'], 'control_update_timeout')

  def test_following_to_payload_end_to_end(self):
    control = armed(); planner = T9RvvFollowing()
    for ms in range(2200, 3201, 10):
      now = BASE + ms * 1_000_000
      # Observe source frames through the same path as CarInterface.
      engine = bytes.fromhex('0000000008000000')
      frames = [(0x208, engine, 0)]
      if ms % 100 == 0:
        frames.append((0x50E, stock_frame(counter=(ms // 50) & 15), 2))
      control.observe([(now, frames)])
      decision = follow(planner, ms)
      status = control.update_following(now, inputs(now), decision)
      self.assertEqual(status['candidate_setpoint_kph'], max(79, 90 - (ms - 2200) // 100))
    self.assertEqual(decode_stock(bytes.fromhex(control.candidate['data_hex'])).setpoint_kph, 80)
    now = BASE + 3210 * 1_000_000
    lost = follow(planner, 3210, lead=lead(now, present=False))
    self.assertEqual(control.update_following(now, inputs(now), lost)['reason'], 'lead_lost')
    self.assertIsNone(control.candidate)

  def test_explicit_target_is_bounded_by_driver_and_finite(self):
    now = BASE
    for target in (39., 91., math.nan, math.inf):
      self.assertEqual(T9RvvAdapter().update(now, inputs(now, target_setpoint_kph=target)).phase, 'blocked')

  def test_following_slows_then_recovers_gradually_to_driver_ceiling(self):
    control, planner = armed(), T9RvvFollowing()
    for ms in range(2200, 3701, 10):
      status = following_tick(control, planner, ms)
    self.assertEqual(status['candidate_setpoint_kph'], 79)
    last_value, last_change = 79, 3700
    for ms in range(3710, 6201, 10):
      status = following_tick(control, planner, ms, lead_changes={'distance_m': 100.})
      value = status['candidate_setpoint_kph']
      if ms < 4710:
        self.assertEqual(value, 79)
      if value != last_value:
        self.assertEqual(value - last_value, 1)
        self.assertGreaterEqual(ms - last_change, 500)
        last_change = ms
      last_value = value
      self.assertLessEqual(value, 82)
      self.assertTrue(status['candidate_increase_permitted'])
      self.assertFalse(status['tx_allowed'])
      if control.candidate is not None:
        stock = decode_stock(bytes.fromhex(control.candidate['data_hex']))
        self.assertLessEqual(stock.setpoint_kph, 82)
        self.assertEqual(stock.counter, (ms // 50) & 15)
        self.assertTrue(stock.activation)

  def test_driver_cancel_brake_and_gas_interrupt_recovery_without_restart(self):
    for changes, reason in (({'cancel': True}, 'driver_cancel'),
                            ({'brake_pressed': True}, 'brake_pressed'),
                            ({'gas_pressed': True}, 'driver_accelerator')):
      with self.subTest(reason=reason):
        control, planner = armed(), T9RvvFollowing()
        for ms in range(2200, 3701, 10):
          following_tick(control, planner, ms)
        for ms in range(3710, 5201, 10):
          status = following_tick(control, planner, ms, lead_changes={'distance_m': 100.})
        self.assertGreaterEqual(status['candidate_setpoint_kph'], 80)
        status = following_tick(control, planner, 5210, lead_changes={'distance_m': 100.}, **changes)
        self.assertEqual(status['reason'], reason)
        self.assertIsNone(control.candidate)
        self.assertFalse(status['candidate_increase_permitted'])
        self.assertEqual(following_tick(control, planner, 5220)['reason'], reason)

  def test_driver_lower_ceiling_takes_priority_during_recovery(self):
    control, planner = armed(), T9RvvFollowing()
    for ms in range(2200, 3701, 10):
      following_tick(control, planner, ms)
    for ms in range(3710, 5300, 10):
      status = following_tick(control, planner, ms, lead_changes={'distance_m': 100.})
    self.assertGreaterEqual(status['candidate_setpoint_kph'], 80)
    status = following_tick(control, planner, 5300, stock_setpoint_kph=79., lead_changes={'distance_m': 100.})
    self.assertEqual(status['candidate_setpoint_kph'], 79)
    self.assertEqual(decode_stock(bytes.fromhex(control.candidate['data_hex'])).setpoint_kph, 79)
    self.assertFalse(status['tx_allowed'])

  def test_loss_of_lead_or_its_data_never_restores_stock_target(self):
    now = BASE + 3710 * 1_000_000
    for changes, reason in (({'present': False}, 'lead_lost'),
                            ({'probability': .74}, 'lead_low_confidence'),
                            ({'model_ns': now - 500_000_001}, 'lead_invalid_or_stale'),
                            ({'valid': False}, 'lead_invalid_or_stale')):
      with self.subTest(reason=reason):
        control, planner = armed(), T9RvvFollowing()
        for ms in range(2200, 3701, 10):
          following_tick(control, planner, ms)
        status = following_tick(control, planner, 3710, lead_changes=changes)
        self.assertEqual(status['reason'], reason)
        self.assertIsNone(control.candidate)
        self.assertFalse(status['candidate_increase_permitted'])
        self.assertEqual(following_tick(control, planner, 3720)['reason'], reason)

  def test_timeout_during_following_is_not_a_recovery_request(self):
    control, planner = armed(), T9RvvFollowing()
    for ms in range(2200, 3701, 10):
      following_tick(control, planner, ms)
    status = following_tick(control, planner, 4000, lead_changes={'distance_m': 100.})
    self.assertEqual(status['reason'], 'following_update_timeout')
    self.assertIsNone(control.candidate)

  def test_physical_rvv_restart_clears_previous_following_interruptions(self):
    cases = [({'brake_pressed': True}, {}), ({'gas_pressed': True}, {}),
             ({'cancel': True}, {}), ({'can_valid': False}, {}),
             ({}, {'present': False}), ({}, {'probability': .74}),
             ({}, {'distance_m': 10.}), ({}, {'valid': False})]
    for input_changes, lead_changes in cases:
      with self.subTest(inputs=input_changes, lead=lead_changes):
        control, planner = armed(), T9RvvFollowing()
        for ms in range(2200, 3701, 10):
          following_tick(control, planner, ms)
        status = following_tick(control, planner, 3710, lead_changes=lead_changes, **input_changes)
        self.assertEqual(status['phase'], 'blocked')
        for ms in range(3720, 3801, 10):
          tick(control, ms, requested=False, cruise_active=False)
          follow(planner, ms, active=False)
        self.assertTrue(control.status['physical_off_observed'])
        status = following_tick(control, planner, 3900)
        self.assertEqual(status['phase'], 'prepared')
        self.assertFalse(status['rearm_required'])
        self.assertIsNotNone(control.candidate)
        self.assertFalse(status['tx_allowed'])

  def test_fault_label_cannot_authorize_recovery_even_with_a_target(self):
    control = armed()
    tick(control, 2200)
    now = BASE + 2210 * 1_000_000
    decision = replace(follow(T9RvvFollowing(), 2210), driver_intervention_required=True)
    status = control.update_following(now, inputs(now), decision)
    self.assertEqual(status['reason'], 'following_decision_inconsistent')
    self.assertIsNone(control.candidate)

  def test_fresh_can_does_not_make_an_old_following_decision_fresh(self):
    control = armed()
    tick(control, 2200, requested=False)
    old = follow(T9RvvFollowing(), 2000)
    now = BASE + 2200 * 1_000_000
    self.assertEqual(control.update_following(now, inputs(now), old)['reason'], 'following_decision_stale')
    self.assertIsNone(control.candidate)


class TestAnticipatedFollowing(unittest.TestCase):
  def test_large_closing_speed_reduces_at_a_still_comfortable_gap(self):
    now = BASE
    result = follow(T9RvvFollowing(), 0, speed_kph=130., stock_setpoint_kph=130.,
      lead=lead(now, distance_m=150., speed_ms=90/3.6, relative_speed_ms=-40/3.6))
    self.assertEqual(result.reason, 'vision_following_candidate')
    self.assertLess(result.target_kph, 130)
    self.assertGreaterEqual(result.target_kph, 40)
    self.assertTrue(result.anticipation_only)
    self.assertIsNone(result.requested_accel_ms2)
    self.assertFalse(result.rearm_required)

  def test_lower_target_cannot_recover_while_still_closing(self):
    planner = T9RvvFollowing()
    first = follow(planner, 0, speed_kph=130., stock_setpoint_kph=130.,
      lead=lead(BASE, distance_m=100., speed_ms=110/3.6, relative_speed_ms=-20/3.6))
    for ms in range(10, 3011, 10):
      now = BASE + ms * 1_000_000
      result = follow(planner, ms, speed_kph=130., stock_setpoint_kph=130.,
        lead=lead(now, distance_m=150., speed_ms=125/3.6, relative_speed_ms=-5/3.6))
      self.assertEqual(result.target_kph, first.target_kph)
      self.assertTrue(result.recovery_waiting)
    for ms in range(3020, 4020, 10):
      now = BASE + ms * 1_000_000
      result = follow(planner, ms, speed_kph=130., stock_setpoint_kph=130.,
        lead=lead(now, distance_m=150., speed_ms=130/3.6, relative_speed_ms=0.))
      self.assertEqual(result.target_kph, first.target_kph)
    result = follow(planner, 4020, speed_kph=130., stock_setpoint_kph=130.,
      lead=lead(BASE + 4020_000_000, distance_m=150., speed_ms=130/3.6, relative_speed_ms=0.))
    self.assertEqual(result.target_kph, 130)

  def messages(self, now, **changes):
    return observer_messages(now, speed_kph=130., stock_setpoint_kph=130., dRel=150.,
      vLead=90/3.6, vRel=-40/3.6, **changes)

  def test_early_acquisition_needs_new_models_before_any_wire_request(self):
    observer = T9RvvFollowingObserver()
    for ms in range(0, 251, 50):
      now = BASE + ms * 1_000_000
      observer.update(self.messages(now), now)
      payload = rvv_wire.command(observer.decision, now=now, engaged=True)
      observer.command_published(payload)
      self.assertEqual(rvv_wire.WIRE.unpack(payload)[1] > 0, ms >= 200)
      self.assertEqual(observer.request_episode_started, ms >= 200)

  def test_transient_marginal_detection_cannot_commit_an_early_episode(self):
    observer = T9RvvFollowingObserver()
    observer.update(self.messages(BASE, modelProb=.751), BASE)
    payload = rvv_wire.command(observer.decision, now=BASE, engaged=True)
    observer.command_published(payload)
    self.assertFalse(observer.request_episode_started)
    now = BASE + 50_000_000
    observer.update(self.messages(now, modelProb=.74), now)
    self.assertFalse(observer.decision.rearm_required)
    self.assertFalse(observer.request_episode_started)
    for ms in range(100, 301, 50):
      now = BASE + ms * 1_000_000
      sm = self.messages(now)
      sm['radarState'].mdMonoTime = BASE + 100_000_000
      observer.update(sm, now)
      self.assertIsNone(observer.decision.target_kph)


class TestRvvFollowing(unittest.TestCase):
  def test_unchanged_stock_target_does_not_start_a_takeover(self):
    planner = T9RvvFollowing()
    decision = follow(planner, 0, lead=lead(BASE, distance_m=100.))
    self.assertEqual(decision.reason, 'stock_setpoint_sufficient')
    self.assertIsNone(decision.target_kph)
    self.assertFalse(decision.rearm_required)
    # Recorded trial: an unchanged setpoint followed by a confidence dip
    # incorrectly latched a takeover even though no reduction was requested.
    decision = follow(planner, 10, lead=lead(BASE + 10_000_000, probability=.74))
    self.assertEqual(decision.reason, 'waiting_for_confident_lead')
    self.assertFalse(decision.rearm_required)
    self.assertEqual(follow(planner, 20).target_kph, 79)

  def test_manual_stock_reduction_is_not_an_adaptive_braking_request(self):
    decision = follow(T9RvvFollowing(), 0, stock_setpoint_kph=70.,
                      lead=lead(BASE, distance_m=100.))
    self.assertEqual(decision.reason, 'stock_setpoint_sufficient')
    self.assertIsNone(decision.target_kph)
    self.assertFalse(decision.driver_intervention_required)

  def test_real_reduction_then_recovery_keeps_loss_protection(self):
    planner = T9RvvFollowing()
    self.assertEqual(follow(planner, 0).target_kph, 79)
    decision = follow(planner, 10, lead=lead(BASE + 10_000_000, distance_m=100.))
    self.assertEqual(decision.target_kph, 79)
    self.assertTrue(decision.recovery_waiting)
    self.assertEqual(decision.reason, 'vision_following_candidate')
    decision = follow(planner, 20, lead=lead(BASE + 20_000_000, probability=.74))
    self.assertEqual(decision.reason, 'lead_low_confidence')
    self.assertTrue(decision.rearm_required)

  def test_first_detection_waits_for_confidence_without_rearming(self):
    planner = T9RvvFollowing()
    result = follow(planner, 0, lead=lead(BASE, probability=.6))
    self.assertEqual(result.reason, 'waiting_for_confident_lead')
    self.assertFalse(result.rearm_required)
    self.assertIsNone(result.target_kph)
    self.assertEqual(follow(planner, 10).target_kph, 79)

  def test_valid_lead_receding_allows_a_higher_target_bounded_by_driver(self):
    planner = T9RvvFollowing()
    result = follow(planner, 0)
    self.assertEqual(result.target_kph, 79)
    self.assertIsNone(result.requested_accel_ms2)
    self.assertFalse(result.tx_allowed)
    now = BASE + 10_000_000
    for ms in range(10, 1011, 10):
      now = BASE + ms * 1_000_000
      faster = follow(planner, ms, lead=lead(now, distance_m=100.))
    self.assertEqual(faster.target_kph, 90)
    self.assertTrue(faster.candidate_increase_permitted)
    self.assertFalse(faster.tx_allowed)

  def test_lost_low_confidence_and_stale_model_need_physical_rearm(self):
    now = BASE + 10_000_000
    for observation, reason in ((lead(now, present=False), 'lead_lost'),
                                (lead(now, probability=.74), 'lead_low_confidence'),
                                (lead(now, model_ns=1), 'lead_invalid_or_stale'),
                                (lead(now, rx_ns=now+1), 'lead_invalid_or_stale')):
      planner = T9RvvFollowing(); follow(planner, 0)
      self.assertEqual(follow(planner, 10, lead=observation).reason, reason)
      self.assertIsNone(follow(planner, 20).target_kph)
      follow(planner, 30, active=False)
      self.assertEqual(follow(planner, 40).target_kph, 79)

  def test_critical_or_invalid_lead_still_requests_driver(self):
    cases = [({'distance_m': 10.}, 'fixed_cruise_cannot_handle_critical_lead'),
             ({'distance_m': 35., 'speed_ms': 10., 'relative_speed_ms': 10. - 80/3.6},
              'fixed_cruise_cannot_handle_critical_lead'),
             ({'distance_m': math.nan}, 'nonfinite_lead'),
             ({'probability': 1.1}, 'lead_probability_invalid'),
             ({'relative_speed_ms': 10.}, 'lead_speed_inconsistent')]
    for change, reason in cases:
      result = follow(T9RvvFollowing(), 0, lead=lead(BASE, **change))
      self.assertEqual(result.reason, reason)
      self.assertTrue(result.driver_intervention_required)
      self.assertIsNone(result.requested_accel_ms2)

  def test_recorded_route2d_target_has_no_two_second_arrival_requirement(self):
    # Segment 5, following[51], monotonic 2404.232601773 s. The old
    # (96 - 98.49) / 7.2 proxy rejected this target as -0.346 m/s².
    now = 2404232601773
    observation = RvvLead(True, 62.40969467163086, 26.299819946289062,
                         -1.0491962432861328, .9936181902885437,
                         2404223442883, 2404211746675, True, False)
    result = T9RvvFollowing().update(now, active=True, car_valid=True, car_ns=now,
      speed_kph=98.49, stock_setpoint_kph=99., calibrated=True, calibration_ns=now, lead=observation)
    self.assertEqual(result.reason, 'vision_following_candidate')
    self.assertEqual(result.target_kph, 93)
    self.assertIsNone(result.requested_accel_ms2)
    self.assertFalse(result.rearm_required)
    self.assertEqual(rvv_wire.WIRE.unpack(rvv_wire.command(result, now=now, engaged=True, split_axes=True))[1], 93)

  def test_speed_difference_alone_does_not_impose_a_two_second_deadline(self):
    for distance, expected in ((65., 94), (64., 93), (61., 91), (50., 83)):
      with self.subTest(distance=distance):
        result = follow(T9RvvFollowing(), 0, speed_kph=100., stock_setpoint_kph=100.,
          lead=lead(BASE, distance_m=distance, speed_ms=95/3.6, relative_speed_ms=-5/3.6))
        self.assertEqual(result.target_kph, expected)
        self.assertIsNone(result.requested_accel_ms2)
        self.assertFalse(result.rearm_required)

  def test_calibration_inputs_and_clock(self):
    cases = [({'calibrated': False}, 'calibration_invalid_or_stale'),
             ({'calibration_ns': 0}, 'calibration_invalid_or_stale'),
             ({'car_ns': 0}, 'car_state_invalid_or_stale'),
             ({'car_valid': False}, 'car_state_invalid_or_stale'),
             ({'stock_setpoint_kph': 140.01}, 'outside_rvv_envelope'),
             ({'speed_kph': math.nan}, 'nonfinite_vehicle_input')]
    for change, reason in cases:
      self.assertEqual(follow(T9RvvFollowing(), 0, **change).reason, reason)
    planner = T9RvvFollowing(); follow(planner, 10)
    self.assertEqual(follow(planner, 0).reason, 'invalid_clock')
    planner = T9RvvFollowing(); follow(planner, 0)
    self.assertEqual(follow(planner, 300).reason, 'following_update_timeout')


class TestNativeIntegration(unittest.TestCase):
  def test_carcontroller_never_emits_rvv_even_with_long_active(self):
    from opendbc.car.psa.interface import CarInterface
    from opendbc.car.psa.values import CAR
    for lateral in ('0', '1'):
      with patch.dict(os.environ, {'PSA_T9_LATERAL_TEST': lateral}):
        cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
      ci = CarInterface(cp)
      self.assertFalse(cp.openpilotLongitudinalControl)
      state = structs.CarState(canValid=True, vEgoRaw=80/3.6, gearShifter=structs.CarState.GearShifter.drive)
      state.cruiseState.enabled = state.cruiseState.available = True
      state.cruiseState.speed = 90/3.6
      cs = SimpleNamespace(out=state, t9_pedal_valid=True, t9_shadow_safety_rx_nanos=BASE,
                           t9_shadow_rvv_rx_nanos=BASE, t9_shadow_latest_rx_nanos=BASE)
      cc = structs.CarControl(enabled=True, longActive=True)
      cc.actuators.accel = -.2
      with patch('opendbc.car.psa.rvv_control.carlog.info') as logger:
        _, sends = ci.CC.update(cc, cs, BASE)
      self.assertFalse(any(address == 0x50E for address, _, _ in sends))
      snapshot = json.loads(logger.call_args_list[0].args[0].removeprefix('psa_t9_rvv '))
      self.assertFalse(snapshot['tx_allowed'])

  def test_native_observer_uses_model_timestamp_without_mutating_messages(self):
    now = BASE
    car = structs.CarState(canValid=True, vEgoRaw=80/3.6)
    car.cruiseState.enabled = True; car.cruiseState.speed = 90/3.6
    native = SimpleNamespace(status=True, dRel=48.4, vLead=80/3.6, vRel=0., modelProb=.95, radar=False)
    messages = {'carState': car, 'radarState': SimpleNamespace(leadOne=native, mdMonoTime=now),
                'liveCalibration': SimpleNamespace(calStatus='calibrated')}
    class SM(dict):
      valid = {key: True for key in messages}
      alive = valid.copy()
      logMonoTime = {key: now for key in messages}
    sm = SM(messages)
    observer = T9RvvFollowingObserver()
    status = observer.update(sm, now)
    self.assertEqual(status['target_kph'], 79)
    self.assertEqual(car.cruiseState.speed, 90/3.6)
    observer.command_published(rvv_wire.command(observer.decision, now=now, engaged=True))
    sm['radarState'].mdMonoTime = 1
    status = observer.update(sm, now+10_000_000)
    self.assertEqual(status['reason'], 'lead_invalid_or_stale')


class TestNativeFollowingRequestEpisodes(unittest.TestCase):
  @staticmethod
  def publish(observer, now, engaged):
    payload = rvv_wire.command(observer.decision, now=now, engaged=engaged)
    observer.command_published(payload)
    return rvv_wire.WIRE.unpack(payload)[1]

  def test_disabled_native_proposals_do_not_acquire_or_latch_lead_loss(self):
    observer = T9RvvFollowingObserver()
    observer.update(observer_messages(BASE), BASE)
    self.assertEqual(observer.decision.target_kph, 79)
    self.assertEqual(self.publish(observer, BASE, False), 0)
    self.assertFalse(observer.request_episode_started)
    now = BASE + 10_000_000
    observer.update(observer_messages(now, status=False), now)
    self.assertEqual(observer.decision.reason, 'waiting_for_lead')
    self.assertFalse(observer.decision.rearm_required)
    self.assertEqual(self.publish(observer, now, False), 0)
    now += 10_000_000
    observer.update(observer_messages(now), now)
    self.assertEqual(self.publish(observer, now, True), 79)
    self.assertTrue(observer.request_episode_started)

  def test_published_episode_keeps_loss_fault_across_native_disable_until_stock_off(self):
    for changes, reason in (({'status': False}, 'lead_lost'),
                            ({'modelProb': .74}, 'lead_low_confidence')):
      with self.subTest(reason=reason):
        observer = T9RvvFollowingObserver()
        observer.update(observer_messages(BASE), BASE)
        self.assertEqual(self.publish(observer, BASE, True), 79)
        now = BASE + 10_000_000
        observer.update(observer_messages(now, **changes), now)
        self.assertEqual(observer.decision.reason, reason)
        self.assertTrue(observer.decision.rearm_required)
        self.assertEqual(self.publish(observer, now, False), 0)
        now += 10_000_000
        observer.update(observer_messages(now), now)
        self.assertEqual(observer.decision.reason, reason)
        self.assertEqual(self.publish(observer, now, True), 0)
        now += 10_000_000
        observer.update(observer_messages(now, active=False), now)
        self.assertEqual(observer.decision.reason, 'stock_rvv_inactive')
        self.assertFalse(observer.request_episode_started)
        self.publish(observer, now, False)
        now += 10_000_000
        observer.update(observer_messages(now), now)
        self.assertEqual(self.publish(observer, now, True), 79)

  def test_current_critical_lead_still_blocks_first_enabled_request(self):
    observer = T9RvvFollowingObserver()
    for i, engaged in enumerate((False, True)):
      now = BASE + i * 10_000_000
      observer.update(observer_messages(now, dRel=10.), now)
      self.assertEqual(observer.decision.reason, 'fixed_cruise_cannot_handle_critical_lead')
      self.assertTrue(observer.decision.driver_intervention_required)
      self.assertTrue(rvv_wire.blocked(rvv_wire.command(observer.decision, now=now, engaged=engaged), now))
      self.assertEqual(self.publish(observer, now, engaged), 0)
      self.assertFalse(observer.request_episode_started)
    now += 10_000_000
    observer.update(observer_messages(now), now)
    self.assertEqual(self.publish(observer, now, True), 79)

  def test_unpublished_or_ineligible_commands_cannot_commit_a_proposal(self):
    for kind in ('unpublished', 'disabled', 'stale', 'different_target', 'malformed'):
      with self.subTest(kind=kind):
        observer = T9RvvFollowingObserver()
        observer.update(observer_messages(BASE), BASE)
        decision = observer.decision
        payload = rvv_wire.command(decision, now=BASE, engaged=True)
        if kind == 'disabled':
          payload = rvv_wire.command(decision, now=BASE, engaged=False)
        elif kind == 'stale':
          payload = rvv_wire.command(decision, now=BASE+100_000_001, engaged=True)
        elif kind == 'different_target':
          payload = rvv_wire.command(replace(decision, target_kph=78), now=BASE, engaged=True)
        elif kind == 'malformed':
          payload = b''
        if kind != 'unpublished':
          observer.command_published(payload)
        self.assertFalse(observer.request_episode_started)
        now = BASE + 10_000_000
        observer.update(observer_messages(now, status=False), now)
        self.assertEqual(observer.decision.reason, 'waiting_for_lead')

  def test_route28_candidates_do_not_acquire_control_while_disabled(self):
    # Segment 15: recorded numerical inputs, with fresh synthetic message
    # timestamps. These are planner regressions, not a replay of the motor.
    samples = (
      (125.37250213623047, 80.60371398925781, 33.736305236816406, -1.0953216552734375, .9574722647666931),
      (125.44999694824219, 78.59554290771484, 34.09931945800781, -.742401123046875, .9719494581222534),
      (125.38499908447265, 79.76372528076172, 33.929256439208984, -.9123992919921875, .96912682056427),
      (125.33999633789062, 79.82166290283203, 33.22151565551758, -1.6165275573730469, .9674521684646606),
      (124.56750640869141, 73.54485321044922, 33.664947509765625, -.9427947998046875, .981626033782959),
    )
    observer = T9RvvFollowingObserver()
    for i, (speed, distance, lead_speed, relative, probability) in enumerate(samples):
      now = BASE + i * 10_000_000
      observer.update(observer_messages(now, speed_kph=speed, stock_setpoint_kph=126.,
        dRel=distance, vLead=lead_speed, vRel=relative, modelProb=probability), now)
      self.assertLessEqual(observer.decision.target_kph, (125, 125, 125, 123, 120)[i])
      self.assertIsNone(observer.decision.requested_accel_ms2)
      self.assertFalse(observer.decision.rearm_required)
      self.assertEqual(self.publish(observer, now, False), 0)
      self.assertFalse(observer.request_episode_started)
    # A past disabled observation cannot poison a later fresh eligible request.
    now += 10_000_000
    observer.update(observer_messages(now), now)
    self.assertEqual(self.publish(observer, now, True), 79)
