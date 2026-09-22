import os
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.psa import rvv_wire
from opendbc.car.psa.eps_cycle import T9EpsCycleGate
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.lka import LkaFeedback, LkaInputs, LkaPhase, T9LkaLifecycle
from opendbc.car.psa.tests import test_lateral_test as integration
from opendbc.car.psa.values import CAR
from opendbc.safety.tests import test_psa_t9_split as split_tests


class TestCycleLifecycle(unittest.TestCase):
  def start(self, enabled=True, activity=True, ready=True):
    self.lka = T9LkaLifecycle(15, cycle_supported=enabled)
    self.now = 1_000_000_000
    for i in range(249):
      d = self.step(eps=1 if i < 4 else 3, activity=activity, ready=ready)
    return d

  def step(self, eps=3, activity=True, ready=True, **changes):
    self.now += 50_000_000
    values = dict(requested=True, pause_supported=True, cycle_ready=ready, driver_activity=activity,
      torque=1., speed_kph=75., vehicle_ready=True, can_valid=True, safety_rx_nanos=self.now,
      feedback=LkaFeedback(stock_state=3, stock_nanos=self.now, eps_state=eps, eps_nanos=self.now))
    values.update(changes)
    return self.lka.update(self.now, LkaInputs(**values))

  def test_off_never_cycles_and_normal_stop_requires_rearm(self):
    self.start(enabled=False)
    for _ in range(400):
      self.assertEqual(self.step().phase, LkaPhase.ACTIVE)
    self.assertTrue(self.step(eps=0).rearm_required)
    self.assertEqual(self.step(eps=1).phase, LkaPhase.BLOCKED)

  def test_zero_marker_release_ack_prepare_and_fresh_reactivation(self):
    d = self.start()
    self.assertEqual(d.phase, LkaPhase.CYCLE_RELEASING)
    self.assertEqual((d.state, d.factor, d.torque_raw), (2, 0, 0))
    for _ in range(3):
      self.assertEqual(self.step().phase, LkaPhase.CYCLE_RELEASING)
    self.assertEqual(self.step(eps=1).phase, LkaPhase.PREPARING)
    self.assertEqual(self.step(eps=1).state, 3)
    self.assertEqual(self.step(eps=1).phase, LkaPhase.WAITING_EPS)
    self.assertEqual(self.step(eps=3).torque_raw, 0)
    self.assertEqual(self.lka.cycle_count, 1)
    self.assertFalse(self.lka.cycling)
    previous = 0
    for _ in range(30):
      d = self.step()
      self.assertLessEqual(abs(d.torque_raw - previous), 1)
      previous = d.torque_raw
    self.assertEqual(d.torque_raw, 15)

  def test_entry_needs_ready_and_actual_eps_activity(self):
    for activity, ready in ((False, True), (True, False), (False, False)):
      self.start(activity=activity, ready=ready)
      for _ in range(100):
        self.assertEqual(self.step(activity=activity, ready=ready).phase, LkaPhase.ACTIVE)
      self.assertEqual(self.step().phase, LkaPhase.CYCLE_ARMING)

  def test_each_interruption_latches_and_cannot_restart_from_recovered_input(self):
    for change in ({'eps': 0}, {'eps': 4}, {'eps': 7}, {'brake_pressed': True},
                   {'driver_torque_raw': 16}, {'driver_torque_raw': -16}, {'ready': False},
                   {'pause_requested': True}, {'requested': False}, {'can_valid': False}, {'speed_kph': 66.}):
      with self.subTest(change=change):
        self.start()
        d = self.step(**change)
        self.assertEqual((d.phase, d.torque_raw), (LkaPhase.BLOCKED, 0))
        self.assertFalse(self.lka.cycling)
        self.assertEqual(self.step(eps=1).phase, LkaPhase.BLOCKED)

  def test_release_timeout_and_late_activation_are_not_retried(self):
    self.start()
    for _ in range(35):
      d = self.step()
    self.assertEqual(d.phase, LkaPhase.BLOCKED)
    self.assertEqual(self.step(eps=1).phase, LkaPhase.BLOCKED)
    self.start()
    self.step(eps=1); self.step(eps=1); self.step(eps=1)
    for _ in range(10):
      d = self.step(eps=1)
    self.assertEqual(d.reason, 'eps_activation_timeout')
    self.assertEqual(self.step(eps=3).phase, LkaPhase.BLOCKED)


class TestCycleModelGate(unittest.TestCase):
  def setUp(self):
    self.gate = T9EpsCycleGate()
    self.car = structs.CarState(vEgo=25., yawRate=0.)
    self.model = NS(laneLineProbs=[0., .9, .9, 0.], laneLines=[NS(y=[v]) for v in [-5., -1.8, 1.8, 5.]],
      meta=NS(laneChangeState='off'), orientationRate=NS(t=[0., .5, 1., 1.5, 2.], z=[0.] * 5), velocity=NS(x=[25.] * 5))

  def update(self, ms, model_ms=None, **kw):
    values = dict(eligible=True, car=self.car, model=self.model, model_valid=True,
                  model_ns=1_000_000_000 + (ms if model_ms is None else model_ms) * 1_000_000)
    values.update(kw)
    return self.gate.update(1_000_000_000 + ms * 1_000_000, **values)

  def test_requires_half_second_of_new_straight_confident_predictions(self):
    for ms in range(0, 500, 50):
      self.assertFalse(self.update(ms))
    self.assertTrue(self.update(500))
    self.model.orientationRate.z[-1] = .02
    self.assertFalse(self.update(550))
    self.model.orientationRate.z[-1] = 0.
    self.assertFalse(self.update(600))

  def test_frozen_stale_invalid_curve_and_driver_data_cannot_authorize(self):
    for ms in range(0, 601, 50):
      self.assertFalse(self.update(ms, model_ms=0))
    for mutation in ('curve', 'lanes', 'blinker', 'driver', 'nan', 'short_prediction'):
      self.setUp()
      if mutation == 'curve': self.car.yawRate = .1
      if mutation == 'lanes': self.model.laneLineProbs[1] = .74
      if mutation == 'blinker': self.car.leftBlinker = True
      if mutation == 'driver': self.car.steeringTorque = 16
      if mutation == 'nan': self.model.orientationRate.z[0] = float('nan')
      if mutation == 'short_prediction': self.model.orientationRate.t[-1] = 1.8
      for ms in range(0, 601, 50):
        self.assertFalse(self.update(ms), mutation)


class TestCyclePanda(unittest.TestCase):
  def setUp(self):
    self.h = split_tests.TestT9SplitSafety()
    self.h.setUp()
    self.h.safety.set_timer(0)
    self.assertEqual(self.h.safety.set_safety_hooks(31, 0x1316), 0)
    self.h.safety.init_tests(); self.h.now = 0
    for _ in range(42): self.h.feed()

  def active_due(self, activity=True):
    self.h.engage()
    for _ in range(240):
      self.h.feed(driver_activity=activity)
      self.assertTrue(self.h.tx(0))

  def release(self):
    self.active_due()
    self.assertTrue(self.h.tx(0, 4, 0))
    self.h.feed()
    self.assertTrue(self.h.tx(0, 2, 0))

  def test_cycle_needs_independent_profile_period_and_driver_proof(self):
    self.h.engage()
    self.assertFalse(self.h.tx(0, 4, 0))
    self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
    self.setUp(); self.active_due(activity=False)
    self.assertFalse(self.h.tx(0, 4, 0))
    self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())

  def test_each_phase_keeps_zero_and_needs_physical_release_then_new_ack(self):
    self.release()
    self.assertTrue(self.h.safety.get_t9_split_lateral_allowed())
    self.assertEqual(self.h.safety.safety_fwd_hook(2, 0x3F2), -1)
    self.h.feed(eps=1)
    self.assertTrue(self.h.tx(0, 3, 0))
    self.h.feed(); self.assertTrue(self.h.tx(0, 3, 0))
    self.h.feed(); self.assertTrue(self.h.tx(0, 4, 1))
    self.h.feed(eps=3)
    self.assertTrue(self.h.tx(1))
    self.assertTrue(self.h.safety.get_t9_split_lateral_allowed())
    self.assertTrue(self.h.safety.get_t9_split_rvv_allowed())

  def test_no_release_cannot_skip_to_prepare_or_active(self):
    for state in (3, 4):
      self.setUp(); self.release(); self.h.feed()
      self.assertFalse(self.h.tx(0, state, 0 if state == 3 else 1))
      self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())

  def test_fault_intervention_timeout_and_normal_stop_never_auto_rearm(self):
    for changes in ({'eps': 0}, {'eps': 4}, {'eps': 7}, {'driver': 16}, {'blinker': 1}, {'speed': 66}):
      self.setUp(); self.release(); self.h.feed(**changes)
      self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
      self.assertTrue(self.h.tx(0, 2, 0))
      self.h.feed(eps=1, driver=0, blinker=0, speed=75)
      self.assertFalse(self.h.tx(0, 3, 0))
    self.setUp(); self.active_due()
    self.assertTrue(self.h.tx(0, 2, 0))  # No preceding cycle marker.
    self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
    self.setUp(); self.release()
    for _ in range(42):
      self.h.feed()
      if self.h.safety.get_t9_split_lateral_allowed(): self.assertTrue(self.h.tx(0, 2, 0))
    self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())

  def test_probe_is_non_actuating_and_capability_is_read_only(self):
    self.assertEqual(self.h.safety.test_t9_rvv_control_request(0, 4), 8)
    self.assertFalse(self.h.safety.get_t9_split_lateral_allowed())
    self.h.safety.set_safety_hooks(31, 0x1317)
    for _ in range(45): self.h.feed(cruise=True)
    self.assertFalse(self.h.tx(0, 3, 0))


class TestCycleIntegration(unittest.TestCase):
  def setUp(self):
    self.h = integration.TestT9LateralIntegration(); self.h.setUp(); self.addCleanup(self.h.doCleanups)
    with patch.dict(os.environ, {'PSA_T9_LATERAL_TEST': '1', 'PSA_T9_RVV_TEST': '1',
                               'PSA_T9_SPLIT_AXES_TEST': '1', 'PSA_T9_EPS_CYCLE_TEST': '1'}):
      cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.h.cp = cp; self.h.ci = CarInterface(cp)
    self.h.safety.set_safety_hooks(31, 0x1316); self.h.safety.init_tests()
    self.assertTrue(rvv_wire.eps_cycle(cp)); self.assertTrue(rvv_wire.split(cp))

  def test_can_handshake_passes_panda_and_preserves_rvv(self):
    phases = []
    sequence = 0
    for ms in range(0, 16601, 10):
      lifecycle = self.h.ci.CC.t9_lateral.lateral
      eps = 1 if ms < 2350 or 14450 <= ms < 14650 else 3
      applied, sends = self.h.step(ms, eps=eps, driver_activity=True, cycle_ready=True,
                                  lat_active=ms >= 2200 and not lifecycle.cycling and not 14650 <= ms <= 14750)
      phases.append(lifecycle.phase)
      if lifecycle.cycling: self.assertEqual(applied.torqueOutputCan, 0)
      if ms >= 2200:
        sequence += 1
        self.assertEqual(self.h.safety.test_t9_rvv_request(0x2100 | 74, sequence), 0)
        self.assertTrue(self.h.safety.get_t9_split_lateral_allowed())
    self.assertIn(LkaPhase.CYCLE_ARMING, phases)
    self.assertIn(LkaPhase.CYCLE_RELEASING, phases)
    self.assertEqual(lifecycle.cycle_count, 1)
    self.assertEqual(applied.torqueOutputCan, 15)

  def test_toggle_off_and_non_split_profiles_cannot_enable_cycle(self):
    for split, mode, expected in (('1', '0', 0x1314), ('0', '1', 0x1312), ('1', 'invalid', 0x1314)):
      with patch.dict(os.environ, {'PSA_T9_LATERAL_TEST': '1', 'PSA_T9_RVV_TEST': '1',
                                 'PSA_T9_SPLIT_AXES_TEST': split, 'PSA_T9_EPS_CYCLE_TEST': mode}):
        cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
      self.assertEqual(cp.safetyConfigs[0].safetyParam, expected)
      self.assertFalse(rvv_wire.eps_cycle(cp))


if __name__ == '__main__':
  unittest.main()
