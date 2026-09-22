import os

from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.lateral_test import SAFETY_PARAM
from opendbc.car.psa.lka import DRIVER_TORQUE_LIMIT, MIN_SPEED_KPH, LkaPhase, fresh
from opendbc.car.psa.rvv_wire import (
  COMBINED_SAFETY_PARAM,
  EPS_CYCLE_SAFETY_PARAM,
  SPLIT_SAFETY_PARAM,
)
from opendbc.car.psa.rvv_wire import SAFETY_PARAM as RVV_SAFETY_PARAM
from opendbc.car.psa.rvv_wire import only as rvv_only
from opendbc.car.psa.rvv_wire import split as split_axes
from opendbc.car.psa.values import CAR

TransmissionType = structs.CarParams.TransmissionType

# Everything outside the known EPS / driver episode remains a common stop.
LOCAL_LATERAL_STOP_REASONS = frozenset({
  'driver_override', 'eps_not_authorized', 'eps_fault', 'eps_feedback_invalid', 'eps_activation_lost',
  'eps_activation_timeout', 'eps_release_timeout', 'eps_not_released',
  'eps_active_before_request', 'stock_lka_not_authorized', 'new_request_required',
  'speed_below_lateral_envelope', 'eps_cycle_interrupted', 'eps_cycle_timeout',
})


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def update(self, can_packets):
    if self.CC.t9_rvv is not None:
      self.CC.t9_rvv.observe(can_packets)
    if self.CC.t9_lateral is not None:
      self.CC.t9_lateral.observe(can_packets)
      self.CS.t9_shadow_now_nanos = max((timestamp for timestamp, _ in can_packets), default=0)
      # All input origins remain in the physical observer. CarState consumes
      # the reunited network after the hardware has split BSI and vehicle.
      can_packets = self.CC.t9_lateral.merge_for_carstate(can_packets)
    if self.CC.read_only:
      self.CC.t9_shadow.observer.update(can_packets)
      self.CS.t9_shadow_now_nanos = max((timestamp for timestamp, _ in can_packets), default=0)
    if rvv_only(self.CP):
      can_packets = [(timestamp, [(address, data, 0) for address, data, bus in frames if bus in (0, 2)])
                     for timestamp, frames in can_packets]
    result = super().update(can_packets)
    if rvv_only(self.CP):
      rvv = self.CC.t9_rvv
      now = self.CS.t9_shadow_now_nanos
      valid = rvv.stock_problem is None and rvv.stock is not None and fresh(now, rvv.stock_ns)
      result.cruiseState.enabled = bool(result.cruiseState.enabled and valid
        and rvv.stock.mode == 1 and rvv.stock.activation and rvv.stock.setpoint_kph < 255)
      result.canValid = bool(result.canValid and valid)
      # An engine 2 -> 1 -> 2 transition is pedal override, not a manual
      # re-engagement. Match the MCU's physical OFF/ON requirement so the
      # native state machine cannot enable into a predictable mismatch.
      if rvv.physical_off_observed(now, can_valid=result.canValid and not result.canTimeout,
          safety_rx_nanos=getattr(self.CS, 't9_shadow_safety_rx_nanos', 0),
          cruise_active=result.cruiseState.enabled):
        self.t9_rvv_native_rearm = True
      active = result.cruiseState.enabled
      if active and not getattr(self, 't9_rvv_was_active', False):
        self.t9_rvv_episode_authorized = getattr(self, 't9_rvv_native_rearm', False)
        self.t9_rvv_native_rearm = False
      if not active:
        self.t9_rvv_episode_authorized = False
      self.t9_rvv_was_active = active
      result.blockPcmEnable = bool(active and not getattr(self, 't9_rvv_episode_authorized', False))
    if self.CC.t9_lateral is not None:
      lateral = self.CC.t9_lateral
      rvv = self.CC.t9_rvv
      now = self.CS.t9_shadow_now_nanos
      stock_valid = rvv.stock_problem is None and rvv.stock is not None and fresh(now, rvv.stock_ns)
      # Match Panda's combined BSI/engine engagement even when these two
      # messages arrive in separate host updates. Engine state 2 alone must
      # not emit an early pcmEnable while the BSI still reports cruise off.
      result.cruiseState.enabled = bool(result.cruiseState.enabled and stock_valid
        and rvv.stock.mode == 1 and rvv.stock.activation and rvv.stock.setpoint_kph < 255)
      result.canValid = bool(result.canValid and stock_valid)
      # Clear the old episode before generating native NO_ENTRY events. A
      # fault during the off interval must not hide the next physical enable
      # edge after recovery. Current EPS/CAN/driver inhibitions remain below.
      physical_off = rvv.physical_off_observed(now,
          can_valid=result.canValid and not result.canTimeout,
          safety_rx_nanos=getattr(self.CS, 't9_shadow_safety_rx_nanos', 0),
          cruise_active=result.cruiseState.enabled)
      if physical_off:
        lateral.rearm_on_physical_cruise_off(now)
      if (split_axes(self.CP) and result.cruiseState.enabled
          and not getattr(self, 't9_rvv_was_active', False) and lateral.lateral.phase == LkaPhase.DISABLED):
        # The physical edge admits each axis once. If steering is unavailable
        # at that edge, latch the refusal even though controlsd never issued
        # a lateral request. A later EPS/driver/stock-LKA recovery alone must
        # not create the authorization that the MCU withheld on this edge.
        feedback = lateral.observer.feedback
        if round(result.vEgoRaw * 3.6, 4) < MIN_SPEED_KPH:
          lateral.lateral._block('speed_below_lateral_envelope')
        elif result.steeringPressed or abs(result.steeringTorque) > DRIVER_TORQUE_LIMIT:
          lateral.lateral._block('driver_override')
        elif result.leftBlinker or result.rightBlinker:
          lateral.lateral._block('new_request_required')
        elif feedback.eps_state not in (1, 2):
          lateral.lateral._block('eps_not_released' if feedback.eps_state == 3 else 'eps_not_authorized')
        elif feedback.stock_state not in (3, 4):
          lateral.lateral._block('stock_lka_not_authorized')
      result.psaEpsCycling = lateral.lateral.cycling
      blocked = lateral.lateral.phase == LkaPhase.BLOCKED
      driver_cut = blocked and lateral.lateral.reason == 'driver_override'
      # Match the immediate raw-effort cut in the host/Panda, including while
      # steeringPressed's debounce is still false. A latched driver cut is a
      # disengagement, not an EPS fault or a resumable lateral override.
      result.steeringDisengage = bool(abs(result.steeringTorque) > DRIVER_TORQUE_LIMIT
                                     or result.steeringPressed or driver_cut)
      # Driver cuts have a persistent native NO_ENTRY event. Keep the physical
      # cruise edge visible so a refused attempt shows "Steering Pressed".
      result.blockPcmEnable = bool(blocked and not driver_cut)
      result.steerFaultTemporary = bool(result.steerFaultTemporary or (blocked and not driver_cut)
                                       or lateral.observer.feedback.eps_state not in (1, 2, 3))
      if split_axes(self.CP):
        pause_session = (lateral.lateral.phase in (LkaPhase.ACTIVE, LkaPhase.PAUSED)
          and result.cruiseState.enabled and result.canValid and not result.canTimeout
          and not result.steerFaultTemporary and not result.steerFaultPermanent
          and lateral.observer.feedback.eps_state == 3 and fresh(now, lateral.observer.feedback.eps_nanos))
        result.psaLateralPaused = bool(pause_session and (lateral.lateral.phase == LkaPhase.PAUSED
          or result.leftBlinker or result.rightBlinker or result.steeringPressed
          or abs(result.steeringTorque) > DRIVER_TORQUE_LIMIT))
        if result.psaLateralPaused:
          result.steeringDisengage = False
        # The common session also requires a real OFF/ON edge. An engine
        # pedal override or an EPS recovery cannot create a new admission.
        if physical_off:
          self.t9_rvv_native_rearm = True
        active = result.cruiseState.enabled
        if active and not getattr(self, 't9_rvv_was_active', False):
          self.t9_rvv_episode_authorized = getattr(self, 't9_rvv_native_rearm', False)
          self.t9_rvv_native_rearm = False
        if not active:
          self.t9_rvv_episode_authorized = False
        self.t9_rvv_was_active = active
        common_lateral_stop = blocked and lateral.lateral.reason not in LOCAL_LATERAL_STOP_REASONS
        result.blockPcmEnable = bool(common_lateral_stop or
          (active and not getattr(self, 't9_rvv_episode_authorized', False)))
    return result

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    ret.dashcamOnly = True

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    if candidate == CAR.PSA_PEUGEOT_308_T9:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
      # Factory captures suggest a torque API; no EPS calibration is supplied.
      # Keep an inert PID configuration for this observation-only interface.
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kpV = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kiV = [0.]
      ret.steerAtStandstill = False
      ret.openpilotLongitudinalControl = False
      ret.autoResumeSng = False

      rvv_requested = os.getenv('PSA_T9_RVV_TEST') == '1'
      lateral_requested = os.getenv('PSA_T9_LATERAL_TEST') == '1'
      if rvv_requested and not lateral_requested and not docs:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa, RVV_SAFETY_PARAM)]
        ret.dashcamOnly = False
        ret.minEnableSpeed = 40. / 3.6
        # Conventional Peugeot cruise still actuates throttle. No service
        # brake or steering command is claimed by this isolated profile.
      elif lateral_requested and not docs:
        profile = COMBINED_SAFETY_PARAM if rvv_requested else SAFETY_PARAM
        if rvv_requested and os.getenv('PSA_T9_SPLIT_AXES_TEST') == '1':
          profile = EPS_CYCLE_SAFETY_PARAM if os.getenv('PSA_T9_EPS_CYCLE_TEST') == '1' else SPLIT_SAFETY_PARAM
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa, profile)]
        ret.dashcamOnly = False
        ret.steerActuatorDelay = 0.15
        ret.steerLimitTimer = 0.4
        ret.minSteerSpeed = 67.1 / 3.6
        ret.minEnableSpeed = (40. if profile in (SPLIT_SAFETY_PARAM, EPS_CYCLE_SAFETY_PARAM) else 67.1) / 3.6
        ret.maxLateralAccel = 0.63
        ret.lateralTuning.init('torque')
        ret.lateralTuning.torque.latAccelFactor = 0.63
        ret.lateralTuning.torque.latAccelOffset = 0.0
        # Active high-speed logs show the 1.4 raw friction prior reinforcing
        # the 0.6-0.7 Hz correction cycle. Keep 0.7 raw to cross rack friction
        # without dominating the quantized CAN yaw feedback near center.
        ret.lateralTuning.torque.friction = 0.07
        ret.lateralTuning.torque.steeringAngleDeadzoneDeg = 0.0

    return ret
