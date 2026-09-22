from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering
from opendbc.car.psa.shadow import T9ShadowController
from opendbc.car.psa.values import CAR, CarControllerParams
from opendbc.car.psa.lateral_test import T9LateralTestController, enabled
from opendbc.car.psa.rvv_control import T9RvvControl
from opendbc.car.psa.rvv_wire import enabled as rvv_enabled, split as split_axes, eps_cycle


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.status = 2
    self.t9_lateral = T9LateralTestController(pause_supported=split_axes(CP), cycle_supported=eps_cycle(CP)) if CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9 and enabled(CP) else None
    self.read_only = CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9 and self.t9_lateral is None
    self.t9_shadow = T9ShadowController() if self.read_only else None
    self.t9_rvv_active_profile = rvv_enabled(CP)
    self.t9_rvv = T9RvvControl() if CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9 else None

  def update(self, CC, CS, now_nanos):
    if self.t9_rvv is not None and not self.t9_rvv_active_profile:
      # Diagnostic candidates are intentionally never appended to can_sends.
      self.t9_rvv.update_from_carcontrol(CC, CS, now_nanos)
    if self.t9_lateral is not None:
      self.frame += 1
      return self.t9_lateral.update(CC, CS, now_nanos)
    if self.read_only:
      if not self.t9_rvv_active_profile:
        self.t9_shadow.update(CC, CS, now_nanos)
      self.frame += 1
      # No CAN output, even if a caller explicitly requests lateral/longitudinal
      # control. Report zero applied actuators rather than echoing the request.
      return structs.CarControl.Actuators(), []

    can_sends = []
    actuators = CC.actuators

    # lateral control
    if self.frame % 5 == 0:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                 CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

      # EPS disengages on steering override, activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      if not CC.latActive:
        self.status = 2
      elif not CS.eps_active and not CS.out.steeringPressed:
        self.status = 2 if self.status == 4 else self.status + 1
      else:
        self.status = 4

      can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_angle, self.status))

      self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
