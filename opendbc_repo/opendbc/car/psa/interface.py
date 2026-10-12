import os

from opendbc.car import structs, get_safety_config
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import CAR, LKAS_LIMITS, PSA_TORQUE_CARS, PSA_5008_LATERAL_PARAM

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def update(self, can_packets):
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_5008_P87:
      for timestamp, frames in can_packets:
        self.CS.p87_can_timestamp = timestamp
        for address, data, bus in frames:
          if address == 0x228 and bus == 0:
            # This parser version does not enforce DBC message lengths. Check
            # the P87 variant explicitly before accepting its pedal value.
            self.CS.p87_pedal_packet_valid = len(data) == 8 and data[2] <= 200 and sum((b >> 4) + (b & 15) for b in data) % 16 == 3
          # The stock camera side must keep producing its native template.
          if address == 0x3F2 and bus == 2 and len(data) == 8:
            self.CS.p87_lka_template = bytes(data)
            self.CS.p87_lka_timestamp = timestamp
    ret, ret_sp = super().update(can_packets)
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_5008_P87:
      ret.canValid &= not self.CS.p87_wheel_speeds_invalid and self.CS.p87_pedal_valid
    return ret, ret_sp

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    #
    ret.dashcamOnly = False

    if candidate in PSA_TORQUE_CARS:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS
      # Measured from route 00000029--0f498d7077 with FIXED_TORQUE_FACTOR: torque request ->
      # steering rate response lag ~140ms. The old 0.376803 was tuned on the variable
      # torque-factor plant, whose ramp made the EPS look ~4x slower than it is.
      ret.steerActuatorDelay = 0.15
      ret.steerLimitTimer = 0.1
      ret.steerAtStandstill = False
      ret.enableBsm = True
    else:
      ret.steerAtStandstill = True
      ret.steerLimitTimer = 0.1
      ret.steerControlType = structs.CarParams.SteerControlType.angle

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False
    # ret.openpilotLongitudinalControl = alpha_long

    if candidate == CAR.PSA_PEUGEOT_5008_P87:
      # CAN recognition does not authorize actuation. This opt-in selects a
      # dedicated, lateral-only Panda profile; the default is observation.
      lateral_test = os.environ.get("PSA_5008_LATERAL_TEST") == "1"
      ret.dashcamOnly = not lateral_test
      ret.openpilotLongitudinalControl = False
      ret.autoResumeSng = False
      ret.safetyConfigs = [get_safety_config(
        structs.CarParams.SafetyModel.psa if lateral_test else structs.CarParams.SafetyModel.noOutput,
        PSA_5008_LATERAL_PARAM if lateral_test else 0,
      )]

    return ret

  @staticmethod
  def _get_params_sp(stock_cp, ret, candidate, fingerprint, car_fw,
                    alpha_long, is_release_sp, docs):
    if candidate in (
      CAR.PSA_PEUGEOT_3008,
      CAR.PSA_CITROEN_C4_SPACETOURER,
      CAR.PSA_PEUGEOT_5008_P87,
    ):
      ret.intelligentCruiseButtonManagementAvailable = False
      ret.pcmCruiseSpeed = True

    return ret
