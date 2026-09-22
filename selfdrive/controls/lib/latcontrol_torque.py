import math
from collections import deque

import numpy as np
from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, apply_center_deadzone, get_friction
from opendbc.car.psa.lateral_test import enabled as t9_lateral_enabled
from opendbc.car.psa.values import CAR
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

# The T9 CAN yaw signal has 0.1 deg/s resolution. Road logs at 110-132 km/h
# show a 0.6-0.7 Hz correction cycle while the requested path remains close
# to straight. Preserve the stock schedule through 90 km/h, then reduce the
# feedback terms as speed and one yaw-rate count's lateral acceleration grow.
# Feedforward remains unchanged; the experimental command envelope is +/-15 raw.
T9_INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 25, 30, 33, 36, 40]
T9_KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 1.2, .7, .6, .5, .5]
T9_KI_INTERP = [.15, .15, .15, .15, .15, .15, .15, .15, .12, .10, .08, .06, .06]
T9_YAW_RATE_RESOLUTION_RAD_S = math.radians(.1)

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1


def clip_t9_curvature_to_torque_envelope(v_ego, desired_curvature, max_lateral_accel):
  """Keep T9 curvature inside the configured lateral acceleration envelope."""
  max_curvature = max_lateral_accel / max(v_ego, 1.) ** 2
  clipped = float(np.clip(desired_curvature, -max_curvature, max_curvature))
  return clipped, clipped != desired_curvature


class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.t9_can_response = CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9 and t9_lateral_enabled(CP)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    if self.t9_can_response:
      self.pid = PIDController([T9_INTERP_SPEEDS, T9_KP_INTERP],
                               [T9_INTERP_SPEEDS, T9_KI_INTERP], rate=1/self.dt)
    else:
      self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    if self.t9_can_response:
      # The standard learner fits gravity-corrected pose acceleration. Its
      # estimates cannot replace this fixed, CAN-response development model.
      return
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    if self.t9_can_response:
      # This profile was identified against v * CAN yaw, not against a
      # gravity-corrected pose/vehicle-model acceleration. The CAN convention
      # is opposite openpilot's curvature convention. Keep input and output
      # coordinates matched; do not transplant the unvalidated roll fit.
      measurement = -CS.yawRate * CS.vEgo
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    if self.t9_can_response:
      roll_compensation = 0.0
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement
    if self.t9_can_response:
      # Ignore less than half a decoded yaw-rate count. Without this deadzone,
      # the proportional and friction terms chase quantization at high speed.
      error = apply_center_deadzone(error, .5 * T9_YAW_RATE_RESOLUTION_RAD_S * CS.vEgo)

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
