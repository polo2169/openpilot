"""Conservative P87 PID response for the measured steering hunting.

Raw CAN ceilings and driver/EPS authority remain owned by the car controller
and Panda. This profile only reduces the host's feedback/friction response.
The next vehicle test must establish the resulting tracking behavior.
"""
import math

from opendbc.car.psa.values import CarControllerParams, p87_torque_limit

FINGERPRINT = 'PSA_PEUGEOT_5008_P87'
KP_SCALE = .5
KI_SCALE = .5
MAX_FRICTION = .07  # normalized torque: at most 10.5 raw counts at scale 150
LOG_VERSION = 3  # shared marker for the P87 profile in both selected PID versions


def friction_limit(value):
  return min(MAX_FRICTION, max(0., value)) if math.isfinite(value) else 0.


def normalized_limit(speed_kph):
  return p87_torque_limit(speed_kph) / CarControllerParams.STEER_MAX
