"""Versioned RVV command on cereal's existing customReservedRawData0 service.

32 bytes, little endian: magic/version, target km/h (0 = release), flags,
reserved zero, computation/lead/model monotonic timestamps. Logs retain the
entire command. Only the explicit RVV-only and combined profiles consume it.
"""
import struct

from opendbc.car import structs
from opendbc.car.psa.values import CAR

SAFETY_PARAM = 0x1310
COMBINED_SAFETY_PARAM = 0x1312
SPLIT_SAFETY_PARAM = 0x1314
EPS_CYCLE_SAFETY_PARAM = 0x1316
SERVICE = 'customReservedRawData0'
WIRE = struct.Struct('<4sBBHQQQ')


def enabled(CP):
  return (CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9 and not CP.dashcamOnly
          and not CP.openpilotLongitudinalControl and len(CP.safetyConfigs) == 1
          and CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.psa
          and CP.safetyConfigs[0].safetyParam in (SAFETY_PARAM, COMBINED_SAFETY_PARAM, SPLIT_SAFETY_PARAM, EPS_CYCLE_SAFETY_PARAM))


def only(CP):
  return enabled(CP) and CP.safetyConfigs[0].safetyParam == SAFETY_PARAM


def split(CP):
  return enabled(CP) and CP.safetyConfigs[0].safetyParam in (SPLIT_SAFETY_PARAM, EPS_CYCLE_SAFETY_PARAM)


def eps_cycle(CP):
  return split(CP) and CP.safetyConfigs[0].safetyParam == EPS_CYCLE_SAFETY_PARAM


# These failures concern shared vehicle/perception/timing inputs. Unknown
# blocked reasons also fail closed rather than silently becoming axis-local.
LOCAL_RVV_REASONS = frozenset({
  'lead_lost', 'lead_low_confidence', 'lead_probability_invalid', 'lead_geometry_invalid',
  'lead_speed_inconsistent', 'fixed_cruise_cannot_handle_critical_lead',
  'lead_target_below_rvv_minimum', 'deceleration_exceeds_unvalidated_engine_brake_prior',
})


def common_fault(decision):
  return bool(getattr(decision, 'common_fault', False)
              or (decision.rearm_required and decision.reason not in LOCAL_RVV_REASONS))


def command(decision, *, now, engaged, split_axes=False):
  """A disabled or blocked planner never sends a target; zero is explicit."""
  target = decision.target_kph
  permit = (engaged and decision.reason == 'vision_following_candidate'
            and not decision.rearm_required and not decision.driver_intervention_required
            and decision.candidate_increase_permitted and type(target) is int and 40 <= target <= 140
            and 0 < decision.computed_ns <= now and now - decision.computed_ns <= 100_000_000
            and 0 < decision.lead_rx_ns <= now and now - decision.lead_rx_ns <= 250_000_000
            and 0 < decision.model_ns <= now and now - decision.model_ns <= 500_000_000)
  shared_failure = split_axes and common_fault(decision)
  permit = permit and not shared_failure
  flags = int(engaged) | (int(decision.rearm_required) << 1) | (int(shared_failure) << 2)
  return WIRE.pack(b'RVV2' if split_axes else b'RVV1', target if permit else 0, flags, 0, now,
                   decision.lead_rx_ns if permit else 0, decision.model_ns if permit else 0)


def blocked(payload, now):
  if len(payload) != WIRE.size:
    return True
  magic, _, flags, reserved, computed, _, _ = WIRE.unpack(payload)
  return (magic != b'RVV1' or reserved != 0 or flags & ~3 != 0 or flags & 2 != 0
          or computed <= 0 or computed > now or now - computed > 150_000_000)


def split_status(payload, now):
  """Return common fault, local RVV block and actual adaptation request.

  This has the same strict RVV2 boundary as pandad. A malformed candidate
  cannot be mistaken for a legitimate local planner release.
  """
  if len(payload) != WIRE.size:
    return True, False, False
  magic, target, flags, reserved, computed, lead, model = WIRE.unpack(payload)
  invalid = (magic != b'RVV2' or reserved != 0 or flags & ~7 != 0
             or computed <= 0 or computed > now or now - computed > 100_000_000)
  if target:
    invalid |= (not 40 <= target <= 140 or flags != 1
                or not 0 < lead <= now or now - lead > 250_000_000
                or not 0 < model <= now or now - model > 500_000_000)
  else:
    invalid |= lead != 0 or model != 0
  return bool(invalid or flags & 4), bool(flags & 2), bool(target and not invalid and flags == 1)


def split_waiting(payload, now):
  """A fresh enabled session may await a lead without requesting a setpoint.

  Disabled heartbeats and latched releases are never available adaptation.
  """
  common, stopped, requested = split_status(payload, now)
  return not (common or stopped or requested) and WIRE.unpack(payload)[2] == 1
