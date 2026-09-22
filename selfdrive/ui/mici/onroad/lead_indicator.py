"""Select the current primary lead for display, independently of engagement."""
import math
import time

RADAR_TIMEOUT = 0.25
MODEL_TIMEOUT = 0.5


def lead_speed_text(lead, is_metric):
  """Absolute estimated lead speed; a relative closing speed is not a speedometer."""
  if lead is None or not math.isfinite(lead.vLead) or lead.vLead < 0:
    return ''
  speed = lead.vLead * 3.6
  if not is_metric:
    speed /= 1.609344
  # The embedded Inter font includes '~', but not the approximation sign.
  return f'~{speed:.0f} {"km/h" if is_metric else "mph"}'


def visible_lead(sm, started_frame, now=None):
  now = time.monotonic() if now is None else now
  if (sm.recv_frame['radarState'] < started_frame or not sm.valid['radarState'] or not sm.alive['radarState'] or
      not 0 <= now - sm.recv_time['radarState'] <= RADAR_TIMEOUT or
      not 0 <= now - sm.logMonoTime['radarState'] / 1e9 <= RADAR_TIMEOUT):
    return None

  radar = sm['radarState']
  # A newly delivered radar message can still refer to an obsolete model.
  if not 0 <= now - radar.mdMonoTime / 1e9 <= MODEL_TIMEOUT:
    return None

  lead = radar.leadOne
  if (not lead.status or not all(math.isfinite(v) for v in (lead.dRel, lead.yRel, lead.vRel, lead.vLead, lead.modelProb)) or
      lead.dRel <= 0 or lead.vLead < 0 or not 0 <= lead.modelProb <= 1):
    return None
  return lead
