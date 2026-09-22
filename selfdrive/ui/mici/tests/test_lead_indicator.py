import math
from types import SimpleNamespace
import unittest

from openpilot.selfdrive.ui.mici.onroad.lead_indicator import lead_speed_text, visible_lead


class RadarState(dict):
  def __init__(self):
    lead = SimpleNamespace(status=True, dRel=45., yRel=0.3, vRel=-2., vLead=20., modelProb=0.95)
    super().__init__(radarState=SimpleNamespace(mdMonoTime=99_950_000_000, leadOne=lead))
    self.recv_frame = {'radarState': 20}
    self.recv_time = {'radarState': 100.}
    self.logMonoTime = {'radarState': 100_000_000_000}
    self.valid = {'radarState': True}
    self.alive = {'radarState': True}


class TestLeadIndicator(unittest.TestCase):
  def test_estimated_speed_uses_absolute_lead_speed_and_display_units(self):
    lead = RadarState()['radarState'].leadOne
    self.assertEqual(lead_speed_text(lead, True), '~72 km/h')
    self.assertEqual(lead_speed_text(lead, False), '~45 mph')
    lead.vLead = 0.
    self.assertEqual(lead_speed_text(lead, True), '~0 km/h')

  def test_missing_and_invalid_speed_has_no_label(self):
    self.assertEqual(lead_speed_text(None, True), '')
    for speed in [-1., math.nan, math.inf]:
      with self.subTest(speed=speed):
        self.assertEqual(lead_speed_text(SimpleNamespace(vLead=speed), True), '')

  def test_primary_lead_does_not_require_engagement_or_longitudinal_control(self):
    sm = RadarState()
    self.assertIs(visible_lead(sm, 10, now=100.1), sm['radarState'].leadOne)

  def test_lost_lead_disappears(self):
    sm = RadarState()
    self.assertIsNotNone(visible_lead(sm, 10, now=100.1))
    sm['radarState'].leadOne.status = False
    self.assertIsNone(visible_lead(sm, 10, now=100.15))

  def test_expired_lead_disappears_without_a_new_message(self):
    sm = RadarState()
    self.assertIsNotNone(visible_lead(sm, 10, now=100.1))
    self.assertIsNone(visible_lead(sm, 10, now=100.251))

  def test_stale_invalid_and_previous_drive_are_hidden(self):
    for field, value in [('valid', False), ('alive', False), ('recv_frame', 9),
                         ('recv_time', 99.5), ('recv_time', 101.),
                         ('logMonoTime', 99_500_000_000), ('logMonoTime', 101_000_000_000)]:
      with self.subTest(field=field, value=value):
        sm = RadarState()
        getattr(sm, field)['radarState'] = value
        self.assertIsNone(visible_lead(sm, 10, now=100.1))

  def test_new_radar_message_cannot_refresh_old_model(self):
    for timestamp in [0, 99_500_000_000, 101_000_000_000]:
      with self.subTest(timestamp=timestamp):
        sm = RadarState()
        sm['radarState'].mdMonoTime = timestamp
        self.assertIsNone(visible_lead(sm, 10, now=100.1))

  def test_bad_geometry_and_probability_are_hidden(self):
    for field, value in [('dRel', 0.), ('dRel', -1.), ('dRel', math.nan), ('yRel', math.inf),
                         ('vRel', math.nan), ('vLead', -1.), ('vLead', math.inf),
                         ('modelProb', math.nan), ('modelProb', -0.1), ('modelProb', 1.1)]:
      with self.subTest(field=field, value=value):
        sm = RadarState()
        setattr(sm['radarState'].leadOne, field, value)
        self.assertIsNone(visible_lead(sm, 10, now=100.1))

  def test_no_fallback_to_a_second_target(self):
    sm = RadarState()
    sm['radarState'].leadTwo = SimpleNamespace(status=True, dRel=30.)
    sm['radarState'].leadOne.status = False
    self.assertIsNone(visible_lead(sm, 10, now=100.1))


if __name__ == '__main__':
  unittest.main()
