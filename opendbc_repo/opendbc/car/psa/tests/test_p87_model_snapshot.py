import unittest

from opendbc.car.psa.p87_model_snapshot import P87ModelSnapshot


class TestModelSnapshot(unittest.TestCase):
  def setUp(self):
    self.selector = P87ModelSnapshot()

  def test_recorded_sub_tick_publication_uses_fresh_past_then_new_frame(self):
    past, current = object(), object()
    now = 1_108_260_857_700
    self.selector.select(now-50_000_000, True, now-99_723_593, past)
    self.assertEqual(self.selector.select(now, True, now+152_448, current),
                     (True, now-99_723_593, past))
    self.assertEqual(self.selector.mode, 'deferred_newer_publication')
    self.assertEqual(self.selector.select(now+10_000_000, True, now+152_448, current),
                     (True, now+152_448, current))

  def test_future_without_past_or_with_stale_past_never_authorizes(self):
    self.assertFalse(self.selector.select(1_000_000_000, True, 1_000_152_448, object())[0])
    self.selector.select(1_000_000_000, True, 1_000_000_000, object())
    self.assertFalse(self.selector.select(1_151_000_000, True, 1_151_152_448, object())[0])

  def test_large_future_jump_is_not_hidden_by_valid_cached_data(self):
    self.selector.select(1_000_000_000, True, 1_000_000_000, object())
    self.assertFalse(self.selector.select(1_010_000_000, True, 1_031_000_000, object())[0])

  def test_current_invalid_frame_immediately_replaces_previous_valid_frame(self):
    self.selector.select(1_000_000_000, True, 1_000_000_000, object())
    self.assertFalse(self.selector.select(1_010_000_000, False, 1_010_000_000, object())[0])
    self.assertFalse(self.selector.select(1_020_000_000, True, 1_020_152_448, object())[0])

  def test_reversed_and_missing_timestamps_never_authorize(self):
    self.selector.select(1_000_000_000, True, 1_000_000_000, object())
    for now, timestamp in ((1_010_000_000,0),(1_010_000_000,999_000_000),(999_000_000,1_000_000_000)):
      self.assertFalse(self.selector.select(now, True, timestamp, object())[0])

  def test_deferring_publications_cannot_extend_cached_frame_lifetime(self):
    self.selector.select(1_000_000_000, True, 1_000_000_000, object())
    for ms in range(10,151,10):
      now=1_000_000_000+ms*1_000_000
      self.assertTrue(self.selector.select(now, True, now+152_448, object())[0])
    self.assertFalse(self.selector.select(1_160_000_000, True, 1_160_152_448, object())[0])


if __name__ == '__main__':
  unittest.main()
