import os
import unittest
from unittest.mock import Mock, patch

from openpilot.system.manager.psa_t9_settings import configure_eps_cycle, request_eps_cycle_change


class TestEpsCycleToggle(unittest.TestCase):
  def test_setting_is_saved_before_reboot_for_both_directions(self):
    params = Mock()
    params.get_bool.return_value = False
    for checked in (True, False):
      params.reset_mock()
      self.assertTrue(request_eps_cycle_change(params, checked, started=False, engaged=False))
      writes = [call.args for call in params.put_bool.call_args_list]
      self.assertEqual(writes, [('PsaT9EpsCycleTest', checked), ('DoReboot', True)])

  def test_click_rechecks_offroad_and_engagement_before_any_write(self):
    for blocker in ('started', 'engaged', 'IsOnroad', 'IsEngaged'):
      params = Mock()
      params.get_bool.side_effect = lambda key: key == blocker
      self.assertFalse(request_eps_cycle_change(params, True, started=blocker == 'started', engaged=blocker == 'engaged'))
      params.put_bool.assert_not_called()

  def test_boot_snapshot_overrides_inherited_mode_and_changes_only_when_reloaded(self):
    params = Mock()
    with patch.dict(os.environ, {'PSA_T9_EPS_CYCLE_TEST': '1'}):
      params.get_bool.return_value = False
      self.assertEqual(configure_eps_cycle(params), '0')
      params.get_bool.return_value = True
      self.assertEqual(os.environ['PSA_T9_EPS_CYCLE_TEST'], '0')
      self.assertEqual(configure_eps_cycle(params), '1')
      params.get_bool.assert_called_with('PsaT9EpsCycleTest')


if __name__ == '__main__':
  unittest.main()
