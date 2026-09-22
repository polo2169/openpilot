"""Snapshot the offroad EPS test selection before any child process starts."""
import os


def configure_eps_cycle(params):
  value = '1' if params.get_bool('PsaT9EpsCycleTest') else '0'
  os.environ['PSA_T9_EPS_CYCLE_TEST'] = value
  return value


def request_eps_cycle_change(params, enabled, *, started, engaged):
  if started or engaged or params.get_bool('IsOnroad') or params.get_bool('IsEngaged'):
    return False
  params.put_bool('PsaT9EpsCycleTest', bool(enabled), block=True)
  params.put_bool('DoReboot', True, block=True)
  return True
