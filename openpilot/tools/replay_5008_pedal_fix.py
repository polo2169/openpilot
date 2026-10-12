#!/usr/bin/env python3
"""Replay recorded P87 CAN through CarInterface and compiled Panda pedal checks.

No hardware or CAN transmission. Keep interface state across recorder rotations.
Only tolerate a truncated tail when all remaining bytes are zero.
"""
import argparse
import hashlib
import json
import logging
import math
import sys
from collections import Counter, deque
from pathlib import Path


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--opendbc-root', type=Path, required=True)
  parser.add_argument('--openpilot-root', type=Path, required=True)
  parser.add_argument('--captures', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  sys.path[:0] = [str(args.opendbc_root.resolve()), str(args.openpilot_root.resolve())]
  import capnp
  from opendbc.car import structs
  from opendbc.car.can_definitions import CanData
  from opendbc.car.car_helpers import can_fingerprint_p87_startup
  from opendbc.car.carlog import carlog
  from opendbc.car.psa.interface import CarInterface
  from opendbc.car.psa.values import CAR
  from opendbc.safety.tests.libsafety import libsafety_py
  from openpilot.cereal import log
  carlog.setLevel(logging.CRITICAL)
  safety = libsafety_py.libsafety
  sessions = {}
  files = []
  session_id = None
  control = structs.CarControl(latActive=True)
  control.actuators.torque = 1.0

  for path in sorted(args.captures.glob('*.rlog')):
    metadata = json.loads(path.with_suffix('.json').read_text())
    if metadata['session'] != session_id:
      session_id = metadata['session']
      assert session_id not in sessions, 'Recorder sessions must be contiguous'
      cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_5008_P87)
      assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput
      ci = CarInterface(cp, CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_5008_P87))
      sessions[session_id] = {'counts': Counter(), 'max_speed_kph': 0.0}
      latest, timestamps = {}, {}
      recent = deque(maxlen=220)
      next_identification = 0
      assert safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 0x5008) == 0
      safety.init_tests()
    session = sessions[session_id]
    counts = session['counts']
    raw = path.read_bytes()
    consumed = 0
    file_counts = Counter()
    tail = 0
    try:
      for event in log.Event.read_multiple_bytes(raw):
        consumed += len(event.as_builder().to_bytes())
        file_counts['events'] += 1
        if event.which() != 'can' or not event.can:
          continue
        now = int(event.logMonoTime)
        safety.set_timer((now // 1000) & 0xFFFFFFFF)
        frames = [(int(f.address), bytes(f.dat), int(f.src)) for f in event.can]
        file_counts['can_frames'] += len(frames)
        for address, data, bus in frames:
          if bus == 1 and address == 0x2B6:
            latest['acc'] = data[3] & 15
            timestamps['acc'] = now
          if bus != 0:
            continue
          if address == 0x208:
            latest['engine_demand'] = data[3]
            timestamps['engine_demand'] = now
          if address not in (0x208, 0x228):
            continue
          before = bool(safety.get_gas_pressed_prev())
          accepted = bool(safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))
          if address == 0x208:
            counts['engine_demand_packets'] += 1
            counts['engine_demand_changes_panda_pedal'] += bool(safety.get_gas_pressed_prev()) != before
          else:
            counts['pedal_packets'] += 1
            counts['pedal_packets_rejected_by_panda'] += not accepted
            counts['panda_pedal_mismatches'] += bool(safety.get_gas_pressed_prev()) != (data[2] != 0)
            latest['pedal'] = data[2]
            timestamps['pedal'] = now
            fresh = all(0 <= now - timestamps.get(k, 0) <= 150_000_000 for k in ('acc', 'engine_demand'))
            if fresh and latest['acc'] == 4:
              counts['acc_active_pedal_packets'] += 1
              counts['acc_engine_demand_positive_pedal_zero'] += latest['engine_demand'] > 0 and data[2] == 0
              counts['acc_driver_pedal_positive'] += data[2] > 0
        state, _ = ci.update([(now, frames)])
        counts['interface_updates'] += 1
        counts['can_valid_updates'] += bool(state.canValid)
        counts['can_invalid_updates'] += not state.canValid
        assert math.isfinite(state.vEgoRaw) and math.isfinite(state.vEgo), (path.name, now)
        session['max_speed_kph'] = max(session['max_speed_kph'], float(state.vEgoRaw) * 3.6)
        moving = float(state.vEgoRaw) * 3.6 > 5.0
        counts['moving_can_invalid_updates'] += moving and not state.canValid
        recent.append((now, frames, float(state.vEgoRaw) * 3.6))
        if now >= next_identification and len(recent) == recent.maxlen and all(p[2] > 5 for p in recent):
          iterator = iter([[CanData(*f) for f in p[1]] for p in recent])
          candidate, _ = can_fingerprint_p87_startup(lambda wait_for_one=False, source=iterator: [next(source, [])])
          counts['rolling_identification_windows'] += 1
          counts['rolling_identification_failures'] += candidate != CAR.PSA_PEUGEOT_5008_P87
          next_identification = now + 5_000_000_000
        if ci.CS.p87_pedal_valid:
          counts['python_pedal_mismatches'] += bool(state.gasPressed) != (latest['pedal'] != 0)
        _, sends = ci.apply(control.as_reader(), structs.CarControlSP(), now)
        counts['observation_tx_frames'] += len(sends)
    except capnp.KjException:
      assert consumed < len(raw) and not any(raw[consumed:]), (path.name, consumed, 'Nonzero undecodable data')
      tail = len(raw) - consumed
    files.append({'file': path.name, 'session': session_id, 'sha256': hashlib.sha256(raw).hexdigest(),
                  'counts': dict(file_counts), 'trailing_zero_bytes': tail})
    print(path.name, dict(file_counts), 'zero tail:', tail, flush=True)

  totals = Counter()
  for session in sessions.values():
    totals.update(session['counts'])
  for key in ('pedal_packets_rejected_by_panda', 'panda_pedal_mismatches', 'python_pedal_mismatches',
              'engine_demand_changes_panda_pedal', 'moving_can_invalid_updates', 'observation_tx_frames', 'rolling_identification_failures'):
    assert totals[key] == 0, (key, totals[key])
  assert totals['pedal_packets'] > 0 and totals['acc_engine_demand_positive_pedal_zero'] > 0
  assert totals['rolling_identification_windows'] > 0
  report = {'scope': 'offline replay; native Panda library receives pedal/engine-demand only; no physical actuation',
            'files': files, 'sessions': sessions, 'totals': totals, 'passed': True}
  args.output.write_text(json.dumps(report, indent=2) + '\n')
  print(json.dumps(dict(totals), indent=2))


if __name__ == '__main__':
  main()
