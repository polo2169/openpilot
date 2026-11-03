#!/usr/bin/env python3
"""Read-only P87 LKA wire audit of original rlogs; never opens CAN sockets."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys


def decode_lka(data):
  if len(data) != 8:
    return {'length': len(data)}
  torque = (data[3] << 3) | (data[4] >> 5)
  return {
    'torque': torque - 2048 if torque >= 1024 else torque,
    'status': (data[4] >> 2) & 7, 'factor': data[5] >> 1,
    'drive': (data[0] >> 6) & 1, 'lxa': data[5] & 1,
    'counter_nibble_dbc': data[1] & 15, 'checksum_nibble_dbc': data[1] >> 4,
    'nibble_sum_mod16': sum((b >> 4) + (b & 15) for b in data) & 15,
    'angle_raw': (data[6] << 6) | (data[7] >> 2),
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--openpilot-root', type=Path, required=True)
  parser.add_argument('--rlogs', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--opendbc-root', type=Path, help='Also replay the compiled native safety RX/TX policy')
  args = parser.parse_args()
  sys.path.insert(0, str(args.openpilot_root.resolve()))
  # cereal imports opendbc.car.structs during initialization. Select the
  # requested dependency before that import, including in editable venvs.
  sys.path.insert(0, str((args.opendbc_root or args.openpilot_root/'opendbc_repo').resolve()))
  import zstandard
  from openpilot.cereal import log

  monitored = {(0x228, 0), (0x348, 0), (0x30D, 0), (0x2F5, 0), (0x495, 0),
               (0x412, 2), (0x452, 1), (0x2B6, 1), (0x32D, 1)}
  safety = packet = None
  if args.opendbc_root:
    from opendbc.car import structs
    from opendbc.safety.tests.libsafety import libsafety_py
    safety, packet = libsafety_py.libsafety, libsafety_py.make_CANPacket
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 0x5008) == 0
    safety.init_tests()

  counts = Counter()
  distributions = defaultdict(lambda: defaultdict(Counter))
  payloads = defaultdict(Counter)
  latest = {}
  control = {'latActive': False}
  state = {}
  panda = {}
  stock = None
  rejects, files, ack_samples = [], [], []
  first = None
  previous_eps = None
  active_request_eps = Counter()
  native_rx_rejected = Counter()
  native_tx = Counter()
  native_tx_examples = []
  for path in sorted(args.rlogs.rglob('rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    compressed = path.read_bytes()
    with zstandard.ZstdDecompressor().stream_reader(compressed) as stream:
      raw = stream.read()
    event_count = 0
    for event in sorted(log.Event.read_multiple_bytes(raw), key=lambda e: int(e.logMonoTime)):
      event_count += 1
      which, now = event.which(), int(event.logMonoTime)
      if first is None:
        first = now
      if safety is not None:
        safety.set_timer((now // 1000) & 0xFFFFFFFF)
      if which == 'carControl':
        control = {'latActive': bool(event.carControl.latActive), 'enabled': bool(event.carControl.enabled),
                   'requested_torque': float(event.carControl.actuators.torque)}
        counts['latActive_frames'] += control['latActive']
      elif which == 'carOutput':
        counts['nonzero_carOutput'] += event.carOutput.actuatorsOutput.torqueOutputCan != 0
      elif which == 'carState':
        cs = event.carState
        state = {'speed_kph': float(cs.vEgoRaw) * 3.6, 'driver_torque_filtered': float(cs.steeringTorque),
                 'gas': bool(cs.gasPressed), 'brake': bool(cs.brakePressed), 'acc': bool(cs.cruiseState.enabled),
                 'canValid': bool(cs.canValid)}
      elif which == 'pandaStates' and event.pandaStates:
        ps = event.pandaStates[0]
        panda = {'model': str(ps.safetyModel), 'param': int(ps.safetyParam),
                 'allowed': bool(ps.controlsAllowed), 'rx_invalid': bool(ps.safetyRxChecksInvalid),
                 'faults': [str(f) for f in ps.faults]}
      elif which in ('can', 'sendcan'):
        for frame in getattr(event, which):
          address, bus, data = int(frame.address), int(frame.src), bytes(frame.dat)
          if which == 'can' and (address, bus) in monitored:
            latest[address, bus] = (now, data)
            if safety is not None and not safety.safety_rx_hook(packet(address, bus, data)):
              native_rx_rejected[f'{address:#x}:{bus}'] += 1
          if which == 'can' and address == 0x495 and bus == 0 and len(data) == 4:
            eps = (data[2] >> 2) & 7
            counts[f'eps_{eps}'] += 1
            if control['latActive']:
              active_request_eps[str(eps)] += 1
            if eps != previous_eps:
              ack_samples.append({'t_s': (now-first)/1e9, 'eps': eps, **control, **state})
              previous_eps = eps
          if address != 0x3F2:
            continue
          label = f'{which}:{bus}'
          decoded = decode_lka(data)
          counts[label] += 1
          payloads[label][data.hex()] += 1
          for key, value in decoded.items():
            distributions[label][key][str(value)] += 1
          if which == 'can' and bus == 2:
            stock = (now, data)
          if which == 'sendcan' and bus == 0 and safety is not None:
            accepted = bool(safety.safety_tx_hook(packet(address, bus, data)))
            native_tx[f'{"lat" if control["latActive"] else "idle"}:{"accepted" if accepted else "rejected"}'] += 1
            if not accepted and len(native_tx_examples) < 12:
              native_tx_examples.append({'t_s': (now-first)/1e9, 'hex': data.hex(), **decoded, **control, **state,
                                         'native_controls_allowed': bool(safety.get_controls_allowed())})
          if which == 'can' and bus == 0xC0:
            context = {}
            for (addr, src), (stamp, value) in latest.items():
              context[f'{addr:#x}:{src}'] = {'hex': value.hex(), 'age_ms': (now-stamp)/1e6}
            raw_driver = latest.get((0x2F5, 0))
            raw_driver = int.from_bytes(raw_driver[1][1:2], 'big', signed=True) * 3 if raw_driver else None
            rejects.append({'t_s': (now-first)/1e9, 'ns': now, 'hex': data.hex(), **decoded,
                            'driver_torque_raw': raw_driver,
                            'native_recheck_at_echo': bool(safety.safety_tx_hook(packet(address, 0, data))) if safety else None,
                            'control': dict(control), 'state': dict(state), 'panda': dict(panda),
                            'stock': {'hex': stock[1].hex(), 'age_ms': (now-stock[0])/1e6,
                                      **decode_lka(stock[1])} if stock else None,
                            'rx': context})
    files.append({'segment': path.parent.name, 'sha256': hashlib.sha256(compressed).hexdigest(),
                  'bytes': len(compressed), 'events': event_count})
    print(path.parent.name, event_count, flush=True)
  report = {'files': files, 'counts': counts, 'fields': distributions,
            'common_payloads': {k: v.most_common(12) for k, v in payloads.items()},
            'eps_transitions': ack_samples, 'eps_during_lateral_request': active_request_eps,
            'rejection_counts': Counter('latActive' if r['control']['latActive'] else 'idle' for r in rejects),
            'rejections': rejects, 'native_rx_rejections': native_rx_rejected,
            'native_tx_replay': native_tx, 'native_tx_rejection_examples': native_tx_examples,
            'scope': 'Recorded RX and attempted TX. Send-event and echo timings differ from firmware timing; native rechecks are contextual, not exact rejection reproduction. No physical EPS response to changed commands is simulated.'}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(report, indent=2) + '\n')
  print(json.dumps({k: report[k] for k in ('counts', 'rejection_counts', 'native_rx_rejections', 'native_tx_replay')}, indent=2))


if __name__ == '__main__':
  main()
