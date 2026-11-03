#!/usr/bin/env python3
"""Offline candidate-controller replay with recorded CAN, modelV2 and requests."""
import argparse
from collections import Counter
import hashlib
import json
import logging
import os
from pathlib import Path
import sys


class RecordedModels:
  def __init__(self):
    self.seen = {'modelV2': False}
    self.valid = {'modelV2': False}
    self.alive = {'modelV2': False}
    self.logMonoTime = {'modelV2': 0}
    self.model = None

  def update(self, timeout):
    pass

  def __getitem__(self, key):
    return self.model


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--openpilot-root', type=Path, required=True)
  parser.add_argument('--opendbc-root', type=Path, required=True)
  parser.add_argument('--rlogs', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  sys.path[:0] = [str(args.opendbc_root.resolve()), str(args.openpilot_root.resolve())]
  import zstandard
  from openpilot.cereal import log
  from opendbc.car import structs
  from opendbc.car.carlog import carlog
  from opendbc.car.psa.interface import CarInterface
  from opendbc.car.psa.values import CAR, p87_torque_limit
  from opendbc.safety.tests.libsafety import libsafety_py
  carlog.setLevel(logging.CRITICAL)
  os.environ['PSA_5008_LATERAL_TEST'] = '1'
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_5008_P87)
  ci = CarInterface(cp, CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_5008_P87))
  models = ci.CC.model_sm = RecordedModels()
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 0x5008) == 0
  safety.init_tests()
  monitored = {(0x228, 0), (0x348, 0), (0x30D, 0), (0x2F5, 0), (0x495, 0),
               (0x412, 2), (0x452, 1), (0x2B6, 1), (0x32D, 1)}
  counts, phases = Counter(), Counter()
  files = []
  transitions = []
  previous_phase = None
  state = None
  for path in sorted(args.rlogs.rglob('rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    compressed = path.read_bytes()
    with zstandard.ZstdDecompressor().stream_reader(compressed) as stream:
      raw = stream.read()
    # Logger multiplexing can place a newer CAN event before an older
    # carControl event. Reconstruct timestamp order instead of presenting
    # future CAN samples to the freshness checks.
    for event in sorted(log.Event.read_multiple_bytes(raw), key=lambda e: int(e.logMonoTime)):
      now, which = int(event.logMonoTime), event.which()
      safety.set_timer((now//1000) & 0xFFFFFFFF)
      if which == 'modelV2':
        models.model = event.modelV2
        models.seen['modelV2'] = models.alive['modelV2'] = True
        models.valid['modelV2'] = bool(event.valid)
        models.logMonoTime['modelV2'] = now
      elif which == 'can':
        frames = [(int(f.address), bytes(f.dat), int(f.src)) for f in event.can]
        for address, data, bus in frames:
          if (address, bus) in monitored:
            counts['native_rx_rejected'] += not safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
        state, _ = ci.update([(now, frames)])
      elif which == 'carControl' and state is not None:
        output, sends = ci.apply(event.carControl, structs.CarControlSP(), now)
        counts['control_updates'] += 1
        counts['lateral_requested'] += bool(event.carControl.latActive)
        counts['nonzero_output'] += output.torqueOutputCan != 0
        assert abs(output.torqueOutputCan) <= p87_torque_limit(ci.CS.speed_kph)
        status = ci.CC.p87_lateral_status
        phase = status['phase']
        phases[phase] += 1
        if phase != previous_phase:
          transitions.append({'ns': now, 'phase': phase, 'eps_state': ci.CS.eps_state_lka,
                              'speed_kph': ci.CS.speed_kph})
          previous_phase = phase
        for frame in sends:
          assert (frame.address, frame.src) == (0x3F2, 0)
          wire_status, factor = (frame.dat[4] >> 2) & 7, frame.dat[5] >> 1
          counts[f'lka_state_{wire_status}'] += 1
          accepted = bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, frame.dat)))
          counts['native_tx_accepted' if accepted else 'native_tx_rejected'] += 1
          counts['zero_torque_activation_requests'] += wire_status == 4 and output.torqueOutputCan == 0 and factor > 0
    files.append({'segment': path.parent.name, 'sha256': hashlib.sha256(compressed).hexdigest()})
    print(path.parent.name, flush=True)
  # This route contains no physical EPS=3. Candidate replay must remain inert.
  assert counts['nonzero_output'] == 0
  assert counts['zero_torque_activation_requests'] > 0
  assert counts['native_rx_rejected'] == 0
  args.output.write_text(json.dumps({'files': files, 'counts': counts, 'phases': phases,
    'transitions': transitions,
    'scope': 'Controller and native safety against recorded CAN/models/requests. The new zero-torque request cannot cause a new EPS response in a recording; no physical steering success is claimed.'}, indent=2)+'\n')
  print(json.dumps({'counts': counts, 'phases': phases}, indent=2))


if __name__ == '__main__':
  main()
