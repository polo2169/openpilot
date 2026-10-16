#!/usr/bin/env python3
"""Read-only P87 route audit. No sockets, Panda API, parameter writes or CAN TX.

Run from the tested checkout with its Python environment. An rlog audit is
needed to conclude that a command was absent; qlogs are sampled observations.
"""
import argparse
import bz2
from collections import Counter, defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import sys


def value(obj, name):
  try:
    for part in name.split('.'):
      obj = getattr(obj, part)
    if isinstance(obj, (bool, int, float, str)):
      return obj
    return str(obj)
  except (AttributeError, RuntimeError):
    return None


def decode_lka(data):
  if len(data) != 8:
    return None
  raw = (data[3] << 3) | (data[4] >> 5)
  return dict(torque_raw=raw-2048 if raw & 1024 else raw,
              state=(data[4] >> 2) & 7, factor=data[5] >> 1)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('realdata', type=Path)
  selection = parser.add_mutually_exclusive_group(required=True)
  selection.add_argument('--route', help='Exact route prefix, without segment number')
  selection.add_argument('--latest', action='store_true', help='Read the newest local route')
  parser.add_argument('--root', type=Path, default=Path('/data/openpilot'))
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  sys.path.insert(0, str(args.root.resolve()))
  from openpilot.cereal import log
  import zstandard
  if args.latest:
    candidates = [p for p in args.realdata.iterdir() if p.is_dir() and len(p.name.rsplit('--', 2)) == 3
                  and p.name.rsplit('--', 1)[1].isdigit()
                  and any((p/n).is_file() for n in ('rlog.zst','rlog.bz2','rlog','qlog.zst','qlog.bz2','qlog'))]
    if not candidates:
      raise SystemExit('No local route segments')
    newest = max(candidates, key=lambda p: p.stat().st_mtime_ns)
    args.route = newest.name.rsplit('--', 1)[0]
  report = dict(route=args.route, files=[], car_params=[], init=[], counts=Counter(),
                lka=defaultdict(Counter), errors=[], snapshots=[], controller_diagnostics=[])
  histories = defaultdict(lambda: deque(maxlen=60))
  last_snapshot = 0
  directories = sorted(args.realdata.glob(args.route+'--*'), key=lambda p: int(p.name.rsplit('--',1)[1]))
  if not directories:
    raise SystemExit('No matching route segments')
  def stash(kind, ns, row):
    histories[kind].append((ns,row))
  def prior(kind, ns):
    rows = [r for r in histories[kind] if 0 <= ns-r[0] <= 200_000_000]
    return max(rows, key=lambda r:r[0])[1] if rows else None
  for directory in directories:
    path = next((directory/n for n in ('rlog.zst','rlog.bz2','rlog','qlog.zst','qlog.bz2','qlog')
                 if (directory/n).is_file()), None)
    if path is None:
      report['errors'].append(dict(segment=directory.name, error='No log file'))
      continue
    compressed = path.read_bytes()
    entry = dict(segment=directory.name, file=path.name, sha256=hashlib.sha256(compressed).hexdigest(),
                 bytes=len(compressed), complete=False)
    report['files'].append(entry)
    try:
      if path.suffix == '.zst':
        decoder=zstandard.ZstdDecompressor().decompressobj()
        raw=decoder.decompress(compressed)
        if not decoder.eof:
          report['errors'].append(dict(segment=directory.name, error='Truncated zstd stream'))
      else:
        raw=bz2.decompress(compressed) if path.suffix == '.bz2' else compressed
      for event in log.Event.read_multiple_bytes(raw):
        kind, ns = event.which(), int(event.logMonoTime)
        report['counts'][kind] += 1
        report['counts'][kind+'_invalid'] += not event.valid
        if kind == 'carParams':
          cp=event.carParams
          fields=('carFingerprint','brand','dashcamOnly','passive','openpilotLongitudinalControl',
                  'pcmCruise','autoResumeSng','minSteerSpeed','minEnableSpeed','alternativeExperience')
          row={k:value(cp,k) for k in fields}
          row['safetyConfigs']=[s.to_dict() for s in cp.safetyConfigs]
          report['car_params'].append(row)
        elif kind == 'initData':
          report['init'].append({k:value(event.initData,k) for k in ('gitCommit','gitBranch','gitDirty','version')})
        elif kind == 'carState':
          cs=event.carState
          fields=('canValid','canTimeout','vEgoRaw','steeringTorque','steeringPressed','steerFaultTemporary',
            'steerFaultPermanent','brakePressed','gasPressed','cruiseState.enabled','cruiseState.available',
            'cruiseState.standstill','accFaulted')
          stash(kind,ns,{k:value(cs,k) for k in fields})
        elif kind in ('selfdriveState','selfdriveStateSP','carOutput'):
          src=getattr(event,kind)
          fields={'selfdriveState':('enabled','active','state','alertType','alertText1','alertText2'),
            'selfdriveStateSP':('mads.available','mads.enabled','mads.active','mads.state'),
            'carOutput':('actuatorsOutput.torque','actuatorsOutput.torqueOutputCan')}[kind]
          row={k:value(src,k) for k in fields};stash(kind,ns,row)
          if kind=='carOutput':
            for k,v in row.items():
              report['counts'][k+'_missing'] += v is None
              report['counts'][k+'_nonzero'] += v is not None and math.isfinite(v) and abs(v)>1e-9
        elif kind == 'pandaStates':
          stash(kind,ns,[{k:value(p,k) for k in ('safetyModel','safetyParam','controlsAllowed',
              'controlsAllowedLateral','safetyRxChecksInvalid','faults')} for p in event.pandaStates])
        elif kind == 'carControl':
          cc=event.carControl
          torque=value(cc,'actuators.torque')
          report['counts']['carControl_nonzero_torque'] += torque is not None and abs(torque)>1e-9
          report['counts']['carControl_torque_missing'] += torque is None
          report['counts']['carControl_latActive'] += bool(cc.latActive)
          if ns-last_snapshot >= 1_000_000_000:
            last_snapshot=ns
            row={'ns':ns,'carControl':{k:value(cc,k) for k in ('enabled','latActive','longActive','actuators.torque')}}
            for name in ('carState','selfdriveState','selfdriveStateSP','carOutput','pandaStates'):
              row[name]=prior(name,ns)
            report['snapshots'].append(row)
        elif kind in ('logMessage', 'errorLogMessage'):
          message = str(getattr(event, kind))
          try:
            outer = json.loads(message)
            message = str(outer.get('msg', outer.get('msg$s', message)))
          except ValueError:
            pass
          if message.startswith('psa_p87_status '):
            try:
              row = json.loads(message.removeprefix('psa_p87_status '))
              row['log_ns'] = ns
              report['controller_diagnostics'].append(row)
            except ValueError:
              report['errors'].append(dict(segment=directory.name, error='Malformed P87 diagnostic'))
        elif kind in ('can','sendcan'):
          for frame in getattr(event,kind):
            bus,address,data=int(frame.src),int(frame.address),bytes(frame.dat)
            if address == 0x3F2:
              command=decode_lka(data)
              if command:
                counts=report['lka'][f'{kind}_bus_{bus}']
                counts['frames']+=1;counts['nonzero_torque']+=command['torque_raw']!=0
                counts['max_abs_torque_raw']=max(counts['max_abs_torque_raw'],abs(command['torque_raw']))
                counts['state_'+str(command['state'])]+=1
    except Exception as exc:
      report['errors'].append(dict(segment=directory.name,error=f'{type(exc).__name__}: {exc}'))
    else:
      entry['complete']=True
  segments=[int(d.name.rsplit('--',1)[1]) for d in directories]
  report['missing_segments'] = sorted(set(range(0,max(segments)+1))-set(segments))
  report['complete_rlog_coverage'] = (not report['errors'] and not report['missing_segments']
    and all(f['complete'] and f['file'].startswith('rlog') for f in report['files']))
  report['p87_fingerprint_confirmed'] = (bool(report['car_params']) and
    all(cp['carFingerprint'] == 'PSA_PEUGEOT_5008_P87' for cp in report['car_params']))
  report['coverage_scope']='Local segments 0 through highest available; route end is not independently verified.'
  report['note']='Counts distinguish requested torque, controller output and raw CAN. None is a measured steering response.'
  args.output.parent.mkdir(parents=True,exist_ok=True)
  args.output.write_text(json.dumps(report,indent=2)+'\n')
  print(json.dumps({k:report[k] for k in ('route','complete_rlog_coverage','counts','lka','errors')},indent=2))
  return 0 if report['complete_rlog_coverage'] else 1


if __name__ == '__main__':
  sys.exit(main())
