#!/usr/bin/env python3
"""Extract P87 lateral dynamics from closed rlogs without vehicle access."""
import argparse
from array import array
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

FIELDS = {
  'car': 't valid speed angle rate yaw driver pressed gas brake acc blinker can_valid can_timeout steer_fault'.split(),
  'control': 't valid active desired actual error p i d f output saturated curvature jerk'.split(),
  'command': 't valid enabled lat_active torque'.split(),
  'output': 't valid torque raw'.split(),
  'eps': 't state byte0 byte1 byte2 byte3'.split(),
  'driver': 't raw'.split(),
  'can': 't kind bus torque status factor'.split(),
  'model': 't valid curvature left_prob right_prob left_y right_y lane_change'.split(),
  'params': 't valid ratio angle_offset roll'.split(),
  'torque_params': 't valid use factor offset friction'.split(),
  'delay': 't valid delay estimate status'.split(),
}


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

  args.output.mkdir(parents=True, exist_ok=True)
  rows = {key: array('d') for key in FIELDS}
  metadata = {'files': [], 'fields': FIELDS, 'counts': Counter(), 'car_params': [], 'selected_params': []}
  with gzip.open(args.output/'events.jsonl.gz', 'wt') as events:
    def emit(kind, t, data):
      events.write(json.dumps({'kind': kind, 't': t, 'data': data}, separators=(',', ':'))+'\n')
    for path in sorted(args.rlogs.rglob('rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
      compressed = path.read_bytes()
      with zstandard.ZstdDecompressor().stream_reader(compressed) as stream:
        raw = stream.read()
      count = 0
      for e in log.Event.read_multiple_bytes(raw):
        kind, t = e.which(), int(e.logMonoTime)/1e9
        count += 1
        metadata['counts'][kind] += 1
        if kind == 'carState':
          c = e.carState
          rows['car'].extend((t,e.valid,c.vEgoRaw*3.6,c.steeringAngleDeg,c.steeringRateDeg,c.yawRate,
            c.steeringTorque,c.steeringPressed,c.gasPressed,c.brakePressed,c.cruiseState.enabled,
            c.leftBlinker or c.rightBlinker,c.canValid,c.canTimeout,c.steerFaultTemporary or c.steerFaultPermanent))
        elif kind == 'controlsState' and e.controlsState.lateralControlState.which() == 'torqueState':
          c = e.controlsState; q = c.lateralControlState.torqueState
          rows['control'].extend((t,e.valid,q.active,q.desiredLateralAccel,q.actualLateralAccel,q.error,
            q.p,q.i,q.d,q.f,q.output,q.saturated,c.desiredCurvature,q.desiredLateralJerk))
        elif kind == 'carControl':
          c=e.carControl; rows['command'].extend((t,e.valid,c.enabled,c.latActive,c.actuators.torque))
        elif kind == 'carOutput':
          c=e.carOutput.actuatorsOutput; rows['output'].extend((t,e.valid,c.torque,c.torqueOutputCan))
        elif kind in ('can', 'sendcan'):
          for f in getattr(e,kind):
            data=bytes(f.dat); bus=int(f.src)
            if f.address == 0x3F2 and len(data)==8 and (kind=='sendcan' or bus in (0,2,128,192)):
              raw_torque=(data[3]<<3)|(data[4]>>5)
              rows['can'].extend((t,kind=='sendcan',bus,raw_torque-2048 if raw_torque&1024 else raw_torque,(data[4]>>2)&7,data[5]>>1))
            elif kind=='can' and f.address==0x495 and bus==0 and len(data)==4:
              rows['eps'].extend((t,(data[2]>>2)&7,*data))
            elif kind=='can' and f.address==0x2F5 and bus==0 and len(data)==7:
              rows['driver'].extend((t,int.from_bytes(data[1:2],'big',signed=True)*3))
        elif kind == 'modelV2':
          m=e.modelV2
          if len(m.laneLineProbs)==4 and len(m.laneLines)==4 and m.laneLines[1].y and m.laneLines[2].y:
            rows['model'].extend((t,e.valid,m.action.desiredCurvature,m.laneLineProbs[1],m.laneLineProbs[2],
                                 m.laneLines[1].y[0],m.laneLines[2].y[0],m.meta.laneChangeState.raw))
        elif kind == 'vehicleParameters':
          p=e.vehicleParameters; rows['params'].extend((t,e.valid,p.steerRatio,p.angleOffsetDeg,p.roll))
        elif kind == 'lateralTorqueParameters':
          p=e.lateralTorqueParameters; rows['torque_params'].extend((t,e.valid,p.useParams,p.latAccelFactorFiltered,p.latAccelOffsetFiltered,p.frictionCoefficientFiltered))
        elif kind == 'lateralDelay':
          p=e.lateralDelay; rows['delay'].extend((t,e.valid,p.lateralDelay,p.lateralDelayEstimate,p.status.raw))
        elif kind == 'logMessage':
          try:
            text=json.loads(str(e.logMessage)).get('msg','')
          except (ValueError,TypeError):
            text=str(e.logMessage)
          if isinstance(text,str) and text.startswith('psa_p87_status '):
            emit('controller',t,json.loads(text[len('psa_p87_status '):]))
        elif kind == 'selfdriveState':
          s=e.selfdriveState
          if s.alertType:emit('alert',t,{'type':str(s.alertType),'text1':str(s.alertText1),'text2':str(s.alertText2)})
        elif kind == 'pandaStates':
          for p in e.pandaStates:
            if p.faults:emit('panda_fault',t,{'faults':[str(f) for f in p.faults]})
        elif kind == 'carParams':
          if not metadata['car_params']:
            p=e.carParams;metadata['car_params'].append({'fingerprint':str(p.carFingerprint),
              'steerActuatorDelay':p.steerActuatorDelay,'lateralTuning':p.lateralTuning.to_dict()})
        elif kind == 'initData' and not metadata['selected_params']:
          entries=e.initData.params.entries
          selected={str(p.key):bytes(p.value).decode(errors='replace') for p in entries
            if any(token in str(p.key).lower() for token in ('torque','neural','jerk','nnlc','lateralcontrol','lateralcontrollag'))
            and len(bytes(p.value))<300}
          metadata['selected_params'].append(selected)
      metadata['files'].append({'segment':path.parent.name,'sha256':hashlib.sha256(compressed).hexdigest(),
                                'bytes':len(compressed),'events':count})
      print(path.parent.name, count, flush=True)
  output = {}
  for key, values in rows.items():
    a=np.asarray(values).reshape((-1,len(FIELDS[key])))
    a=a[np.argsort(a[:,0],kind='stable')]
    output[key]=a
  np.savez_compressed(args.output/'streams.npz',**output)
  (args.output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
  print({key:len(a) for key,a in output.items()},flush=True)


if __name__ == '__main__':
  main()
