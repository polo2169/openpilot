#!/usr/bin/env python3
"""Frozen-input P87 PID comparison. Does not simulate the new vehicle path."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np


def prior(a,t,age=.3):
  j=np.maximum(0,np.searchsorted(a[:,0],t,side='right')-1)
  result=a[j].copy()
  result[(t<a[j,0])|(t-a[j,0]>age)]=np.nan
  return result


def quantiles(a):
  return np.nanpercentile(a,[5,50,95]).tolist() if len(a) else []


def main():
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument('--features',type=Path,required=True)
  p.add_argument('--openpilot-root',type=Path,required=True)
  p.add_argument('--output',type=Path,required=True)
  p.add_argument('--baseline-version',type=int,choices=(0,1),required=True,help='Actual torqueState.version from the recorded controller')
  args=p.parse_args()
  sys.path[:0]=[str((args.openpilot_root/'opendbc_repo').resolve()),str(args.openpilot_root.resolve())]
  from openpilot.common.pid import PIDController
  from openpilot.selfdrive.controls.lib import p87_torque
  if args.baseline_version == 0:
    from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import INTERP_SPEEDS, KP_INTERP, KI
  else:
    from openpilot.selfdrive.controls.lib.latcontrol_torque import INTERP_SPEEDS, KP_INTERP, KI
  meta=json.loads((args.features/'metadata.json').read_text());z=np.load(args.features/'streams.npz')
  F={k:{v:i for i,v in enumerate(cols)} for k,cols in meta['fields'].items()}
  a=z['control'];t=a[:,0];K=F['control']
  car=prior(z['car'],t);command=prior(z['command'],t);eps=prior(z['eps'],t);tp=prior(z['torque_params'],t)
  v=car[:,F['car']['speed']]/3.6
  active=(a[:,K['active']]>0)&(command[:,F['command']['lat_active']]>0)&np.isfinite(tp).all(axis=1)&np.isfinite(car).all(axis=1)
  factor=tp[:,F['torque_params']['factor']];friction=tp[:,F['torque_params']['friction']]
  if 'version' in K:
    assert np.all(a[active,K['version']]==args.baseline_version), 'Mixed or unexpected recorded controller versions'
  baseline_reconstructed=-np.clip(a[:,K['p']]+a[:,K['i']]+a[:,K['d']]+a[:,K['f']],-factor,factor)/factor
  # Both optional torque-space extensions were disabled in the supplied route.
  params=meta['selected_params'][0]
  assert params.get('NeuralNetworkLateralControl')=='0'
  assert params.get('LateralJerkTorqueController')=='0'
  friction_input=a[:,K['error']] if args.baseline_version == 0 else a[:,K['error']]+.3*a[:,K['jerk']]
  threshold=.3 if args.baseline_version == 0 else .2
  old_ff_friction=np.clip(friction_input/threshold,-1.,1.)*friction*factor
  new_ff=a[:,K['f']]-old_ff_friction+old_ff_friction*np.minimum(friction,p87_torque.MAX_FRICTION)/friction
  candidate=np.zeros(len(a));candidate_i=np.zeros(len(a))
  pid=PIDController([INTERP_SPEEDS,[x*p87_torque.KP_SCALE for x in KP_INTERP]],KI*p87_torque.KI_SCALE)
  previous=False
  for i in range(len(a)):
    if not active[i]:
      previous=False
      continue
    if not previous or i==0 or t[i]-t[i-1]>.15:
      pid.reset();pid.i=a[i,K['i']]*p87_torque.KI_SCALE
    # Keep the recorded integrator-freeze decision. Changed vehicle response
    # and changed future safety-limited flags cannot be known from this log.
    freeze=(not previous or abs(a[i,K['i']]-a[i-1,K['i']])<1e-9)
    ceiling=p87_torque.normalized_limit(v[i]*3.6)
    pid.set_limits(ceiling*factor[i],-ceiling*factor[i])
    candidate[i]=-pid.update(a[i,K['error']],speed=v[i],feedforward=new_ff[i],freeze_integrator=freeze)/factor[i]
    candidate_i[i]=pid.i
    previous=True
  valid=active&(eps[:,F['eps']['state']]==3)&(car[:,F['car']['can_valid']]>0)
  baseline=command[:,F['command']['torque']]
  report={'scope':'Frozen recorded errors, perception, torque estimates and freeze decisions; no changed steering trajectory or physical EPS response is simulated.',
    'recorded_controller_version':args.baseline_version,
    'profile':{'kp_scale':p87_torque.KP_SCALE,'ki_scale':p87_torque.KI_SCALE,'max_friction_normalized':p87_torque.MAX_FRICTION},
    'baseline_reconstruction_absolute_error':quantiles(abs(baseline_reconstructed[valid]-a[valid,K['output']])),
    'speed_groups':[],'frequency_windows':[]}
  for speed in (70,90,110,120):
    m=valid&(abs(v*3.6-speed)<5)
    old=abs(baseline[m])*150.;new=abs(candidate[m])*150.
    report['speed_groups'].append({'speed_center_kph':speed,'samples':int(m.sum()),
      'baseline_request_abs_raw':quantiles(old),'candidate_request_abs_raw':quantiles(new),
      'baseline_request_rms_raw':float(np.sqrt(np.mean(old**2))),'candidate_request_rms_raw':float(np.sqrt(np.mean(new**2))),
      'actual_lateral_accel':quantiles(a[m,K['actual']]),'desired_lateral_accel':quantiles(a[m,K['desired']])})
  for start,end in ((350,370),(380,400),(720,740),(780,800),(1210,1230),(1380,1400)):
    m=valid&(t>=start)&(t<end)
    tx=t[m];x=-car[m,F['car']['yaw']]*v[m]
    if len(x)<1000:
      continue
    freq=np.fft.rfftfreq(len(x),np.median(np.diff(tx)))
    power=abs(np.fft.rfft((x-x.mean())*np.hanning(len(x))))**2
    ids=np.flatnonzero((freq>.1)&(freq<3));peak=int(ids[np.argmax(power[ids])])
    report['frequency_windows'].append({'start_mono_s':start,'end_mono_s':end,'samples':len(x),
      'mean_speed_kph':float(np.mean(v[m])*3.6),'dominant_yaw_frequency_hz':float(freq[peak]),
      'measured_yaw_lateral_accel_std':float(x.std()),'desired_lateral_accel_std':float(a[m,K['desired']].std())})
  args.output.mkdir(parents=True,exist_ok=True)
  (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
  np.savez_compressed(args.output/'comparison.npz',t=t,speed=v*3.6,valid=valid,baseline=baseline*150,
    candidate=candidate*150,actual=a[:,K['actual']],desired=a[:,K['desired']],yaw_accel=-car[:,F['car']['yaw']]*v)
  print(json.dumps(report,indent=2))


if __name__=='__main__':
  main()
