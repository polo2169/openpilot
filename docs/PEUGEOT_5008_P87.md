# Peugeot 5008 P87 — steering ceiling by speed, 26 October 2026

This update replaces the P87's fixed 150-count steering ceiling with the
requested 70-count ceiling at 50 km/h, decreasing linearly to 30 at 130 km/h.
The ACC-coupled engagement, EPS acknowledgement and diagnostics from the
16 October release remain in place.

## Steering ceiling

| Measured wheel speed, km/h | Maximum absolute raw CAN torque |
| --- | --- |
| 50 | 70 |
| 70 | 60 |
| 90 | 50 |
| 110 | 40 |
| 130 | 30 |
| 140 | 30 |

The two knots are interpolated linearly and fractional command magnitudes
are rounded down. Below 50 the ceiling remains 70; above 130 it remains 30.
The existing 30–140 km/h eligibility and real EPS acknowledgement still apply.
These are raw CAN command counts, not a calibrated mechanical torque in Nm.
The previous ceiling was 150 at every speed, so this is a reduction.

The controller and dedicated P87 Panda hook both enforce the curve. The
controller rounds speed up to centi-km/h, conservatively relative to Panda's
integer wheel-speed average. Non-finite host input produces zero torque.
The 150-count normalization, driver-intervention threshold, 8/38-count slew
limits and 75-count real-time safety allowance are unchanged. Other PSA
profiles do not use this curve. Controller diagnostics add torque_limit_raw
alongside the requested and produced torque.

If Panda receives a newer speed before the host and rejects an outdated
command, the controller uses the real rejected-TX echo to restart its ramp
from zero, matching native safety history. Vehicle CAN frames and ordinary
TX echoes cannot trigger this reset. No authority or safety limit is forced;
the existing ACC, EPS, pedal and fault gates must still pass. The rejection
count is logged as panda_lka_rejections.

Tests check the compiled native hook at every 0.01 km/h from 50 through 140,
accept the ceiling and reject one extra count in both directions. Controller
integration also covers reaching each ceiling and tracking acceleration and
deceleration. These software checks do not establish physical EPS authority,
actual lane tracking or a resolution of intermittent ACC engagement.

The branch includes the matching Panda source. Let the ordinary comma build
and firmware update complete before use; keep firmware-signature checking
active. No full comma build or physical test of this curve has been performed
by this release process. A local installation can be rolled back to the
previous installation commit 1ae9cd283641a3030764e9304b0bc18dec8b4f52 with its
matching firmware.

## Engagement and MADS

The experimental P87 safety policy grants steering only with real Peugeot
ACC activity. It has no separate controlsAllowedLateral authorization.
Sunnypilot MADS could nevertheless start independently on cruise-main activity,
causing Controls Mismatch: Lateral alerts while openpilot was disengaged.
MADS availability and its alternativeExperience flags are now disabled only
for P87. User preferences and other cars are unchanged. Ordinary ACC-coupled
engagement, the main controlsMismatch check, its 200-frame threshold and all
fault/pedal/RX checks remain. No Panda permission is forced.

With MADS unavailable, controlsd uses the normal selfdriveState.active value.
The integration tests cover this path through the real controller and compiled
native safety, including EPS preparation, acknowledgement and withdrawal.
The stock ACC remains owned by Peugeot: openpilot neither cancels nor controls
its throttle/braking. CarParams remains openpilotLongitudinalControl=false,
pcmCruise=true, autoResumeSng=false, safetyParam=0x5008 in lateral-test mode.

## Software eligibility from 30 km/h

The P87 software and native Panda gate now use 30–140 km/h. Other PSA ports
retain their thresholds. This is software eligibility, not proof that the
physical EPS supports steering at 30. Torque still requires a real, fresh
EPS state 3, actual ACC activity, valid camera template and every existing
vehicle/driver/safety prerequisite. EPS states 1/2 permit zero-torque preparation
only; EPS 0, faults, unknown states and stale feedback produce no torque.
Python now uses the same 250 ms maximum EPS-feedback age as native safety.
The existing torque/rate/driver limits remain unchanged.

The twelve old captures contain 3,008 EPS=0 samples and 622 EPS=2 samples in
30–<52 km/h. The minimum observed available EPS speed is 49.875 km/h, and
all observed EPS=3 samples are above 68.48 km/h. No command at 30 was tested
physically. If the EPS stays unauthorized, this update keeps a neutral command
and reports eps_not_authorized. Do not interpret an enabled/green UI alone as
confirmation of steering assistance. No EPS state, driver torque or hands
message is spoofed to overcome this limit.

## Diagnostics and longitudinal research

The controller logs psa_p87_status on phase changes and once per second.
It distinguishes lateral_not_requested, EPS refusal, waiting_for_eps_ack,
stale CAN/template/EPS feedback, speed and driver inhibitions, and active
commands. It records requested torque, actual controller torque, LKA state
and factor, native EPS state/age, stock ACC status and relevant stock
braking/torque/standstill/GO fields. The fields labelled DBC are observations;
their names/scaling alone do not validate a P87 braking controller.

The old captures contain native requested deceleration from -2.4 to the
2.05 DBC sentinel, but no stock-ACC-active stationary interval, no asserted
VEHICLE_STANDSTILL, DRIVE_AWAY_REQUEST or COCKPIT_GO_ACC_REQUEST. They do not
validate standstill holding or automatic restart. Openpilot longitudinal and
Stop & Go therefore remain unimplemented and disabled in this driving branch.
No brake, engine-torque or resume TX is whitelisted. The new logging is the
concrete preparation for examining normal Peugeot ACC stops and resumptions.

After a completed trip, run this read-only command on the comma:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

Share /data/5008-diagnostic.json. An exact older route can instead be selected
with --route PREFIX. The collector reads local logs only: no sockets, parameter
writes or CAN transmissions. It distinguishes actuators.torque, carOutput torque,
sendcan 0x3F2 and CAN echoes/rejections, plus MADS state and controller diagnostics.
Its coverage flag reports missing/corrupt segments and qlog-only sampling;
those cannot prove that a CAN command was absent. Original rlogs remain needed
for detailed validation of new longitudinal commands.

## Installation and validation

Installation branch: https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

Update while parked. Allow the normal build and matching Panda firmware update,
then reboot. Keep firmware-signature checking enabled. The standard branch
explicitly selects lateral-test; local install.py defaults to observation.
A local patch installation checks supported revisions and clean trees before
changing files and saves the old configuration, docs and reversible patches.
The new source/flat 13 October releases are supported in addition to older
P87 releases. Unknown/dirty revisions are rejected.

Targeted tests exercise the host/native 30 km/h boundaries, EPS freshness and
acknowledgement, driver priorities, real ACC edges, MADS manual-mode behavior,
ordinary selfdrive activation, and the prohibition of brake/resume TX.
The twelve old captures are replayed through native RX, car events and the
host state machine. Synthetic ACK tests and CAN replay do not validate the
physical vehicle response. A full on-device build and physical steering at
30 have not been verified by this release process.

For rollback of a local patch install, reverse the saved host and opendbc
patches in their recorded directories, restore the saved configuration and
documentation, then rebuild the previous sources before restarting. An older
branch checkout likewise requires its matching firmware; do not skip flashing.
