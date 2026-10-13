# Peugeot 5008 P87 — ACC / Panda correction, 13 October 2026

The previous release `fc68d36` selected the lateral test profile correctly, but
Panda could consume the activation-request edge on 0x452 before the active
ACC status arrived on 0x2B6. The later status update never granted authority.
An offline reproduction using the published safety code rejects steering;
the old safety tests hid this by forcing controlsAllowed=true.

## Changes

Panda waits for all nine valid, fresh RX prerequisites before interpreting
initial engagement. It tracks activation request AND actual ACC active status
(4 or 6), reevaluating when either message changes. Python uses the same state.
Armed/suspended ACC (3/5/7) is available but not engaged. Fault, ignition, body
and pedal gates still inhibit controls; recovering a fault or releasing a pedal
alone does not re-arm a still-active ACC session. A new cruise off/on or actual
inactive-to-active transition is required, with the pedals released.

The driver pedal cuts openpilot immediately even if the optional generic
accelerator-disengagement setting is disabled. Enable requests while pressing
a pedal are blocked. Clearing an ACC fault without a new ACC transition also
blocks an unsolicited host engagement.

This P87 controller emits only lateral LKA and does not cancel or control
Peugeot ACC. The stock-cruise mismatch alert now respects that ownership on
this P87 lateral-only port. Other ports retain their existing check. The
controlsMismatch check, its 200-frame threshold, RX checks and torque limits
are unchanged. CarParams remains openpilotLongitudinalControl=false and
pcmCruise=true; Panda uses psa, safetyParam=0x5008 in the opt-in lateral mode.

## Validation and limits

63 targeted/interface/native-safety/car-state-machine tests pass. They no
longer force Panda authority, and cover both message orders, startup feedback,
ACC suspension/resume, pedal/fault recovery, the 2-second window and steering
gates. The existing legacy PSA suite still has the same 10 known failures.

The twelve previously supplied captures contain 5,625,246 CAN frames. Full
native RX replay accepts all 564,527 checked prerequisite packets. The old
version grants zero authority. The corrected replay grants about 22.98 seconds
in one valid session. Car events and the host engagement state machine agree
with Panda for all 2,298 enabled updates: zero enabled/authority disagreements,
zero cruise mismatches and zero unsafe admissions. The other stock-ACC intervals
remain unengaged after driver overrides; stock ACC is allowed to continue.
This replay excludes model/driver-monitoring/MADS state and physical actuation.

Vyciokas's latest trial on fc68d36 is available only as his written report;
its route and /data/mismatch_capture.log have not been supplied here. These
checks do not prove the EPS accepts steering commands on his car.

## Update

Use the same installation branch:
https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

Install while parked, allow compilation and the matching Panda firmware update
to finish, then reboot. Keep the normal firmware-signature check enabled.
Verify PSA safetyParam 0x5008 (20488), openpilotLongitudinalControl=false,
pcmCruise=true and safetyRxChecksInvalid=false.

For a local patch installation, copy this package outside /data/openpilot:

```sh
python3 /data/5008-cristianku/install.py --openpilot-root /data/openpilot --mode lateral-test --check-only
python3 /data/5008-cristianku/install.py --openpilot-root /data/openpilot --mode lateral-test --build
```

The installer accepts stable Cristianku with original/T15 opendbc, original P87
source/flattened releases, and the 12 October source 49d2444 / release fc68d36
with opendbc 30d2e2b. It checks BOTH patches before changing files and saves
both patches, old configuration and documentation. Unknown/dirty revisions are
rejected. Without an explicit --mode, local patch installation selects observe.

Lateral TX remains restricted to 0x3F2 bus 0, 52–140 km/h, valid camera template
and EPS acknowledgement. Recognition while moving is unchanged. ACC can be
active below 52 km/h while steering TX stays inhibited. The 50 km/h setting
for Paul's 308 is separate and is not applied to this 5008.

The first physical validation should be on a closed course. Record CarParams,
pandaStates, carState, carControl and CAN during enable/suspend/resume. Expected:
controlsAllowed=true on a valid ACC edge, no 2-second controlsMismatch kill;
zero steering command below 52 km/h and until EPS acknowledges the handshake.

To roll back a local patch installation, stop openpilot, reverse the saved
openpilot-controls.patch at the root and opendbc.patch in opendbc_repo (or use
--directory=opendbc_repo for a flattened tree), restore the saved configuration
and documentation, and rebuild the previous sources before restarting.
