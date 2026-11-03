# Peugeot 5008 P87 — EPS admission test release, 3 November 2026

This test release updates the `f76cec60` installation reported by Vyck.
Installation branch: https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

The controller, matching Panda source and test instructions are included.
89 local tests pass. No full comma-target build, device flashing or physical
steering validation has been performed by this release process. Read
PEUGEOT_5008_TEST_PLAN_20261103.md before the first vehicle test.

The old controller requested LKA state 3 until EPS state 3 arrived, and only
then requested LKA state 4. The complete supplied route contains no LKA state 4
from openpilot and no EPS state 3. The candidate separates the activation
request from its acknowledgement: state 4 is first sent with zero torque.
Only a subsequent, fresh physical EPS state 3 can authorize the ordinary
torque ramp. This fixes the software circular wait; the vehicle's response
to the corrected request still needs to be observed.

Every initial admission or resumption requires:

- At least 300 ms of stable conditions and three distinct valid modelV2
  samples, with plausible confident lane boundaries and lane-change state off.
- Fresh vehicle CAN, camera template and EPS feedback, actual Peugeot ACC
  activity, released pedals, and the existing body, fault, speed and driver gates.
- A fresh released EPS state 1/2, 100 ms of state-3 zero-torque preparation,
  followed by state 4 at zero torque with a progressive factor.
- A new EPS=3 timestamp after that activation request. An old active EPS
  cannot authorize a fresh session. A missing ACK times out after one second
  and requires a new request; a failed handshake is never automatically forced.

The compiled Panda policy independently requires a zero-torque state-4
request followed by a new physical ACK. Neutral state 2 withdraws this
authorization. The host validates the lane models; Panda has no lane-model
input. Neither side fabricates EPS, driver torque or hands-on feedback.

Driver intervention and indicators suspend the command and require the
same validation before resuming within an otherwise admissible session.
Faults, stale feedback, pedals and cancellation retain their manual rearm
behavior. This update does not transplant the 308's separate RVV session
policy. The P87 driver threshold remains 45 raw counts and software speed
eligibility remains 30–140 km/h; actual EPS authorization is always required.
P87 MADS remains disabled and engagement follows ordinary stock-ACC activity.

The raw CAN torque ceiling is 90 at 50 km/h and 40 at 130 km/h. Linear
interpolation is rounded down: 77 at 70, 65 at 90 and 52 at 110. It clamps
to 90 below 50 and 40 above 130, within the separate eligibility envelope.
These are command counts, not measured mechanical Nm. The host rounds
wheel speed up to centi-km/h, conservatively relative to Panda's integer
average. Normalization remains 150, ordinary slew limits remain 8/38 per
50 ms, and the real-time allowance remains 75. Both host and native safety
enforce the ceiling, with no one-count tolerance above it.

The original rlogs contain 128 rejected LKA echoes: 125 during a lateral
request and three during startup. All 125 have unfiltered driver torque
outside ±45 in their recorded context; the corresponding compiled original
native hook rejects all 125 contextual rechecks. The host previously used
filtered torque. It now checks the raw signal too and logs both values.
Thread and firmware timings differ, so sendcan-only replay is not an exact
reconstruction of rejection timing.

All 36,905 native camera LKA frames have zero in the DBC-labelled counter
and checksum nibbles, DRIVE and LXA bits, and requested angle. The nibble
sum is not constant. The candidate preserves the native template and does
not invent a checksum or raise authorization bits. Factory state 4 is
visible on the camera side; the P87 safety forwarding policy blocks factory
0x3F2 toward the car. The capture contains no car-side state-4 LKA frame.
These observations do not establish the physical wiring or prove the EPS
will accept the candidate handshake.

Pressing the accelerator continues to prevent engagement and to remove
steering authority. Releasing it alone cannot restore an already cancelled
session: a new actual ACC edge is required. This is the current P87 policy,
not evidence of a pedal decoding defect in this route. The reported lane
assist startup fault cannot be diagnosed from this already-running route.

Validation consists of 89 passing local tests, native C compilation with
undefined-behavior checks, exact ceiling/+1 rejection checks over 9,001
centi-km/h speed points in both directions, host/native handshake integration,
and all 31 original rlogs replayed with their recorded models and requests.
The replay emits 60 zero-torque activation requests, all accepted by native
safety, and never emits torque without physical EPS=3. A synthetic ACK tests
the software ramp; it does not simulate the vehicle's physical response.

`install.py` is a source patch installer for supported clean revisions. It
defaults to observation. A read-only compatibility check is:

```sh
python3 install.py --openpilot-root /data/openpilot --check-only --mode lateral-test
```

Actual patch application saves the previous launch configuration, source
revisions, applied patches and documentation. Rebuild the matching sources
and Panda firmware through the normal comma build and signature-verification
path before use. No firmware-signature bypass is provided. For rollback,
reverse the saved patches in their recorded directories, restore the saved
configuration/documentation, and rebuild the previous matching firmware.

After a completed trip the existing read-only diagnostic command remains:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

Share the diagnostic together with the original rlogs. New diagnostics include
the raw and filtered driver torque, model timestamp/count, activation-request
timestamp and manual-rearm flag. They distinguish lane validation, EPS release,
preparation, ACK wait, ACK timeout and actual active torque.
