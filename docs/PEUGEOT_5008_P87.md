# Peugeot 5008 P87 — calmer PID and planned EPS renewal, 8 November 2026

Installation: https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

The tested c15de92 route records torque controller version 0 on all 157,514
controls samples. Sunnypilot's default runtime selector replaces the version-1
controller with version 0. The previous P87 tuning was implemented only in
version 1, so it did not affect the selected controller. This revision applies
the P87 feedback/friction/dynamic-limit profile to the selected version 0 as
well. Both paths identify the profile with torque-state log version 3.
The actual default selector is now exercised in the regression tests.

The 27 supplied rlogs match their diagnostic hashes. They show steering/yaw
hunting, particularly around 1.3–1.8 Hz near 70 km/h. Two planned renewals
both aborted after about 60 ms on changing perception, before EPS release
finished. The release itself follows 3 -> 2 -> 0 -> 2; the last transition
arrives about 1.45 s or 0.67 s after neutral in those two examples.
These observations motivate the bounded zero-torque wait described below.
The new physical tracking and renewal still require a vehicle comparison.

The P87-only changes are:

- Proportional and integral feedback gains multiplied by 0.5.
- Normalized friction compensation capped at 0.07 (10.5 raw CAN counts),
  including after live parameter updates and manual overrides.
- Internal PID output limits matched to the existing speed-dependent native
  steering ceiling, instead of calculating against a normalized +/-1.
- Torque-state log version 3 identifies this P87 controller path.

The learned acceleration factor and geometric feedforward are retained.
Stored user preferences are not rewritten. The supplied route used standard
PID with NNLC and jerk-aware torque modes disabled. Vehicle comparison of
the revised PID and EPS renewal remains necessary.

| Measured wheel speed, km/h | Maximum absolute raw CAN torque |
| --- | --- |
| 50 | 90 |
| 70 | 77 |
| 90 | 65 |
| 110 | 52 |
| 130 | 40 |

The linear 90-at-50 / 40-at-130 curve is unchanged. Magnitudes are rounded
down and clamp to the knot values outside 50–130. These are command counts,
not measured Nm. Normalization remains 150, native slew limits remain 8/38
per 50 ms, and the native real-time allowance remains 75.

The working admission protocol is retained: stable conditions for at least
300 ms and three distinct valid lane models, fresh released EPS state 1/2,
zero-torque preparation, then LKA state 4 at zero torque. Only a new physical
EPS=3 after the request permits a torque ramp. A missing ACK times out after
one second. Actual Peugeot ACC activity and all existing driver, pedal,
fault and freshness gates remain required. Faults and unconfirmed withdrawal
retain manual rearm. Software eligibility remains 30–140 km/h; this is not
physical validation of steering below 50.

Leave the car's Stay in Lane Assistance / LKA setting ON. Switching it OFF
disables the assistance function, not merely its hands-warning sound. Native
hands monitoring and the +/-45 raw driver-override threshold are retained.
The route contains seven active-interval exits associated with effort above
that threshold; it has not been increased to hide these interventions.

This revision adds a planned EPS renewal opportunity after ten seconds of
active assistance. Ten seconds is an experimental interval, not a measured
5008 deadline: the baseline physical EPS=3 interval lasts 297.5 seconds.
Renewal waits for speed >=50 km/h, fresh stable
lanes and a straight current/predicted five-second corridor, small command
magnitude (<=15 raw), and native hands indication stable for at least one
second. Missing hands indication defers renewal; returning indication must
pass the full stability checks again. Before a new activation request,
missing hands feedback keeps renewal neutral; after the request, loss
aborts it. Ordinary native hands monitoring remains effective.

On an eligible 20 Hz send, reduce command magnitude by at most four raw
counts per tick until zero, send LKA=2/factor=0, and wait for a new physical
EPS=1/2. Only during this intentional release may EPS=0 be observed without
immediately aborting; it cannot authorize preparation or torque. A release
must arrive within one second. Revalidate for 300 ms and three distinct
models, prepare at zero, request LKA=4 at zero and require a fresh EPS=3
before ramping torque. After the first released EPS=1/2 is observed, a
temporary EPS=0 before the new request keeps output neutral and postpones
preparation; it cannot authorize steering. The full renewal has a three-second budget, including
the existing one-second first-ACK limit. Count only successful new ACKs.
The driver must steer during the intentional zero-assistance interval.

CAN/EPS faults or stale feedback, pedals, driver override, indicators and
Panda rejection still latch a neutral stop during renewal. Before a new
LKA=4 request, transient perception/corridor uncertainty or missing native
hands feedback waits at zero instead of aborting immediately. Preparation
cannot restart until real EPS authorization, hands stability and fresh lane
validation are restored. The original three-second deadline is never reset.
Once LKA=4 has been requested, a gate loss or missing first ACK latches the
stop; no automatic request retry extends that clock. Unsolicited active EPS
withdrawal cannot start a cycle. No artificial driver/hands feedback is sent.

Validation includes 131 targeted tests: the real PID and vehicle model,
default/forced runtime selection of version 0 or 1, live/manual friction
bounds, unaffected other PSA tuning, current dynamic
limits, real controller/native safety integration, repeated planned cycles
in both directions, abort/timeout checks, and the existing ACK,
fault, pedal and rate-limit regressions. Tests also replay the observed
release pattern and temporary model loss through the real host controller
and native Panda checks. Native C is compiled for the tests.
A version-0 frozen-input comparison reconstructs the installed PID and
shows weaker new requests;
it does not simulate the new vehicle trajectory or prove physical stability.
The next test must check both reduced hunting and adequate tracking in bends.

This revision changes host controller code. The matching Panda safety source
is unchanged. Allow the normal on-device build and firmware-signature checks
to finish before use. Rollback to installation commit c15de92 restores the
previous runtime response and renewal behavior with its matching source set. Keep
normal firmware verification enabled.

After parking at the end of the comparison, collect the read-only diagnostic:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

Share the installed commit, diagnostic and original rlogs. Keep the known
startup order (comma fully loaded before engine start) for the first
comparison; this revision does not claim to fix the separate startup fault.
