# Peugeot 5008 P87 — report 5 corrections, 14 November 2026

Installation: https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

Report 5 confirms installation 6f534dc, 38 hash-matched rlogs and profile
marker 3 on every torque-control sample. The selected default v0 tune was
active. This revision identifies the profile with marker 4 on both PID paths.

The recorded six driver-return attempts include one successful physical
return. Two failed on light effort after a confirmed release; one failed
because a model published 0.152 ms after the control tick was treated as
invalid. The remaining two were interrupted by the indicator and brake.
All ten active-episode exits are explained: six driver overrides, two
planned EPS releases, one brake and one stock ACC cancellation. This does
not establish that every future interruption will have the same causes.

Model polling happens after the tick timestamp is captured. If a new frame
is at most 20 ms newer than the tick, use only an already observed past
snapshot, at most 150 ms old, until the new frame becomes current. Never
accept the future frame itself. No past snapshot, excessive lead, reversed
clock/model time, expiry or invalid current data cannot authorize lanes.
The 300 ms and three-distinct-model requirements are not relaxed.

After initial effort <=30 raw has been stable for 300 ms, light contact up
to the unchanged +/-45 override threshold retains release confirmation.
An actual override, steeringPressed or request-off clears it. This removes
false held-effort stops on a brief +/-33 after a real release, including
during the EPS ACK. Initial release above 30 is still refused. The original
two-second uncleared-effort and five-second total budgets are unchanged.

Residual hunting is observed around 1.5–1.9 Hz near 73 km/h and 0.7–0.9 Hz
near 127 km/h. Proportional response is reduced another 30% relative to
profile 3; friction compensation falls from 0.07 to 0.04 normalized torque.
Integral gain, geometric feedforward, raw torque ceilings, driver priority
and native slew limits are retained. Frozen-input command-band comparisons
show lower oscillatory requests, not a simulated improvement of the actual
vehicle trajectory. Physical comfort and bend tracking require another test.
The planned EPS cycle keeps its existing policy and hard-intervention stops.

The paragraphs about c15de92 below describe the earlier runtime diagnosis.

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
The new physical tracking, renewal and driver resumption require a vehicle comparison.

The P87-only changes are:

- Proportional feedback gain multiplied by 0.35; integral gain remains at 0.5.
- Normalized friction compensation capped at 0.04 (6 raw CAN counts),
  including after live parameter updates and manual overrides.
- Internal PID output limits matched to the existing speed-dependent native
  steering ceiling, instead of calculating against a normalized +/-1.
- Torque-state log version 4 identifies this P87 controller path.

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
disables the assistance function, not merely its hands-warning sound. The car's native
monitoring and the +/-45 raw driver-override threshold are retained.
The route contains eight active-interval exits associated with effort above
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

After a brief driver override, lateral steering can now return automatically
while the lateral request and stock ACC remain active. The operator must
expect this return after releasing steering effort. Any request-off cancels
the pending return. No ACC engagement or longitudinal command is generated.

Only an override observed during established physical EPS=3 assistance arms
this pause. Torque is immediately withdrawn using neutral LKA. Before the
new request, EPS=0 may be observed while waiting at zero; unsolicited EPS
loss from active assistance cannot arm recovery. Driver effort must fall to
<=30 raw (including filtered and unfiltered signals), steeringPressed must
clear, and this initial release must remain stable for 300 ms. Once confirmed,
light contact up to +/-45 retains it; actual override, steeringPressed and
request-off clear it. Lane perception must
be fresh. The EPS driver-activity bit STEERWHL_HOLD_BY_DRV is not a physical
hands sensor and is no longer required to be asserted or stable for driver
resumption. TRQ_LIMIT_STATE and STEERING_REBOOT_REQUEST must still be known
and zero, with fresh physical EPS feedback. No return bit is forced or sent.
The separate experimental scheduling gate for planned EPS renewal described
above is unchanged; removing the driver-resume activity condition does not
force a renewal when that scheduling gate is unavailable.
Then revalidate lanes for 300 ms / three distinct models, observe a fresh
released EPS=1/2 after the pause, prepare at zero and request a new physical
EPS=3 ACK before applying the normal bounded torque ramp.

A driver who has not released effort within two seconds requires manual
reengagement. The complete pause/return budget is five seconds from the
original override, with no reset on repeated effort or perception changes.
The first-ACK limit remains one second. Hard faults, pedals, indicators,
stale CAN/EPS feedback or a Panda rejection during the return latch a stop.
After LKA=4, any eligibility loss also latches a stop. No failed request is
automatically retried. Another override within five seconds of a successful
return requires manual reengagement to avoid repeated steering handovers.
Successful returns, attempts, phase, timestamps and gate veto are logged in
psa_p87_status. This feature has not yet been physically validated.

Validation includes 164 targeted tests: the real PID and vehicle model,
default/forced runtime selection of version 0 or 1, live/manual friction
bounds, unaffected other PSA tuning, current dynamic
limits, real controller/native safety integration, repeated planned cycles
in both directions, abort/timeout checks, and the existing ACK,
fault, pedal and rate-limit regressions. Tests also replay the observed
release pattern and temporary model loss through the real host controller
and native Panda checks. The bounded driver-return tests include actual
CAN/native ACK, off-tick override pulses, released-effort stability, deadlines,
request cancellation, repeated overrides and hard vetoes. Driver resumption
is exercised with the activity bit zero, intermittent, absent and falling
after the new request, while EPS torque-limit/reboot vetoes remain effective.
The tests also cover causal model snapshots, expiry, future/reversed time,
release hysteresis and oscillatory PID inputs. Native C is compiled for the tests.
A version-0 frozen-input comparison reconstructs the installed PID and
shows weaker new requests;
it does not simulate the new vehicle trajectory or prove physical stability.
The next test must check both reduced hunting and adequate tracking in bends.

This revision changes host controller code. The matching Panda safety source
is unchanged. Allow the normal on-device build and firmware-signature checks
to finish before use. Rollback to installation commit 6f534dc restores profile 3 and the previous
release/timestamp gates. Keep
normal firmware verification enabled.

After parking at the end of the comparison, collect the read-only diagnostic:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

Share the installed commit, diagnostic and original rlogs. Keep the known
startup order (comma fully loaded before engine start) for the first
comparison; this revision does not claim to fix the separate startup fault.
