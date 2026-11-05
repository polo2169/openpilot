# Peugeot 5008 P87 — EPS renewal and steering comparison, 6 November 2026

Hi Vyck,

The next update combines the calmer torque response with a planned EPS renewal
cycle for the Peugeot 5008 P87. It is available on the same installation branch:
https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

The torque ceiling remains 90 raw CAN counts at 50 km/h, decreasing linearly
to 40 at 130 km/h. The softer PID gains and reduced friction compensation
from the previous update are retained. Those changes target the repeated
side-to-side corrections visible in your logs.

The EPS cycle becomes eligible after ten seconds of continuously authorized
assistance. It is deferred until speed is at least 50 km/h, the current and
predicted path are straight, lane predictions are fresh and stable, the native
hands signal has been continuously present for at least one second, and steering
command magnitude is small. Ten seconds is an experimental scheduling interval,
not a timeout established by your 5008 logs. There is no forced release in a bend.

When those conditions hold, the command ramps down to zero, sends neutral LKA,
and waits for an actual EPS release to state 1/2. It then repeats the 300 ms /
three-new-model validation, zero-torque preparation and fresh physical EPS=3
acknowledgement before torque returns. EPS=0 is tolerated only while waiting
for our own planned release; it never permits steering or starts a recovery.
The driver must steer during the intentional zero-assistance interval.

A missing release, missing ACK, fault, stale data, driver override, indicator,
changed corridor or missing hands indication during renewal aborts it and
requires a new engagement. The first stop reason is retained in the log.
During ordinary active assistance, a missing native hands indication defers
renewal. Returning feedback must pass the full hands/corridor stability checks
again. This cycle is not used to recover an unsolicited EPS dropout; native
authorization and real driver-feedback checks remain required.

Please keep the car's "Stay in Lane Assistance" / LKA setting ON. Switching it
OFF disables the assistance function, not just the hands-warning sound.
Keep your hands on the wheel and respond to all warnings. Driver override,
pedal/fault checks and native hands monitoring remain active. Stock ACC and
the Panda safety source are unchanged; no hands or driver-feedback messages
are fabricated.

119 targeted tests pass, including repeated host/native-Panda cycles in both
directions and interruption/timeout checks. The new cycle and physical tracking
have not yet been validated on your vehicle; no full comma-target build has
been run here.

For the comparison:

1. Update while parked, allow the normal build and firmware verification to
   finish, reboot, and keep the known startup order (comma ready before engine).
2. Leave LKA ON. Keep the optional NNLC and jerk-aware modes OFF, as in your
   previous route, to compare the same PID path.
3. With hands on the wheel, first compare normal assistance near 90 km/h on
   a suitable straight with clear markings. Expect a brief intentional loss
   of steering assistance during a successful cycle; continue to steer.
   Compare near 110 km/h only if the initial response feels normal.
4. Report sway, drift, weak assistance in bends, how the renewal feels, and
   any interruption or warning. End the comparison if control is uncomfortable
   or a fault appears. Do not deliberately release the wheel to trigger alerts.

After parking, collect the installed commit, diagnostic and original rlogs:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

The diagnostic now records renewal count, phase, eligibility veto, native hands
indication, request time and manual-rearm state. We need those rlogs to confirm
the real release/ACK sequence and compare the new steering response.
