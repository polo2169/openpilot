# Peugeot 5008 P87 — calmer PID test release, 5 November 2026

Installation: https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

The previous 3547d2d release successfully activated steering. Its new rlogs
show repeated steering/yaw oscillation around 1 Hz, including intervals
without native torque-ceiling saturation. This revision reduces the P87
controller's feedback and friction response. The resulting physical tracking
still needs a vehicle comparison; no full comma-target build or device
installation has been performed by this release process.

The P87-only changes are:

- Proportional and integral feedback gains multiplied by 0.5.
- Normalized friction compensation capped at 0.07 (10.5 raw CAN counts),
  including after live parameter updates and manual overrides.
- Internal PID output limits matched to the existing speed-dependent native
  steering ceiling, instead of calculating against a normalized +/-1.
- Torque-state log version 2 identifies this P87 controller path.

The learned acceleration factor and geometric feedforward are retained.
Stored user preferences are not rewritten. The supplied route used standard
PID with NNLC and jerk-aware torque modes disabled; keep those same modes
for the comparison. The test instructions are in
PEUGEOT_5008_TEST_PLAN_20261105.md.

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

There is no periodic EPS reset in the P87 controller. The baseline physical
EPS=3 interval lasts 297.5 seconds. This does not support assuming the 308's
ten-second renewal requirement on the P87. The final EPS withdrawal follows
about thirteen seconds with the native hands flag at zero; that correlation
does not identify the EPS's complete internal decision. No hands or driver
feedback is fabricated, and native withdrawal remains effective.

Validation includes 100 targeted tests: the real PID and vehicle model,
live/manual friction bounds, unaffected other PSA tuning, current dynamic
limits, real controller/native safety integration, and the existing ACK,
fault, pedal and rate-limit regressions. Native C is compiled for the tests.
The 52 original rlogs match their supplied diagnostic hashes. A frozen-input
comparison reconstructs the previous PID and shows weaker new requests;
it does not simulate the new vehicle trajectory or prove physical stability.
The next test must check both reduced hunting and adequate tracking in bends.

This revision changes host controller code. The matching Panda safety source
is unchanged. Allow the normal on-device build and firmware-signature checks
to finish before use. Rollback to installation commit 3547d2d restores the
previous host response with its matching source set; do not bypass normal
firmware verification.

After parking at the end of the comparison, collect the read-only diagnostic:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

Share the installed commit, diagnostic and original rlogs. Keep the known
startup order (comma fully loaded before engine start) for the first
comparison; this revision does not claim to fix the separate startup fault.
