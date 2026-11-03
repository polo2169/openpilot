# Peugeot 5008 P87 — first test of the EPS admission fix

Hi Vyck,

I have updated the same branch with the activation fix:
https://installer.comma.ai/polo2169/peugeot-5008-p87-cristianku

Your logs showed that the old controller waited for EPS=3 before sending
LKA=4, so it never requested activation. This update first sends LKA=4 at
zero torque, then waits for a new physical EPS acknowledgement before
ramping torque. If the EPS does not acknowledge within one second, output
stays neutral and a new engagement request is required.

The raw CAN torque ceiling is now 90 at 50 km/h, decreasing linearly to 40
at 130 km/h. Admission and resumption require 300 ms of stable conditions,
three distinct valid lane models, zero-torque preparation and a new EPS ACK.
The controller also checks unfiltered driver torque, matching Panda's existing
threshold. The accelerator and fault gates remain in place.

89 local tests pass, including compiled native Panda safety. This fixes the
software sequence; the physical response of your EPS still needs checking.

1. Update while parked. Allow the complete build and matching Panda firmware
   update to finish, then reboot. Keep normal firmware-signature verification.
2. For this first test, use your known startup order: comma fully loaded
   before starting the engine, no dash fault, and Stay in Lane Assistance on.
3. In suitable conditions, above 70 km/h with clear lane markings, engage
   stock ACC and comma with pedals released and hands on the wheel. Please
   report whether actual steering assistance appears, the approximate speed,
   and any alert. An engaged/green display alone is not proof of steering.
4. If there is no assistance or a fault appears, disengage and finish that
   test. Do not raise torque or keep retrying to force an EPS acknowledgement.
5. Only if basic steering works normally, observe one ordinary driver-steering
   intervention or indicator pause and whether it resumes after conditions
   are clear. Automatic lane changing remains disabled. Please do not use this
   first test to investigate assistance below 50 km/h.

After parking at the end of the trip, run:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

Please send the new 5008-diagnostic.json and the original rlogs, together with
the installed commit (`git rev-parse --short HEAD`, run while parked). I will
check whether LKA=4 is sent and accepted, whether a new EPS=3 follows, and
whether nonzero torque reaches the output. If EPS remains at 1/2, the new
logs will distinguish the ACK timeout and other inhibitions.

The separate early-engine-start lane-assist fault is not claimed fixed by
this update; keep the known startup order for the initial comparison.
