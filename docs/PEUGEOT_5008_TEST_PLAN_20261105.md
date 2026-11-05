> Historical plan for the PID-only release f7e716e. The current combined
> PID/EPS-renewal comparison is PEUGEOT_5008_TEST_PLAN_20261106.md.

# Peugeot 5008 P87 — steering response comparison, 5 November 2026

Hi Vyck,

Thank you: your logs confirm that steering activation now works. I identified
the long uninterrupted interval: about 4 minutes 57 seconds of physical EPS=3.
The logs also show repeated steering/yaw oscillation around 1 Hz.

This update softens the P87's torque-controller response: proportional and
integral gains are halved, friction compensation is capped at 0.07 normalized
(10.5 raw CAN counts), and the PID uses the same speed-dependent available
range as the car controller. The raw ceiling remains 90 at 50 km/h,
decreasing linearly to 40 at 130 km/h. Stock ACC and Panda authority gates
are unchanged. Offline comparisons show weaker corrections on the recorded
inputs; the new physical lane tracking still needs a vehicle comparison.

Please leave the car's "Stay in Lane Assistance" / LKA setting ON.
Switching LKA OFF disables the assistance function, not just the sound of
the hands-on-wheel warning. Keep your hands on the wheel and respond to the
car's warnings. The native hands monitoring and driver-override threshold
remain active. In the logs, seven interruptions followed steering effort
above that threshold; it has not been raised by this update.

There is no periodic EPS-reset cycle in this P87 port. The 308's ten-second
renewal cycle has not been copied to your car. Your nearly five-minute active
interval does not support assuming the same ten-second limit. The final EPS
withdrawal was preceded by about thirteen seconds with the native hands flag
at zero; that is a correlation, not a confirmed internal fault diagnosis.

For the next comparison:
1. Update while parked, allow the normal build and firmware verification to
   complete, and reboot. Use the same known startup order as before.
2. Keep LKA ON and the optional NNLC/jerk-aware torque modes OFF, as they were
   in your supplied route, so this tests the revised PID in comparable conditions.
3. With hands on the wheel and suitable clear lane markings, compare normal
   assistance near 90 km/h first. Only if steering feels normal, compare near
   110 km/h. Report whether the side-to-side corrections are reduced, whether
   it drifts or feels too weak in bends, and any disengagement or dash alert.
4. Disengage and end the comparison if steering remains uncomfortable or a
   fault appears. Please do not release the wheel to test hands-off operation.

After parking, collect:

```sh
cd /data/openpilot
PYTHONPATH=. python openpilot/tools/audit_5008_engagement.py /data/media/0/realdata --latest --output /data/5008-diagnostic.json
```

Please send the installed commit, the new diagnostic and original rlogs.
I will compare oscillation, requested/applied torque and the exact causes of
interruption. This release has not yet been physically validated on your car.
