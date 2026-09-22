# Peugeot 308 T9 2018: comma mici dashcam branch

> This document describes the receive-only ancestor of the branch. The current
> experimental active-control version is documented in
> [PEUGEOT_308_T9.md](PEUGEOT_308_T9.md).

This device branch is based on the exact installed openpilot v0.11.1 release,
`70e157462304e5ce7d03ffbec6cb7f45bf347bb7` (`release-mici`, AGNOS 18.4).
It integrates the receive-only T9 port from `polo2169/opendbc` commit
`853001dc0a35642700c570d2abad860a8eff3145` into the release's vendored opendbc.
Existing native binaries, driving models, vehicle safety firmware and AGNOS
requirements are unchanged. The CAN parser and checksum extension are Python.

The reference car is a 2018 Peugeot 308 II T9 facelift with conventional cruise
control. Other variants, the physical comma harness and CAN bus assignment
still need vehicle validation. ESP32 fixtures are not a comma route.

## Recording constraints

- The T9 interface sets `dashcamOnly`, `noOutput`, and disables longitudinal
  control. Its controller returns zero applied actuators and no CAN frames.
- The device launcher sets `SKIP_FW_QUERY=1`: no VIN or ECU firmware queries
  are sent during identification. CAN fingerprinting remains automatic.
- The launcher also sets `PSA_DASHCAM_ONLY=1`. The car process then always uses
  passive mode and `noOutput`, even if another car is identified or the user
  enables the openpilot toggle. This is a recording-only device branch.
- CAN acknowledgements and any harness forwarding are distinct from sending
  application frames. The hardware topology still requires validation.
- No joystick, HIL, torque/RVV experimental profile, fixed fingerprint, or
  automatic vehicle actuation is installed by this branch.

## Offline validation on the comma

Run from this checkout using `/usr/local/venv/bin/python` and `PYTHONPATH` set
to this checkout. Tests mock live messaging where necessary and replay bundled
RX fixtures; they do not operate the Panda or publish control messages.

```sh
/usr/local/venv/bin/python -m unittest \
  opendbc.car.psa.tests.test_peugeot_308_t9 \
  openpilot.selfdrive.car.tests.test_psa_dashcam \
  opendbc.car.tests.test_can_fingerprint
```

Before the first drive, verify the harness, detected fingerprint, bus assignment,
CAN validity, passive CarParams and actual Panda `noOutput` state while parked.
Drive manually to collect the first route. Preserve its full rlogs for CAN
analysis; a qlog is a decimated subset. Vehicle validation is pending until
those observations are recorded.
