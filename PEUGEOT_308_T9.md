# Peugeot 308 T9 experimental RVV and lateral control

This branch contains the complete source currently installed and USB-validated
on one 2018 Peugeot 308 II T9. It is based on openpilot commit
`6c928b70b499fae53c3791384e44886f4c352842`.

The implementation is specific to the recorded CAN topology, messages and
checksums of that vehicle. It has not been validated as a drop-in port for
another Peugeot or Citroen model.

## Current behavior

- RVV is available from 40 km/h. Lead following changes the factory cruise
  setpoint and does not command the brakes directly.
- Lateral control is available from 67.1 km/h up to 140 km/h.
- Either turn signal pauses lateral torque immediately. Control resumes at zero
  torque after 0.5 seconds of consecutive fresh and plausible lane estimates.
- Driver effort above +/-15 raw units pauses lateral control. +/-15 remains
  accepted, and the same lane-quality gate applies before resuming.
- The requested curvature is limited by the experimental 0.63 m/s2 and +/-15
  raw envelope. High-speed feedback gains are reduced above 90 km/h.
- The RVV follower projects four seconds of closing distance and limits
  setpoint reduction to 1 km/h per 100 ms. There is no two-second target
  deadline.
- An EPS deactivate/reactivate experiment inspired by cristianku is available
  through `PsaT9EpsCycleTest`. It defaults to OFF and has additional feedback,
  lane and straight-road gates.

Lateral and RVV permissions are kept separate in the host and Panda safety
paths. A real EPS authorization loss, fault or interrupted session remains
latched and requires a physical factory-cruise OFF/ON cycle.

## Validation status

The installed source passed 381 tests with 6 skips, plus two native C++ test
benches. The matching host binary, Panda H7 firmware and bootstub were built on
the comma, then 30 post-boot source, service, parameter and safety invariants
passed with USB power, the vehicle harness disconnected and Panda in
`noOutput`.

Those checks do not validate the latest +/-15 lateral envelope, EPS cycle or
automatic lateral resume during road use. Treat all three as experimental.

## Sharing a bounded dataset

`system/psa_recorder.py` records bounded local segments under
`/data/psa-diagnostics`. It subscribes to existing services and does not open
Panda or transmit CAN.

Use the exporter from this repository to package up to four complete segments,
the matching T9 source files and a SHA-256 manifest:

```sh
python3 tools/export_t9_dataset.py /path/to/psa-diagnostics \
  --segments 4 \
  --label test-308-t9 \
  --output /tmp/t9-dataset-to-share.tar.gz
```

The recorder does not subscribe to camera or GPS services. Raw CAN,
`CarParams`, logs and device state can still contain vehicle or device
identifiers. The event stream remains byte-for-byte unchanged for replay, so
review the archive before publishing it.

Small examples without a raw CAN stream are included in
`examples/t9_dataset/`. `truck_approach.sample.csv` contains decoded points
from the original 130 km/h approach to a slower truck and compares the old and
new offline calculations; it is not a road result from this final version.
