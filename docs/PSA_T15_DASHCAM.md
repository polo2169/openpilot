# Peugeot 308 T9: ignition for passive recording

This device branch derives host-side `pandaStates.ignitionCan` from T15 in
standard eight-byte CAN message **0x348 on logical bus 0**:
`(data[6] & 0x40) != 0`. It is enabled only when `PSA_DASHCAM_ONLY=1`.
The physical ignition-line value remains the Panda hardware value.

The decoder accepts at least three received frames spanning 100 ms before
asserting ignition. During confirmation, a gap over 250 ms restarts confirmation.
An explicit T15=0 clears ignition immediately. No matching frame for one second,
unhealthy communication or a disconnected harness clears the state and requires
fresh confirmation. Returned/rejected frames, other buses, addresses and lengths
cannot set or refresh ignition. No engine-RPM threshold is used.

The value is applied in native pandad before publishing `pandaStates` and before
the existing power-saving decision. The standard hardwared startup conditions
still apply. This avoids a firmware change and avoids forcing `deviceState.started`
or writing an ignition override into Params. The raw Panda firmware health packet
itself does not acquire a PSA ignition implementation; the derived value is host-side.

In this dedicated mode, native pandad drops all application CAN transmissions
and holds `noOutput` through onroad transitions and fingerprinting, without
entering ELM327. The existing passive CarParams and controller guards remain.
Outside `PSA_DASHCAM_ONLY=1`, the original ignition and safety configuration paths
are retained. This is a dashcam port, not active vehicle control.

## Validation

The three comma recordings retrieved on 2026-09-16 contain 3,886,326 CAN frames.
The detector was replayed against 442,979 relevant frames extracted from those
recordings, including frames from other buses. All twelve T15 transitions were
reproduced, with the intended 100–150 ms assertion delay and immediate clearing
on a received zero. All twenty recorded `3 -> 4 -> 5 -> 3` engine-state cycles
compatible with Stop & Start kept ignition asserted.

The standalone C++ test also covers wrong IDs/buses/lengths, disabled mode,
same-timestamp buffered bursts, insufficient confirmation, stale data, clock
regression, communication loss, harness loss and engine stops with T15 still set.

```sh
clang++ -std=c++17 -Wall -Wextra -Werror -I. selfdrive/pandad/tests/test_psa_t15.cc -o /tmp/test_psa_t15
/tmp/test_psa_t15
```

Optional input to the same executable is a text file with one received frame
per line: `monotonic_ns address_decimal bus_decimal data_hex`. Replay operates
only on bytes in the file and never opens a CAN transport or messaging publisher.

The in-vehicle check after installation should verify that contact starts
recording, a Stop & Start pause does not end it, and contact off ends it. Full
accessory-mode coverage remains a vehicle-validation task. A separate CAN leg
has receive errors in the recordings; this detector does not diagnose their cause.

## Comparison with cristianku

Reviewed reference:

- [openpilot `psa-torque-sunny-testing`](https://github.com/cristianku/openpilot/tree/a4a565a52ff409c20e0f3ff225b924ba687c688a),
  which pins opendbc commit `9d2e5bbd33013932961776ea3f6c5b4bd3cab56b`.
- [PSA ignition hooks](https://github.com/cristianku/opendbc/blob/9d2e5bbd33013932961776ea3f6c5b4bd3cab56b/opendbc/safety/ignition.h):
  its 0x348 rule uses `data[5] & 0x06`, the engine-state byte. Its 0x432 rule
  uses the electrical-network state with a counter check.
- [PSA DBC](https://github.com/cristianku/opendbc/blob/9d2e5bbd33013932961776ea3f6c5b4bd3cab56b/opendbc/dbc/psa_aee2010_r3.dbc)
  defines `P372_T15_st` at bit 54 of 0x348, consistent with this implementation.
- [Platform definitions](https://github.com/cristianku/opendbc/blob/9d2e5bbd33013932961776ea3f6c5b4bd3cab56b/opendbc/car/psa/values.py)
  list 208, 508, 3008 and C4 SpaceTourer, but no explicit 308 T9 entry.

Thus the reference supports the protocol mapping, but does not independently
validate complete active-control compatibility for the 308 T9. Its engine-state
rule is different from T15: our captures show 546 samples where they differ,
mostly while contact is on before cranking or during transitions. The T15 choice
here follows the user's recorded contact cycles and keeps Stop & Start separate.
