# Peugeot 5008 T15 passive logger

This temporary diagnostic branch targets a 2019 Peugeot 5008 P87 on a comma
four. It is based on the prebuilt passive PSA image already validated on a
Peugeot 308 T9, so installation does not require compiling openpilot on the
device.

## Install

During comma setup, enter:

```text
installer.comma.ai/polo2169/peugeot-5008-t15-logger
```

Use the comma PSA harness only. Do not reconnect a CANable or add parallel CAN
taps. Keep the vehicle parked during this first diagnostic test.

## Safety and ignition behavior

- Native pandad drops every application CAN transmission.
- Panda remains in `NO_OUTPUT`, including after an onroad transition.
- Active firmware queries are skipped.
- The branch does not contain a Peugeot 5008 fingerprint and cannot enable
  lateral or longitudinal control.
- PSA terminal 15 is read from CAN `0x348` on logical bus 0, byte 6, mask
  `0x40`. Three frames spanning at least 100 ms are required before ignition is
  asserted. A received zero clears it immediately.

This T15 mapping is proven on the 308 T9 and expected, but not yet proven, on
the 5008 P87. The always-on passive recorder stores CAN before ignition too, so
a mismatch can still be diagnosed.

## Capture sequence

1. Leave ignition off for 30 seconds.
2. Turn ignition on without starting the engine and wait 30 seconds.
3. Start the engine and wait 60 seconds.
4. Turn ignition off and wait another 30 seconds.
5. Do not drive until the CAN topology and fingerprint have been reviewed.

The passive recorder keeps rotating uncompressed cereal segments in:

```text
/data/psa-diagnostics
```

It retains up to 32 segments of 16 MiB and never uploads them automatically.
With SSH enabled, copy the complete directory from the comma:

```sh
scp -r comma@COMMA_IP:/data/psa-diagnostics ./peugeot-5008-psa-diagnostics
```

If T15 is recognized, openpilot also creates the normal onroad route. Share the
full `rlog.zst` segments rather than only `qlog`, because qlog heavily decimates
CAN timing.

Expected raw T15 states are:

```text
0x348 data[6] & 0x40 == 0x00  ignition off
0x348 data[6] & 0x40 == 0x40  ignition on
```
