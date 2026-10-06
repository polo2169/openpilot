# Peugeot 5008 P87

This branch adds the 2019 5008 P87 specimen recorded by Vyciokas to
Cristianku's stable `psa-torque-sunny` base. It identifies the vehicle using
90 observed CAN identifiers, reads the common PSA signals with P87 adaptations,
and uses terminal 15 for ignition. Firmware interrogation is skipped.

The default configuration in `psa_5008_env.sh` sets `PSA_5008_LATERAL_TEST=0`.
The interface is a recognized PSA dashcam with `NO_OUTPUT`; `FAKESEND=1` also
blocks application transmissions. This first stage verifies the CAN data and
vehicle faults without issuing commands.

The optional lateral experiment requires changing `PSA_5008_LATERAL_TEST` to
`1`, rebuilding, flashing the matching Panda firmware, and restarting. This
selects PSA safety parameter `0x5008`. Only native-template LKA commands on
`0x3F2` are authorized; there is no radar programming, longitudinal control,
automatic ACC resume, fake driver torque or hands-on-wheel message.

Panda independently enforces stock ACC engagement, fresh valid feedback,
ignition, absence of ACC/brake faults, pedal inhibition, a 52–140 km/h speed
envelope, driver intervention, and torque/rate bounds. The host requires a
fresh native bus-2 LKA template and EPS acknowledgement before sending torque.

The wheelbase is 2.84 m, from
[Peugeot's P87 press pack](https://www.media.stellantis.com/em-en/peugeot/press/press-pack-resume-the-all-new-peugeot-5008-a-whole-new-dimension-for-suvs).
Mass, steering ratio, tire stiffness, torque tuning and actuator delay remain
provisional. Confirm the engine/trim and validate the harness with its relay
open, steering sign, EPS response and driver takeovers on a closed course.

Offline validation: 358 fingerprint replay windows passed, 24 targeted
interface/safety/ignition tests passed, and observation generated zero CAN
commands even with an active lateral request. The supplied captures were
stationary. A full comma build, firmware flash and vehicle lateral validation
remain to be performed.

The source logs contain a native ACC fault just after cranking alongside a
Panda voltage dip to 6.26 V. Preserve ABS/ESP and radar/ACC DTCs and diagnose
that fault before evaluating the lateral experiment.
