# Peugeot 5008 P87

The original installed release uses observation/dashcam mode:
`PSA_5008_LATERAL_TEST=0`, `NO_OUTPUT`, `FAKESEND=1`. Pilot Assist cannot
activate comma steering while that configuration is selected.

The 12 October correction reads the driver's pedal from eight-byte `0x228`
on bus 0, byte 2 scaled by 0.5%. `0x208` is calculated engine demand, which
can be positive during stock ACC with the driver pedal released. Python and
the dedicated Panda safety profile use the same pedal source. A generated
P87 DBC preserves the five-byte definition on other PSA cars. Length,
checksum, counter, range and freshness are checked; invalid feedback blocks
steering. Driver pedal release alone never reengages Panda.

`PSA_5008_ROLLING_START=1` retries CAN identification after a transient failure,
up to five windows of at most 2.5 seconds each. Recognition works at any speed,
never forces the model and never engages. You can set off while initialization
completes; assistance requires the normal model/vehicle checks to pass.

The updated installation branch selects `PSA_5008_LATERAL_TEST=1` for the
requested lateral experiment, removing the observation-only dashcam setting.
Install the update while parked, let the build and matching Panda firmware
update complete, and restart. Enable the openpilot toggle. To return to
observation, set the flag to `0` and restart; this selects NO_OUTPUT and
FAKESEND again. The selected PSA safety parameter is `0x5008`. Only
native-template `0x3F2` steering is permitted; stock ACC keeps longitudinal
control. No radar programming, automatic resume, fake driver torque or
hands-on-wheel messages are transmitted.

Panda enforces stock ACC engagement, ignition, feedback within 250 ms,
absence of ACC/brake faults, released pedals, 52–140 km/h, driver intervention
and torque/rate limits. The host requires a fresh camera LKA template and
EPS acknowledgement before requesting torque.

The road recordings contain 5,625,246 CAN frames over about 21 minutes, up to
83.22 km/h. Offline checks replay CarInterface, compiled Panda pedal hooks,
all 128,255 pedal frames and 197 identifications while moving. All 41 targeted
interface, ignition, fingerprint and compiled Panda tests pass. Panda was in
NO_OUTPUT throughout, so these recordings do not validate physical steering,
relay-open topology or lateral calibration.

The owner reports replacing the failing battery and no faults since; no ACC
or brake fault appears in the new recordings. Wheelbase is 2.84 m; mass,
steering ratio, tire stiffness and lateral tuning remain provisional.
Confirm engine/trim and validate steering sign, EPS response and driver
interventions on a closed course. A full comma build, firmware flash and
physical lateral validation remain to be performed.
