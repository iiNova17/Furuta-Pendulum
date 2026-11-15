# Fresh USB-only hardware test record

Only fresh-session observations are recorded below. No old observations are
imported. Detailed evidence: `logs/2026-10-20-fresh-usb/RESULTS.md`.

Date: 20 October 2026; capture timestamps are UTC in JSONL
Board marking: not rechecked; esptool identified ESP32-S3 QFN56 revision v0.2
Serial port: COM11, USB VID:PID 0403:6001
Firmware build: fresh USB bench v1, Oct 20 2026 19:55:34; upload hashes verified
Instrument(s): USB serial capture; user cannot access enclosed voltage test points

| Check | Expected observation | Actual / evidence | Status |
| --- | --- | --- | --- |
| Motor supply | 12 V adapter physically disconnected | User explicitly confirmed | USER CONFIRMED |
| Reset protection | External 10 kohm STBY pull-down fitted | User explicitly confirmed | USER CONFIRMED |
| USB/serial | COM connection, readable 115200-baud banner | Fresh firmware uploaded; banner read on COM11 | PASS |
| Board capacity | 16 MiB flash, 8 MiB physical PSRAM | Runtime reports both expected capacities | PASS |
| Logic supply | Measure approximately 3.3 V at driver and encoders | Test points inaccessible inside enclosure | DEFERRED / UNVERIFIED |
| Current-sensor supply | Measure approximately 5 V at sensor VCC/GND | Test points inaccessible inside enclosure | DEFERRED / UNVERIFIED |
| STBY at idle/reset | Measure approximately 0 V at driver header | Test points inaccessible; firmware command is not a measurement | DEFERRED / UNVERIFIED |
| Current sensor at rest | OUT near 2.5 V; compare meter with ADC | ADC mean about 2.506 V; SD 57-68 mV; meter comparison pending | NEEDS INVESTIGATION |
| Current zero command `z` | Accepted fresh sample window; I_est near zero | Rejected: 463 samples, 2506.5 mV mean, 63.0 mV SD | DID NOT PASS |
| Pendulum I2C `i` | Address 0x36 responds | ACK at 0x36 and successful register reads | PASS |
| Pendulum magnet | MD=1, ML=0, MH=0 | MD=1, ML=1, MH=0: field too weak | DID NOT PASS |
| Pendulum motion | Angle changes smoothly both directions | | PENDING |
| Pendulum wrap/zero | `a` zero; small motion through wrap stays continuous | | PENDING |
| Motor encoder movement | Counts change and reverse with actual gentle motion | | PENDING |
| Motor encoder at rest | No unexplained drift; record A/B levels | Count=0, cps=0, A/B=0/0 in stationary captures; motion still unverified | STATIONARY ONLY |
| Driver IN1/IN2 `t` | Physical HIGH/LOW sequence agrees with table | | PENDING |
| Driver PWMA `t` | Scope: 1 kHz/50%; meter alone checks average only | | PENDING |
| STBY during `t` | Measure LOW throughout | | PENDING |
| Automatic signal-test stop | All driver control signals LOW after 12 s | | PENDING |
| Manual stop `s` | Signals return LOW before sequence completes | | PENDING |
| Reboot behavior | Driver disabled; calibration unset; no automatic test | | PENDING |

If the gearbox will not turn gently, mark encoder motion **DEFERRED**, not
failed or passed. Disconnected/missing sensors remain pending or fail their
specific check; an I2C ACK or plausible voltage alone is insufficient.

Motor power-stage operation and current measurement under load: **NOT TESTED
BY THIS FIRMWARE**. A new powered-test project is needed for the later stage.
