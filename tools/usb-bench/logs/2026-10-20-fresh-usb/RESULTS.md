# Fresh USB-only session - 20 October 2026

This record contains only this session's observations.

## Setup and upload

- User confirmed: 12 V adapter physically unplugged, COM USB connected,
  external 10 kohm STBY-to-GND resistor fitted.
- User reported initial pose: motor joint centered, pendulum down.
- Discovered USB port COM11, VID:PID 0403:6001, serial A5069RR4A.
- PlatformIO upload completed and esptool verified flash hashes.
- Esptool identified ESP32-S3 QFN56 revision v0.2 and embedded 8 MB PSRAM.
- Fresh firmware reports 16 MiB flash, 8 MiB physical PSRAM and
  8,386,263 bytes usable PSRAM. Build banner: Oct 20 2026 19:55:34.
- Firmware commands STBY LOW throughout. This is not a physical measurement
  of the driver header.

## Stationary baseline

`stationary_baseline.jsonl` and `.raw` preserve the capture. After the fresh
stop acknowledgement there are 29 complete telemetry frames:

- Motor encoder: count 0, cps 0, sampled A/B=0/0 throughout. This checks only
  the stationary record, not encoder motion, direction or resolution.
- I2C scan: only address 0x36 responded; subsequent pendulum reads succeeded.
- Pendulum status: MD=1, ML=1, MH=0 throughout. The sensor detects a magnet
  but reports weak magnetic field. AGC reached 128. Angle/magnet mounting
  validation has not passed. No relative zero was set.
- Current-sensor mean: 2505.88 mV across report means.
- Per-window ADC voltage sample SD: 57.7-67.6 mV. This measures the whole
  sensor/ADC/wiring path, not sensor noise alone.

## Fresh zero-current calibration attempt

`zero_attempt.jsonl` and `.raw` preserve this capture. Sent `e` to zero the
stationary motor count and `z` to collect a new calibration window.

Firmware result:

```text
ZERO rejected: N=463 mean=2506.5 mV SD=63.0 mV raw=2740..3303.
```

The mean is plausible, but noise exceeds the diagnostic SD limit of 30 mV.
No valid current offset or estimated-current result was established.
The 25 complete frames after the fresh stop acknowledgement still show
motor count 0 and weak pendulum magnet status.

Some buffered/partial serial fragments arrived on reopening COM11. Raw data
is retained. `fresh_analysis.json` excludes pre-acknowledgement data and only
counts anchored, complete telemetry lines. It supersedes the initial capture
summary's inflated 113 encoder-report count for `zero_attempt`.
The capture utility was updated to apply this synchronization to future runs.

## Pending physical checks

Asked the user for:

1. Confirmation of magnet installation/centering and approximate chip-to-magnet
   gap in millimetres.
2. Multimeter voltages at current-sensor VCC, pendulum-encoder supply and
   driver STBY, each relative to GND.

The user subsequently reported approximately **1.7 mm** magnet-to-chip gap.
Magnet centering/type has not been independently established. The user cannot
access the voltage test points because the electronics are enclosed; physical
supply/STBY checks are therefore deferred, not passed.

A pendulum-only motion capture (`pendulum_motion_gap17`) was started, with a
cue to move about 45 degrees either side of hanging down, hold briefly, and
return down while leaving the motor arm centered. Motion results are pending.
No driver-input sequence has been issued yet. Motor motion, physical
PWM/direction pin measurements, and restart behavior remain pending.

Every capture ended with `s`, acknowledged by the firmware, and the serial
port was closed. Keep 12 V disconnected. The motor power stage and loaded
current accuracy are outside this test.
