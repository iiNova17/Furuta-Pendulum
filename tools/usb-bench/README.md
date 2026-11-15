# Furuta fresh USB bench test

Fresh baseline requested 20 October 2026. No previous hardware-test results
are carried forward. This project includes **both encoders** and the current
sensor. Fresh upload and stationary checks have started; see `CHECKLIST.md`
and `logs/2026-10-20-fresh-usb/RESULTS.md`.

## Software validation - 20 October 2026

- PlatformIO build: passed with espressif32 6.12.0 / Arduino 2.0.17.
- Program usage: 324,889 bytes flash; 20,024 bytes static RAM.
- ESP32-S3 firmware image: checksum/hash valid; 16 MB flash header confirmed.
- Native C++ checks: passed angle wrap boundaries (including every possible
  zero reference), sample statistics, and rejection of insufficient samples,
  rail clipping, voltage spikes, noise and implausible offsets.
- Source review: all STBY writes are LOW; no enable command exists.
- **Fresh upload: passed on COM11.** Stationary checks identified weak pendulum
  magnetic field and excessive ADC-path noise; current zero calibration was
  rejected. Motion and physical voltage/pin checks remain pending.

PlatformIO may print the generic board's N8 / 8 MB description before applying
the project's settings; this project's generated image is configured for
16 MB flash, with qio_opi memory configuration and PSRAM support.
The physical board's capacity still needs a fresh runtime check.

To rerun the portable math checks from a terminal with g++ installed:

```powershell
g++ -std=c++11 -Wall -Wextra -pedantic tests/bench_math_test.cpp -o .pio/bench_math_test.exe
.\.pio\bench_math_test.exe
```

## Power and wiring

Finish wiring with USB disconnected. For this firmware, physically unplug the
12 V adapter. Connect the computer to the board's **COM** connector for power
and serial data. Keep the external **10 kohm STBY-to-GND resistor** fitted.
The program has no command that enables the driver, and cannot sense whether
12 V is connected.

| Connection | ESP32 connection / destination |
| --- | --- |
| Driver VCC | 3V3 |
| Driver GND | Common GND |
| Driver PWMA / AIN1 / AIN2 | GPIO15 / GPIO16 / GPIO17 |
| Driver STBY | GPIO18 plus external 10 kohm to GND |
| Driver PWMB / BIN1 / BIN2 | GND |
| Driver BO1 / BO2 | Unconnected |
| Motor encoder pin 2 / pin 5 | GND / 3V3 |
| Motor encoder pin 3 A / pin 4 B | GPIO5 / GPIO6 |
| Current sensor VCC / GND | Verified USB-derived 5 V / common GND |
| Current sensor OUT | GPIO4 directly, as specified in project wiring |
| Pendulum encoder 3.3 V-compatible supply / GND | 3V3 / GND |
| Pendulum encoder SDA / SCL | GPIO8 / GPIO9 |
| Pendulum encoder DIR | GND |
| Pendulum encoder OUT / PGO | Unconnected |

All numbers are GPIO labels, not header positions. On a module with separate
3V3/5V pins, follow that module's 3.3 V supply instructions. I2C pull-ups must
go to 3.3 V; fit 4.7 kohm if absent. Motor encoder outputs may need external
4.7-10 kohm pull-ups to 3.3 V.

The full motor wiring may remain assembled, but unpowered:

```text
Adapter + -> switch -> driver VM       [ADAPTER PHYSICALLY UNPLUGGED]
Adapter - -> common GND
Driver AO1 -> current sensor IP+ -> IP- -> motor pin 6 (+)
Driver AO2 -> motor pin 1 (-)
```

Neither motor lead is GND. Retain the VM capacitors specified in the main
wiring brief: 100-470 uF (at least 25 V) and 100 nF near the driver.
Never connect 12 V to ESP32 power pins, driver VCC, or sensor VCC.

Measure the current sensor's actual VCC-to-GND voltage before calibration:
a header marked 5V is not proof of an available USB-powered output.
The sensor requires 5 V and should output about 2.5 V at zero current.
The direct ADC connection has no overvoltage protection; ADC attenuation
does not add protection. This USB test does not approve it for powered use.

## Build now; upload after wiring is finished

Open this folder in VS Code with PlatformIO IDE and select **Build**.
PlatformIO uses its standard package directory and keeps its cache/temp
storage in the ignored `.pio` folder. There are no extra sensor-library
dependencies.

PowerShell from this folder:

```powershell
.\bench.ps1 build
.\bench.ps1 ports
.\bench.ps1 upload -Port COM7
.\bench.ps1 monitor -Port COM7
```

Replace COM7 with the currently detected board port. No old port assignment
is assumed. The wrapper locates the installed PlatformIO and uses temporary
storage on the project drive. Standard `pio run`, `pio run -t upload`, and
`pio device monitor` also work from a configured PlatformIO terminal.

Monitor: **115200 baud**, UART0 via COM. DTR/RTS are disabled in configuration.
Close the monitor before uploading. If connection fails, hold BOOT, tap
RESET/EN, release BOOT, and retry. Do not upload while wiring is in progress.

## Commands

Commands are single lowercase characters; Enter is optional.

| Command | Action |
| --- | --- |
| `h` | Help |
| `b` | Chip, flash, PSRAM, heap, pin assignment |
| `p` | Pause/resume reports; sensing continues |
| `i` | I2C address scan; stops the driver-input sequence first |
| `a` | Set current pendulum position as relative zero, in RAM |
| `e` | Zero accumulated motor encoder count, in RAM |
| `z` | Collect a fresh one-second zero-current calibration |
| `t` | Start 12-second driver-input signal sequence |
| `s` or `x` | Stop signal sequence, outputs LOW; cancel active calibration |

No automatic motion or output sequence runs at startup.
No AS5600 configuration, zero-position, or OTP memory is programmed.
Restarting the board clears calibration and relative-angle zero.

## What the readings mean

Reports normally appear twice per second. Serial/I2C diagnostics can make
sampling slower; this is a bench utility, not a real-time control loop.

- **Board:** expected N16R8 capacities are 16 MiB flash and 8 MiB physical
  PSRAM. Usable/free PSRAM are separate values.
- **Motor encoder:** hardware PCNT x4 count, counts/second and sampled A/B.
  Count should change with gentle physical motion and reverse with direction.
  No motion is not proof of encoder failure. Do not force a non-backdrivable
  gearbox. Counts are not converted to arm angle without the actual gear ratio.
  Counter extension assumes fewer than 15000 edges between software polls;
  `maxPollGap` makes delays visible. This is for slow manual testing only.
- **Current sensor:** calibrated ADC millivolts, sample standard deviation,
  sample range and raw extrema. Raw and mV values use adjacent conversions.
  `z` requires at least 100 samples, mean 2200-2800 mV, SD <=30 mV,
  range <=200 mV, and raw values between 17 and 4078. These are diagnostic
  sanity thresholds, not proof of sensor identity, supply health, or accuracy.
  Rejected calibration leaves current unavailable. Accepted calibration uses
  `(mV - zero_mV) / 185` for the nominal 5 A sensor sensitivity.
  An invalid voltage window invalidates the offset.
- **Pendulum encoder:** I2C address 0x36, raw count 0-4095, absolute angle,
  magnet detected (`MD`), weak (`ML`), strong (`MH`), AGC and magnitude.
  Good diagnostic status is MD=1, ML=0, MH=0. Rotate the aligned magnet gently.
  `a` gives a signed relative angle in [-180, 180) degrees. It becomes an
  upright reference only if you physically set upright before sending `a`.
  `travel` unwraps motion through 0/360 and starts a new segment after a
  read/magnet fault or sample gap over 100 ms. Rotate less than 180 degrees
  between samples. Invalid angles are labelled, not counted as good motion.
- **Driver logic:** `STBY_cmd=0` is the software command, not measured voltage.
  The program does not have independent driver-pin or VM feedback.

AS5600 register definitions follow the
[manufacturer datasheet hosted by Mouser](https://www.mouser.com/datasheet/2/588/AS5600_DS000365_5_00-1877365.pdf):
STATUS at 0x0B, RAW ANGLE at 0x0C/0x0D, AGC at 0x1A and MAGNITUDE at 0x1B/0x1C.

## Driver-input test

Send `t` only with 12 V disconnected. Measure at the driver header:

| Time | AIN1 | AIN2 | PWMA | STBY |
| --- | --- | --- | --- | --- |
| First 4 seconds | HIGH | LOW | LOW | LOW |
| Next 4 seconds | LOW | HIGH | LOW | LOW |
| Last 4 seconds | LOW | LOW | 1 kHz, 50% duty | LOW |
| After completion / `s` | LOW | LOW | LOW | LOW |

HIGH should be near 3.3 V and LOW near 0 V. A meter may show approximately
1.65 V average for PWM; a scope/logic analyzer verifies frequency and duty.
This tests ESP32 output signals and wiring. It does not validate H-bridge
power outputs or the STBY enable path.

## What remains for a separate 12 V test

Motor rotation, torque, driver power-stage behavior, loaded current accuracy,
current limits and control performance cannot pass this USB-only test.
If the motor encoder shaft cannot be turned gently, its motion test also waits.
Before that separate firmware/test, review motor/driver current compatibility,
ADC input protection, and the physical disable arrangement. The project brief
reports roughly 2.2 A stall current versus 1.2 A driver average rating.
Do not connect 12 V merely because the serial readings look plausible.
