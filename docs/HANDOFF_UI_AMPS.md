# Handoff: Furuta Lab UI/UX revision + current in amperes

Written 2026-11-07 from a claude.ai review session; updated the same night with Nedal's
decisions. Read all of it before starting. Show Nedal the plan before large edits.

Safety rule for every step: never arm the device, run a sequence, or command motor
motion. Flashing and passive acquisition are allowed. STOP and ask if anything would
energize the motor.

## Scope

- Python/Qt app: `desktop/furuta_lab/` (started by `Start-Furuta-Lab.cmd`; `.venv` already
  installed; Python 3.12, PySide6 6.12, pyqtgraph 0.14). Main UI file `app.py` (~500 lines,
  single `LabApp` class); live refresh is `LabApp.refresh()` (200 ms QTimer).
- MATLAB workbench: `matlab/+furuta/WorkbenchApp.m` (older interface, R2023b).
- Firmware 1.1: `firmware/` (PlatformIO, espressif32 6.12.0, Arduino 2.0.17, binary COBS
  protocol, 460800 baud, COM11 via the COM/UART USB port).
- Tests: `.venv/Scripts/python.exe desktop/test_lab.py`; native core test
  `g++ -std=c++17 -Wall -Wextra -pedantic firmware/test/native_core.cpp -o native_core.exe`;
  MATLAB `run_tests()`.

## Verified facts (repo + capture `data/captures/20261107_025940_761100`)

- Device config already holds `currentZeroMv = 2505.95`, `sensitivityMvA = 185`,
  `countsPerRev = 1496` (34:1 × 11 × 4, NOT verified), `pendDownRaw = 2985`,
  `armSign = pendSign = 1`. The UI never applies them; it shows mV, counts, raw 0–4095.
- Firmware already converts for its trip: `I = (mv - currentZeroMv) / sensitivityMvA`
  (`firmware/include/engine.h:155`), using RAW mv, not the filtered value.
- Stream fields: `current_mv`, `filtered_mv` (MCU filter, bypass by default), `count`,
  `pend_raw`, `mag_status`, `requested`, `applied`, etc. (`desktop/furuta_lab/wire.py`).
- Current noise, motor unpowered, ADC 500 Hz, 2500 samples: σ = 61.8 mV = 0.334 A per
  sample; p-p 418 mV ≈ 2.26 A. Spectrum flat ≈ 3.9 mV/√Hz over 0–250 Hz (white, aliased
  from the sensor's ~80 kHz bandwidth). Averaging N samples: N=8 → 0.120 A,
  N=32 → 0.061 A, N=128 → 0.034 A (≈ 1/√N).
- Status in that capture: `adcOverruns 29`, `controlOverruns 1`.
- An earlier hardware log (`test-results/hardware-after.log`) shows 8 parser errors,
  last error "Invalid version/length".
- Course conventions: pendulum 0° upright, 180° hanging; state x = [θ0, θ1, ω0, ω1].

## Decisions from Nedal (2026-11-07)

1. Free hand on code: improve the Python app and/or the MATLAB workbench, whichever
   gives the better result.
2. All phases are in scope, including firmware. Hardware is connected on COM11.
   Claude Code builds AND uploads the firmware itself, then verifies it (see below).
3. Filtering is SOFTWARE ONLY. No FILTER-pin capacitor, no extra RC at GPIO4.
   Treat the current noise above as given.
4. Views: the user must be able to see ALL signals, with a toggle between
   - Separate: one plot per selected signal, stacked, x-linked time axis;
   - Combined: selected signals overlaid on one plot (per-unit axes or normalized).
   Signal checklist covers at least: arm angle, pendulum angle, arm speed, pendulum
   speed, current (A, raw + smoothed), MCU-filtered current, requested and applied duty,
   magnet status/AGC. A numeric readout for every visible signal. Same views for live
   data and saved recordings.

## Plan

Phase 1 — amps and physical units (host)
- Current in A from the device config (never hard-coded); smoothed value with ± σ,
  e.g. "0.04 ± 0.06 A". Prefer `filtered_mv` when an MCU filter is active.
- Arm angle in degrees: (count − center)·360/countsPerRev·armSign, labelled
  "scale unverified" until the scale-verified flag is set.
- Pendulum angle: wrap ((raw − pendDownRaw)·360/4096·pendSign + 180) to (−180, 180],
  0 = upright.
- Raw units remain available behind a "Raw units" toggle and in device details.
- After "Zero current", report the measured zero (mV) and its σ in mV and A.
- Wherever a software filter is offered, show its cutoff AND its delay
  (2nd-order Butterworth low-frequency group delay ≈ √2/(2π·fc), e.g. ≈ 11 ms at 20 Hz).

Phase 2 — dashboard
- Signal checklist + Separate/Combined toggle (decision 4).
- Colored state pill (DISCONNECTED / DISARMED / ARMED / RUNNING / FAULT).
- Reference checklist chips: arm centered, pendulum down set, current zero set,
  scale verified.
- Proper empty state when disconnected (no blank 0–1 axes).

Phase 3 — layout and flow
- Make the "12 V connected — allow motion" permission visually distinct (safety control).
- Remove duplicates ("Mark center" and "Open a saved run" each appear on two tabs).
- Replace long paragraphs with short numbered steps; per-task progress line with Cancel
  next to it.
- Show noise statistics in A as well as mV in the noise/filter tools.

Phase 4 — firmware (software noise reduction)
- Oversample the ADC: several reads per ADC tick, averaged (measure the per-read time
  and choose N so the ADC task does not overrun).
- Allow ADC at 2 kHz without overruns; run the MCU current filter on every ADC sample.
- Base the current trip on the filtered value instead of raw mV (with present noise,
  a raw-sample trip below ~1.5 A would false-trip).
- Bump the firmware version; keep the protocol backward compatible (Python and MATLAB
  clients both still connect). Update `protocol/SPEC.md` and golden vectors if needed.

## Upload and verification ("working well") — all passive, motor never commanded

Before uploading: close the Furuta Lab app, MATLAB serial sessions and any serial monitor.

1. `pio run -d firmware` builds clean; `pio run -d firmware -t upload --upload-port COM11`
   succeeds; device boots DISARMED with zero duty limit.
2. HELLO/STATUS report the new firmware version; config readback matches what was applied.
3. `desktop/test_lab.py`, the native_core g++ test and MATLAB `run_tests()` pass.
4. `desktop/hardware_smoke.py --port COM11 --seconds 65` passes: measured stream rate close
   to configured, dropped = 0, adcOverruns and controlOverruns ≈ 0, no FAULT.
5. Find and fix the cause of the "Invalid version/length" parser errors.
6. Still captures at 500 and 2000 Hz: report current σ in mV and A before vs after the
   oversampling change, with the numbers and how they were computed.
7. Launch the app on the hardware and confirm the new views (Separate/Combined toggle,
   all signals, readouts, amps) work live.

Report results as a table. Label anything not measured as unverified.

## Note

The Control II brief requires a real-time MATLAB dashboard with a synced digital twin.
The Python app is a characterization tool and does not replace that deliverable.
