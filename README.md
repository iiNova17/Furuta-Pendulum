# Furuta Pendulum: onboard control (branch `onboard-control`)

This branch holds only what runs on and next to the rig: the **ESP32 firmware** and the **Furuta Lab app**.
It is where the onboard controllers are brought up on hardware: swing-up + LQR now (firmware 1.10), PID and
pole placement next. The model, the Simulink harness (where the controllers are designed), the report,
the CAD and the recorded data live on `sim-harness` / `main`.

## Layout

| Folder | Contents |
|---|---|
| `firmware/` | ESP32-S3 firmware (PlatformIO). `include/onboard.h` is the onboard controller; `include/onboard_design.h` is **generated** on `sim-harness` by `matlab/harness/controllers/hybrid/export_onboard.m` (copy it here, never hand-edit); `test/native_onboard.cpp` replays harness runs and requires the same duty as Simulink |
| `desktop/` | Furuta Lab (Python/Qt) app. Tab **Onboard control** (`furuta_lab/onboard_page.py`) starts the onboard controller, shows its state, edits its knobs and records the run |
| `docs/ONBOARD_CONTROL.md` | How onboard control works and the hardware bring-up procedure |
| `docs/` | App guides, hardware notes, earlier implementation plans |
| `protocol/`, `config/` | Serial protocol spec and golden vectors; device profile and schema |
| `matlab/furuta_backend.m`, `matlab/+furuta/` | Only the MATLAB backend the app calls for its speed-model fit |
| `data/` | Created by the app at run time (recordings, logs). Not part of this branch: new recordings show as untracked files; commit them on `sim-harness` / `main` with the rest of the data |

## Getting started

1. App: install Python 3.12, run `Setup-Furuta-Lab.ps1` once, then `Start-Furuta-Lab.cmd`.
2. Firmware: `cd firmware && pio run -t upload --upload-port COM11` (the rig is on COM11 here).
3. Onboard replay test after any design change:
   `g++ -std=c++17 -O2 -static -o native_onboard firmware/test/native_onboard.cpp && ./native_onboard firmware/test`
4. Bring-up on hardware: follow `docs/ONBOARD_CONTROL.md`, using the app's Onboard control tab.

## Adding a controller (PID, pole placement)

1. Design it in the harness on `sim-harness` (`controllers/hybrid/`), export its numbers with an
   `export_onboard`-style script into a generated header, and add its law to `firmware/include/onboard.h`
   with a replay test like `native_onboard.cpp`.
2. Let `START_ONBOARD` select it (e.g. a `controller` field) and bump the firmware version.
3. In the app add one entry to `CONTROLLERS` in `desktop/furuta_lab/onboard_page.py`: its `start`
   arguments are sent with `START_ONBOARD` as they are, and its knobs appear automatically from `ONBOARD_GET`
   (add their units and meanings to `KNOBS`).

Motor safety: never command motor motion unless the person at the rig has confirmed that the center rod
is removed, 12 V is on and the area is clear.

---

# Furuta Lab

The primary app is now a focused Python/Qt instrument interface with a
separate UART worker, NumPy/SciPy measurement tools, and an on-demand MATLAB
model backend. It uses raster plots rather than the MATLAB web GUI or GPU
OpenGL rendering.

## Start the focused app

Double-click **Start-Furuta-Lab.cmd**, or run **Start-Furuta-Lab.ps1** from
this folder. This PC's local `.venv` is
installed. For another PC, use Python 3.12 and run Setup-Furuta-Lab.ps1 once.
Choose COM11, Connect USB, and leave the 12 V declaration unchecked for USB-only
sensor work. Open [the focused guide](docs/FOCUSED_GUIDE.md) for the five workflows.

- Live sensor readings, reference setting and recording.
- Still-state noise PSDs and bandwidth/delay-constrained filter comparisons.
- Unpowered travel marking, bounded actuation and breakaway dead-zone pulses.
- Separate motor-only and assembled-system datasets and parameter estimators.
- Background MATLAB PT1/tfest validation with saved model artifacts.

Reduction 34:1 is provisional: with 11 cycles and x4 decoding it implies 1496
counts/output revolution. Confirm against a measured angle before physical
assembled-system estimation. Motor-run buttons remain disabled in USB-only mode.

The current compatible firmware is 1.1. It advertises faster ADC download chunks;
old clients remain compatible. Python automatically falls back for firmware 1.0.
Build/upload still uses the PlatformIO project in `firmware/`.

Tests: `.venv/Scripts/python.exe desktop/test_lab.py`. The explicit real-USB
smoke test is `desktop/hardware_smoke.py --port COM11 --seconds 65 --backend`;
it never arms or commands motion. Its MATLAB backend input is synthetic and
is labelled separately from the real USB recordings.

## Retained MATLAB workbench

ESP32-S3 firmware and a MATLAB R2023b desktop app for bounded motor experiments,
sensor calibration, raw recording/replay, noise spectra, filter comparison,
and system identification. Start in **Simulator** mode to explore the app.

### Launch the older MATLAB interface

From MATLAB, change into this repository and run:

    addpath('matlab');
    app = FurutaWorkbench();

The app starts disconnected. It has eight tabs: Live & Manual, Experiments,
Calibration, Time Analysis, Spectrum & Noise, Filters, Identification, and
Runs & Settings. The source/channel/interval selectors above the tabs apply
to offline analysis.

Start with the [practical operating guide](docs/QUICK_START.md) for connection,
zeros, animation calibration, noise analysis, filtering and identification.
The same guide opens from **Runs & Settings -> Guide / help** in the app.

### Try a complete simulated experiment

1. Select **Simulator**, then **Connect**. Wait for DISARMED.
2. On Live & Manual, click **Simulator setup**. This applies a labelled
   synthetic profile and centers its encoder; it does not move.
3. On Experiments, define a waveform, click **Replace sequence**, then
   **Preview**. Use **Record + run** for an uploaded, armed, finite experiment
   with recording. Alternatively Upload and validate, Arm, then Run sequence.
4. After completion, select a recorded/live channel and use the analysis tabs.
5. In Runs & Settings, **Capture passive burst**, wait until capture completes,
   then **Download completed burst**. Select ADC burst to inspect individual
   current samples; Mechanical burst exposes its simultaneous 500 Hz records.

The interactive simulator advances a bounded synthetic clock, so it can run
slower than wall time on a busy machine. It is not a real-time hardware
benchmark. Saved simulated runs are explicitly marked synthetic.

## Build the firmware

Open the **firmware** folder in PlatformIO IDE, or run:

    pio run -d firmware

Pinned environment: espressif32 6.12.0, Arduino 2.0.17, ESP32-S3, 16 MB flash
and octal PSRAM. ArduinoJson 6.21.5 is the only added library.
No build or MATLAB launch automatically uploads or connects.

This new firmware uses a binary protocol at **460800 baud through COM USB**.
An older 115200-baud ASCII bench program is incompatible. For a coordinated
flash, remove motor power, upload from PlatformIO, verify disabled startup,
then restore motor power for operator-started experiments.

Hardware starts with duty limit zero and no incremental arm reference.
In Runs & Settings, deliberately commission maxDuty (fraction), slew,
maxDuration, raw-count travel and any validated current/velocity limits.
Then set the arm center in Calibration. An unknown gearbox ratio does not
prevent count-limited duty experiments; countsPerRev is required for radians.
Every new MCU boot requires re-centering.

The current circuit has a direct 5 V current-sensor signal to an ESP32 ADC
and a motor/driver current-rating mismatch documented in
[hardware notes](docs/HARDWARE.md). Software does not remove those limitations.
The app preserves the pendulum encoder's weak-field warning.

## What is implemented

- Device-timed hold/step, pulse/doublet, ramp, triangle, sine, square, chirp,
  multisine, seeded PRBS, arbitrary sample tables, repeated sequences and rest.
- Explicit arming, highest-priority stop, session challenge/retry handling,
  MCU heartbeat timeout, finite duration, travel/current/ADC/velocity limits.
- PCNT quadrature extension, AS5600 diagnostics, same-conversion ADC millivolts,
  configurable sampling, raw ADC/mechanical burst capture and chunk download.
- Arm/pendulum/current zeros, encoder-scale and known-current gain fitting,
  RAM profile readback and explicit NVS save.
- Time traces, statistics, response metrics, FFT, Welch PSD/ASD, spectrogram,
  band RMS, autocorrelation, FRF and coherence.
- Bypass, EMA, moving-average, 2nd-order Butterworth and notch filters:
  compare traces/PSD/gain/phase/delay, optionally deploy a supported causal
  filter at the ADC rate. Zero-phase filtering stays offline.
- ARX, local motor speed PT1, toolbox transfer-function/state-space estimators,
  pendulum free-decay fitting and bounded nonlinear physical-model fitting.
- Recording with raw measurements/configuration/events, replay, CSV/MAT export,
  model/filter export and a parameter registry.

Heavy offline analysis requires the device to be disarmed. Waveforms run on
the MCU; MATLAB drawing does not schedule individual actuator updates.
Acquisition rates are engineering targets until measured on the real hardware.
At 2 ksample/s, the current spectrum stops at 1 kHz; it cannot inspect a
20 kHz PWM carrier.

## Tests

From a terminal with g++:

    g++ -std=c++17 -Wall -Wextra -pedantic firmware/test/native_core.cpp -o native_core.exe
    ./native_core.exe

The checked-in golden vectors were generated by the C++ serializer. MATLAB
tests compare exact frames, sample records and waveform values against them.

From MATLAB:

    addpath('matlab');
    results = run_tests();
    result = visual_smoke();  % Simulator only; renders screenshots under test-results

See [validation](docs/VALIDATION.md), [user guide](docs/USER_GUIDE.md),
[experiments](docs/EXPERIMENTS.md), and [wire protocol](protocol/SPEC.md).
The original [implementation plan](IMPLEMENTATION_PLAN.md) remains available.

Raw recordings live under data/ (git-ignored). Live recording uses
length-prefixed binary sample batches, avoiding HDF5 initialization in a
heartbeat callback. Open/replay reconstructs typed tables; selected data can
be exported to MAT v7.3 or CSV with metadata. Older chunked-MAT recordings are
also readable.
