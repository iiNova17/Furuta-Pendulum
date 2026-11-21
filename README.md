# Furuta Pendulum: app and firmware (branch `onboard-control`)

This branch holds only what runs on and next to the rig: the **ESP32-S3 firmware** and the **Furuta Lab** app.
The complete project (model, Simulink, twin, CAD, data, report) is on `main`; the R2023a Simulink model used for
controller development is on `r2023-model`.

## Layout

| Folder | Contents |
|---|---|
| `firmware/` | ESP32-S3 firmware 1.13 (PlatformIO): sensors, safety, telemetry and the onboard controller (`include/onboard.h`: model observer, LQR or PID, energy swing-up, supervisor). `include/onboard_design.h` is generated on `main` by `matlab/simulink_r2026a/controllers/hybrid/export_onboard.m` (never hand-edit); `test/native_onboard.cpp` replays Simulink runs and requires the same duty |
| `desktop/` | Furuta Lab (Python/Qt): live signals and recordings, noise and filters, motor tests, parameter estimation, Onboard control tab, Digital twin tab; tests `test_lab.py`, `test_estimation.py` |
| `protocol/`, `config/` | Serial protocol specification and golden vectors; device profile and schema |
| `matlab/furuta_backend.m`, `matlab/+furuta/` | The MATLAB backend the app calls for its speed-model fit |
| `docs/` | In-app guide (`FOCUSED_GUIDE.md`), hardware notes, onboard control, digital twin |
| `data/` | Created by the app at run time (recordings, logs); not tracked here. Commit recordings to `main` |

## Getting started

1. App: install Python 3.12, run `Setup-Furuta-Lab.ps1` once, then `Start-Furuta-Lab.cmd`.
2. Firmware: `cd firmware; pio run -t upload --upload-port COM11`. Flashing reboots the board: set the arm
   center, the pendulum-down reference and the current zero again.
3. Replay test after any change to the controller:
   `g++ -std=c++17 -O2 -static -o native_onboard firmware/test/native_onboard.cpp && ./native_onboard firmware/test`
   (and once more with `-DONBOARD_OBSERVER_DOUBLE`).
4. Hardware bring-up: `docs/ONBOARD_CONTROL.md`, using the app's Onboard control tab.
5. Digital twin: tab Digital twin (`docs/TWIN.md`). For the lowest delay set the FTDI latency timer of the
   serial port to 1 ms (Device Manager, USB Serial Port, Port Settings, Advanced).

## Adding a controller (PID, pole placement)

1. Design it in Simulink on `r2023-model` (`controllers/hybrid/stab_pid.slx`, `stab_pole_placement.slx`,
   `ctrl_hybrid_design.m`) and bring it into `main`.
2. On `main`, extend `export_onboard.m` to write its numbers into the generated header, add its law to
   `firmware/include/onboard.h` with a replay test like `native_onboard.cpp`, let `START_ONBOARD` select it
   and bump the firmware version; then copy `firmware/` here.
3. In the app add one entry to `CONTROLLERS` in `desktop/furuta_lab/onboard_page.py`: its `start` arguments are
   sent with `START_ONBOARD` as they are, and its knobs appear automatically from `ONBOARD_GET`.

Motor safety: never command motor motion unless the person at the rig has confirmed that the center pin is
removed, 12 V is on and the area is clear.
