# Digital twin in Furuta Lab

Furuta Lab has a **Digital twin** tab: the rig in 3D, posed from the serial stream, with the identified model as a
translucent cyan ghost next to it. It can also open in its own window (View → “Open in its own window”) or replace
the plots on the Onboard control tab (“Show the twin instead of the plots on the Onboard control tab”).

| File | Contents |
|---|---|
| `desktop/furuta_lab/twin_core.py` | meshes, kinematics, model ghost, RIP1 UDP packet (no Qt, unit-tested) |
| `desktop/furuta_lab/twin_view.py` | Qt Quick 3D view (`TwinView`, a `QQuickWidget`) and the mesh geometry type |
| `desktop/furuta_lab/twin_page.py` | the tab: source, model ghost, UDP, latency readout |
| `desktop/furuta_lab/twin_assets/` | `housing/arm/pendulum.ripmesh` and `twin_geometry.json` (copied from `main:twin/RIPTwin/Content/RIP`, generated from the CAD by `twin/matlab/export_twin_meshes.m`), `model_params.json` (the identified model, `claude_loop` session), `twin.qml` (scene) |

## Why Qt Quick 3D and not Unreal 4.27 or three.js

The twin on `main` (`twin/`) is an Unreal Engine 5 project fed by UDP. Three options were compared for the app:

| Option | Path from serial sample to screen | Verdict |
|---|---|---|
| Port the UE 5 project to UE 4.27 | serial thread → UDP → second process → engine frame queue | UE 5 assets do not open in 4.27 (re-import and C++ rebuild); still a separate program next to the app |
| three.js in a `QWebEngineView` | serial thread → Python → JS bridge (Chromium IPC) → WebGL → Chromium compositor | an extra process hop and compositor frame; a vendored JS download |
| **Qt Quick 3D in the app (chosen)** | serial thread buffer → read on every display frame → RHI (Direct3D 11) | same process, no network or IPC hop; already part of PySide6 (`PySide6-Addons`) |

The UE twin still works: “Forward to the Unreal twin” sends the same RIP1 packets it listens for.

## What is drawn

- **Rig (solid):** θ0 and θ1 of the newest stream record, computed exactly as the firmware does
  (`onboard_page.angles`). Twin convention (`twin/README.md`): q_arm = θ0, q_pend = θ1 − π (0 hanging).
  If the arm is not centered, the arm is drawn at 0 (the HUD says so).
  On the rig, positive θ0 and θ1 turn opposite to the CAD frame's right-hand rule, so the view draws both
  angles with the opposite sign (`twin_core.DISPLAY_SIGN`, checked at the rig 2026-11-20). Flipping both is a
  mirror symmetry of the dynamics, so the model is unchanged; UDP packets keep the `twin/README.md` convention.
  Colors as on the rig: black arm, red pendulum.
- **Model ghost (cyan)**, one of:
  1. *Identified model*: `twin_core.ModelGhost` integrates the identified equations (RK4, ≤ 1 ms steps,
     breakaway friction included) driven by the duty the firmware applied, and is put back on the measured
     state every N s (default 2 s, 0 = never). The gap between solid and cyan is the model error over that
     horizon.
  2. *UDP packets*: poses received on port 7400 (e.g. MATLAB `twin_send` / `twin_play` on `main`).
     Source 1 drives the ghost; source 0 drives the rig when no live/recorded data is shown.
  3. *Off*.
- **Playback:** “Playback: the open recording” replays the run that is open in the app (Open recording… or the
  run just recorded) at 0.25–2×, with a seek slider; the model ghost runs along with the recorded duty.
- HUD: arm and pendulum angles (pendulum 0° = upright, as on the Onboard control tab), controller state,
  pose age and display frame rate.

The view runs its frame timer (8 ms, precise) only while it is visible.

## Latency (measured on the rig, passive, motor unpowered)

Firmware 1.11 sends every stream record in its own packet as soon as it is taken (1.10 waited until 4 records
were queued or 20 ms had passed, so a record could sit on the MCU for up to 16–20 ms). Use “Live stream at 250 Hz”
on the twin tab for a record every 4 ms.

| Quantity | Value |
|---|---|
| Stream, firmware 1.11 at 250 Hz | 1 record per packet, 1500 packets in 6 s |
| Display rate of the twin tab | 86–87 frames/s |
| Pose age on screen, above the fastest packet of the last 2 s | median 2.0–2.8 ms, 95 % 14 ms, max 21 ms |
| Command round trip (request → reply) | 33–48 ms, median 39 ms |

The remaining spread is the USB bridge, an FTDI FT232R: its Windows driver holds received bytes until 64 bytes
arrive or its **latency timer** runs out (16 ms by default). Setting it to 1 ms removes most of it:
Device Manager → Ports (COM & LPT) → USB Serial Port (COM11) → Properties → Port Settings → Advanced →
Latency Timer (msec) = 1 → OK, then unplug and replug the USB cable. The pose age above was measured with
the default 16 ms; a re-measurement after the change is not done yet.

## Requirements

`PySide6-Addons==6.12.0` (Qt Quick 3D), listed in `desktop/requirements.txt`; `Setup-Furuta-Lab.ps1` installs it.
Without it the tab shows why the view is unavailable and the rest of the app works as before.
Tests: `desktop/test_lab.py` → `TwinTests` (mesh orientation, kinematics, model ghost at rest and re-sync,
UDP round trip, tab switching and playback with the 3D view).
