# RIP digital twin (Unreal Engine 5.3)

A standalone Unreal app that draws the rig and poses it from UDP packets. Unreal only renders: the
motion always comes from the real rig's encoders or from our model (`matlab/model`, the report's identified model).

- **Solid rig** = real rig (packet source 0). **Translucent cyan ghost** = model (source 1), shown only
  while both are streaming, so model error is visible directly. With no rig data, the solid rig follows the model.
- HUD: latest angles and packet rate per source. Camera: WASD + mouse (free fly). Esc/Alt+F4 quits.

## Run

1. `powershell -File twin\run_twin.ps1` (builds the C++ module if needed, then opens the twin window).
2. In MATLAB: `twin/matlab/twin_demo.m` streams an open-loop swing of the identified model (`furuta_rhs`).
   Or stream anything: `twin_play(t, q_arm, q_pend)`, `twin_play(..., 'real')`, single poses with `twin_send`.

Opening `RIPTwin/RIPTwin.uproject` in the Unreal editor and pressing Play works too.

## Packet (UDP port 7400, 16 bytes, little-endian)

| bytes | content |
|---|---|
| 0–3 | ASCII `RIP1` |
| 4 | source: 0 = real rig, 1 = model |
| 5–7 | padding (0) |
| 8–11 | float32 `q_arm` [rad], right-handed about +Z (Simscape convention) |
| 12–15 | float32 `q_pend` [rad], 0 hanging, π upright |

Conventions vs the harness / report: `q_arm` = theta0, `q_pend` = theta1 - pi (theta1 = 0 upright).
The ESP32 (or the HIL bridge) can send exactly this packet.
Point it at the PC's IP instead of 127.0.0.1.

## How it is built

- `matlab/export_twin_meshes.m` — writes `RIPTwin/Content/RIP/{housing,arm,pendulum}.ripmesh` and
  `twin_geometry.json` from `matlab/harness/plant3d` (meshes) and `twin_frames.mat` (CAD frames). Re-run after any CAD change.
  Meshes are in the zero pose, Unreal frame (y mirrored), **1 mm = 1 Unreal unit** (a 10x model).
- `RIPTwin/Source/RIPTwin/` — `RipTwinActor` (procedural meshes, UDP receive, posing),
  `RipTwinGameMode` (spawns it, camera), `RipTwinHUD` (overlay), `RipLabActor` (the lab corner: desk,
  walls, whiteboard - scaled engine cubes, edit sizes/colours there). Map: the engine's `Template_Default`
  (its sky and sun are kept; its checker floor is removed at start).
- `RIPTwin/Scripts/make_materials.py` — creates `M_Twin` / `M_Ghost` (headless:
  `UnrealEditor-Cmd.exe RIPTwin.uproject -run=pythonscript -script=<abs path>`). Only needed if they are deleted.
- Scripted check: `-ripafterdata -ripshot=2 -ripquit=4` saves `Saved/Screenshots/twin.png` 2 s after the first packet.

Build notes: the Target files pin VS 2022 MSVC 14.40 (VS 18 is also installed and UE 5.3 can't use it).
The C++ compile needs ~3 GB of free commit memory per job — close SolidWorks if it fails with C1060.
