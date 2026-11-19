# Running the controller on the rig: two modes

The swing-up + LQR controller is designed once, in the Simulink harness (`matlab/harness`,
report sections 8–10). It can run on the rig in two ways. In both, the firmware keeps every
safety function: duty clip and slew, arm travel trip, overcurrent trip, heartbeat.

| | 1. Hardware in the loop (HIL) | 2. Onboard (product mode) |
|---|---|---|
| Controller runs in | Simulink on the laptop (`ctrl_hybrid.slx`, Stateflow) | ESP32 firmware (`firmware/include/onboard.h`) |
| In the loop | USB, both directions (delay not measured yet) | nothing: 4 ms, the timing it was designed for |
| Laptop does | runs the controller | commands, knobs, heartbeat, telemetry, digital twin |
| Change a law | edit the Simulink file | edit `ctrl_hybrid_design.m`, re-export, rebuild, flash |
| Status | **not built**: needs the harness Hardware variant and a compact duty message in the firmware | **built** (firmware 1.6), equivalent to the harness, **not yet run on the rig** |

HIL is what the course brief asks for (Simulink/Stateflow against real hardware). Onboard is the
finished device: the laptop is only the GUI and the twin.

## One design, two copies

- The knobs live only in `matlab/harness/controllers/hybrid/ctrl_hybrid_design.m`.
- `export_onboard.m` writes `firmware/include/onboard_design.h` (generated, do not edit) and two
  replay files, `firmware/test/onboard_replay_{swingup,push}.csv`: the angles the Simulink controller
  received in a harness run and the duty it commanded.
- `firmware/test/native_onboard.cpp` feeds those angles to the C++ controller and requires the same duty
  and supervisor state at every 4 ms period (last result: max difference 7e-15 over 3751 + 1501 periods,
  0 state mismatches). It also checks that onboard runs keep the travel and heartbeat trips.

After any design change:

```matlab
cd matlab/harness; harness_setup; export_onboard        % header + replay files
```
```bash
g++ -std=c++17 -O2 -static -o native_onboard firmware/test/native_onboard.cpp && ./native_onboard firmware/test
cd firmware && pio run                                  # build; flash with: pio run -t upload
```
`-static` avoids picking up an older `libstdc++` from Git's folder on PATH (it crashes at start).

## What the firmware does in an onboard run

- Every 4th 1 ms tick: arm angle from the encoder (`armSign`, `countsPerRev`, marked centre), pendulum
  angle from the AS5600 (`pendSign`, `pendDownRaw`), then estimator → LQR / energy swing-up → supervisor
  (SwingUp ↔ Stabilize, SafeStop at |θ0| > 100°). The duty request goes through `Controller::tick`, exactly
  like an experiment.
- `START_ONBOARD` is refused unless: ARMED (centred arm, commissioned limits, current zero if the trip is
  on), encoder scale set, pendulum-down reference set, a fresh pendulum reading, and the profile signs
  equal to the identified model's (`armSign` +1, `pendSign` −1).
- The run stops (motor off) on: any firmware trip, pendulum reading older than 20 ms (`PEND_INVALID`),
  supervisor SafeStop (`ONBOARD_SAFE_STOP`), losing the pendulum in balance-only mode
  (`ONBOARD_PENDULUM_LOST`), `STOP`, heartbeat loss, or the profile's `maxDuration`.
- Telemetry: byte `ctrl` of every core record = 0 not running, 1 SafeStop, 2 SwingUp, 3 Stabilize
  (`wire.CTRL_NAMES`).

Commands (JSON, protocol v1): `ONBOARD_GET`, `ONBOARD_CONFIG {params: {...}}` (disarmed only; keys
`K0..K3, r, velFc, fricGain, fricBand, catchTh1, catchW1, exitTh1, safeTh0, E0, kE, aPush, wc, zc,
aKick, kickE, wSign, wFric, iMax, uMax, swingUp`), `ONBOARD_DEFAULTS`, `START_ONBOARD {swingUp: bool}`.
Knob changes are kept in RAM only; the defaults are the design file's values.

## Bring-up procedure (onboard mode)

Powered steps need the person at the rig to confirm, for that batch, that **the center rod is removed,
12 V is on and the area is clear**. Keep a hand near STOP. Never skip a step.

1. Flash firmware 1.6. Flashing reboots the MCU: redo arm centre, pendulum-down reference, current zero
   and encoder scale in the lab app.
   **Expected:** the app connects; `identity['firmware'] == 'FurutaWorkbench/1.6'`.
2. Check the profile (Setup page): `countsPerRev` 1496, `armSign` +1, `pendSign` −1, `maxDuty` 0.5,
   `slewPerSecond` 100 (the default 0.5 is far too slow for feedback), `travelCounts` ≈ 498 (120°),
   `currentTripA` 1.5, `heartbeatMs` 300, `maxDuration` = the run length you want (e.g. 20 s).
3. Passive sign check (motor unpowered is fine): stream data and move the parts by hand.
   **Expected:** pendulum held upright reads about 0°, hanging ±180°; turning the arm in its positive
   direction gives a positive arm angle. **If not:** fix the profile before any powered run.
4. Stage 1, LQR only. Confirm rod out / 12 V / area clear. Hold the pendulum lightly near upright, then:
   ```python
   from furuta_lab.device import Device
   d = Device(); d.connect('COM5')        # close the Furuta Lab app first (one port owner)
   d.inhibit = False                       # same as enabling powered motion in the app
   d.run_onboard(swing_up=False)           # release the pendulum right after this returns
   ```
   **Expected:** the pendulum stays up; the arm swings slowly by about ±20° (friction limit cycle with
   `fricGain` 0, report 9.9); the run ends at `maxDuration`. **If not:** `d.stop()`; read the last event
   (`d.last_event`) and the recorded telemetry.
5. Stage 2, full swing-up. Pendulum hanging at rest, arm centred. Confirm rod out / 12 V / area clear.
   ```python
   d.run_onboard(swing_up=True)
   ```
   **Expected (simulation):** caught after about 1.5 s, arm within about 50°, current below 1 A while
   swinging and one short peak near 1.5 A at the catch, then balancing. **If not:** `d.stop()`.
6. Stop with `d.stop()` (or close the app: the heartbeat stops and the firmware trips within 300 ms).

| Symptom | Likely cause | Check | Fix |
|---|---|---|---|
| `START_ONBOARD` refused: signs | profile `armSign`/`pendSign` differ from the model | Setup page | set +1 / −1 (step 3 must then pass) |
| `PEND_INVALID` | AS5600 read failing or magnet status bad | `pendStatus`, `pendValid` in STATUS | wiring / magnet distance |
| Stage 1 falls immediately the other way | a sign is wrong despite step 3 | arm vs duty direction (estimation check "Kb not positive") | stop; recheck signs |
| Swing-up stalls below the top | push smaller than designed (friction higher than identified) | arm amplitude in telemetry | report section 10.8: push tolerance is about ±5 %; raise `aPush` slightly via `ONBOARD_CONFIG` |
| Arm runs past 100° after the catch | arm too fast at hand-over | `ctrl` and arm speed at the catch | raise `wc` (6.9 also passed all worst cases) |
| `CONTROL_OVERRUN` | the 1 ms loop is too slow | STATUS `controlOverruns` | report it: the onboard step needs optimising |

## HIL mode: what is missing

- Harness: the `Hardware` variant of the Rig (serial link from Simulink to the ESP32 at 4 ms).
- Firmware: a compact binary duty message (the JSON command path with its status reply is far too large
  for 250 Hz at 460800 baud) with a staleness trip of a few controller periods, as a third run source
  next to `Experiment` and `Onboard` in `Controller` (`engine.h`).
- Then measure the USB delay and set `cfg.link.u_delay` / `y_delay` in `harness_config.m`.
