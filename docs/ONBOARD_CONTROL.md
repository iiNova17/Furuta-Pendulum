# Running the controller on the rig: two modes

The swing-up + LQR controller is designed once, in the Simulink harness (`matlab/harness`,
report sections 8–10). It can run on the rig in two ways. In both, the firmware keeps every
safety function: duty clip and slew, arm travel trip, overcurrent trip, heartbeat.

| | 1. Hardware in the loop (HIL) | 2. Onboard (product mode) |
|---|---|---|
| Controller runs in | Simulink on the laptop (`ctrl_hybrid.slx`, Stateflow) | ESP32 firmware (`firmware/include/onboard.h`) |
| In the loop | USB, both directions (delay not measured yet) | nothing: 1, 2 or 4 ms inside the 1 kHz firmware loop (firmware 1.7) |
| Laptop does | runs the controller | commands, knobs, heartbeat, telemetry, digital twin |
| Change a law | edit the Simulink file | edit `ctrl_hybrid_design.m`, re-export, rebuild, flash |
| Status | **not built**: needs the harness Hardware variant and a compact duty message in the firmware | **built** (firmware 1.6, faster in 1.7), equivalent to the harness, **not yet run on the rig** |

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

## Firmware 1.7: faster loop and better watching

| | 1.6 | 1.7 |
|---|---|---|
| Controller period | 4 ms | **1, 2 or 4 ms** (`ONBOARD_CONFIG {params: {Ts: 0.001}}`; the app's period selector). Each period uses the design's LQR gain for it: same Q and R, `firmware/tools/onboard_rates.py` → `include/onboard_rates.h`; the 4 ms row equals `design::K` to 6e-13 |
| Pendulum (AS5600) read | every 2 ms on the FreeRTOS tick | **every 1 ms on its own timer, 950 µs before the control tick** |
| Pendulum reading age at the controller (measured, median / 99 %) | 0.94 / 1.68 ms | **0.45 / 1.45 ms** |
| AS5600 filter | chip default, not set | written at boot and reported (`as5600Conf`): still the default 16x slow filter |
| Stream during onboard runs | as connected (100 Hz in the app) | **250 Hz** (the app raises it for the run) |
| High-rate record of a run | — | MCU capture with the run (500 Hz records, current at 2 kHz), downloaded after the run into `<run>/burst/` |
| Diagnostics | — | `ONBOARD_BENCH` (µs per controller step), `pendReads`, `pendFails`, `pendReadMaxUs` in STATUS |

Measured on the rig (12 V off, nothing armed, 2026-11-19): one controller step takes **143 µs mean, 352 µs
worst** (soft-float double; 14 % of a 1 ms period); 1016–1022 pendulum reads/s with 0 failures (longest read
about 1 ms); 250 records/s streamed with 0 dropped and 0 link errors; no control overruns.

Why 1 ms and not more, and why the AS5600 filter stays at 16x: a closed-loop simulation on the identified model
(firmware port of `onboard.h`, 1 ms firmware loop with clip / slew / PWM, quantized sensors, the harness's 7 ms
pendulum delay and datasheet sensor noise) gave the LQR a pendulum-delay margin of 22.2 ms at 4 ms, 22.5 ms at
2 ms and **25.0 ms at 1 ms with the pendulum read at 1 kHz**; swing-up caught in 1.45–1.48 s in every case. The
faster AS5600 filters (8x/4x/2x) gave 23.1–23.4 ms: their extra noise cancels the latency they save. A 1 ms
controller with the pendulum read every 2 ms failed the swing-up (every other speed estimate saw a stale
reading), which is why the read rate went up with the controller rate. The cost of 1 ms is about twice the
high-frequency duty switching (5.1 vs 2.2 %/ms rms), well inside the slew limit. The 4 ms setting is still the one
validated in Simulink and replayed by `native_onboard.cpp`.

## Firmware 1.9: friction compensation in balance

**Why.** With 1.8 the swing-up caught the pendulum (run `20261119_205453_867500`, caught after 1.16 s with the arm
at 3 rad/s) and balanced it for 0.7 s. Then the arm stuck at 19.3° for 0.4 s: the LQR's corrections were 1–9 %
duty, below the 13 % breakaway, so the pendulum leaned to −3.5° until the request reached −19 % and the arm broke
free with a lurch. The LQR then asked for ±100–116 % (saturated at the 80 % maxDuty of that run), the current
reached 3.0–3.2 A at each reversal against the back-EMF, the current sensor clipped at the top of the ADC
(3106 mV ≈ +3.2 A) and the firmware stopped on `ADC_INVALID_OR_CLIPPED` (the current trip was off).

**Change.** The LQR output gets the friction compensation in the direction of its request, blending from the
breakaway duty at rest (Rm·tsp/Vs = 13 %, 14 %) to the sliding Coulomb duty (7.4 %, 7.3 %) as the arm moves:
`u = u0 + fricGain·(coul + fricStick·exp(-(w0/wStick)²)·(stat - coul))`, sign smoothed by tanh(u0/fricBand).
**On by default** (`fricGain` 1, `fricStick` 1, set in `ctrl_hybrid_design.m`, which had `fricGain` 0 before); `fricGain` 0 in the knob table
gives the 1.8 behaviour, `fricStick` 0 the Coulomb-only compensation.

**Simulation** (model with breakaway, 7 ms pendulum delay, sensor noise, 4 ms): the share of the balance time with
the arm stuck was 41–49 % without compensation (100 % from a catch-like state at 19°), 74–80 % with the
Coulomb-only compensation (7.4 % cannot beat 13 %), **4 %** with the breakaway compensation; pendulum rms
0.6–1.1° instead of 1.9–2.0°; no saturation. The model has no gearbox play, so it reproduces the sticking but not
the lurch that followed. C++ law = simulated law to 1.5e-13; 226 µs per controller step on the chip.

**For the next run:** current trip on (default 4 A; positive currents clip the sensor at about +3.2 A first),
maxDuty 50 % (what the design assumes), high-rate capture on, and Setup → “Save current profile to device NVS”
so a reflash does not reset the profile.

**Risk.** In the report's worst cases (Table 12, section 9.9) the full compensation tripped every case with a
10 ms pendulum sensor lag, which the uncompensated design survived; the identified lag is 7 ms. If the balance
oscillates, lower `fricGain` (e.g. 0.5) in the knob table; `fricGain` 0 is the 1.8 behaviour.

## Firmware 1.8: breakaway friction and unlimited runs

**Why.** The first powered runs (2026-11-19, `data/runs/20261119_*`) never left the hanging rest on their own:
the swing-up's kick asked for 11.1 % duty and the arm stayed stuck for 9–27 s, because the arm needs 13 % / 14 %
to break free (the breakaway dead zone measured with Motor tests on 2026-11-07). No copy of the model had this:
`furuta_rhs.m`, the Simscape harness plant, `estimation.py` and the controller's motor inverse all had only the
identified sliding Coulomb friction (7.4 % / 7.3 %), smoothed with tanh(w0 / 0.1), so a torque below it still let
the arm creep. Replaying the recorded 11.1 % duty through the model: without breakaway the arm moves 285° in
6 s; with it, 0° (as on the rig).

**Model.** Arm breakaway friction tsp = 13 % · 12 V / Rm = 0.4554 A, tsn = 14 % · 12 V / Rm = 0.4905 A (1.76 × the
sliding friction), stick-slip (Karnopp): within |θ̇0| < 0.05 rad/s the friction holds the arm up to the breakaway
and brings its speed to zero (2 ms time constant); above it the identified Coulomb + viscous friction applies.
`desktop/furuta_lab/estimation.py` (`STICK`, `breakaway_from_deadzone`); parameter sets without tsp / tsn simulate
exactly as before. The validation fits of the identified model are unchanged (within 2 points; the validation
square waves are all above the dead zone).

**Controller.** The swing-up's motor inverse adds the breakaway in the direction of the wanted arm acceleration
when the arm is at rest, blending to the sliding Coulomb friction as it speeds up:
`i = Ja a0 + Beq w0 + coul + stickGain · exp(-(w0/wStick)²) · (stat - coul)`, with `coul` the Coulomb term of 1.7,
`stat = tsp·max(sa,0) + tsn·min(sa,0)`, `sa = tanh(a0/aStick)`. Knobs `tsp`, `tsn`, `stickGain` (1; 0 = the 1.7 law),
`wStick` (0.2 rad/s), `aStick` (5 rad/s²); defaults from `ctrl_hybrid_design.m`, written into
`include/onboard_design.h` by `export_onboard.m` (the measured breakaway comes from `furuta_params.m`). The kick from rest now asks for 24.1 % duty instead of 11.1 %. Closed-loop
simulation on the model with breakaway: the 1.7 law never moves the arm (as on the rig); with the feedforward
the swing-up is caught after 1.44 s (4 ms) / 1.45 s (2 ms), arm within 47°. The Simulink replay test runs with
`stickGain` 0 (the harness runs predate it) and still matches to 6e-15; the C++ law equals the simulated one to 2e-11.
The Simulink swing-up block does not have the feedforward yet (see `docs/HANDOFF_BREAKAWAY.md` on `sim-harness`).

**Run length.** `maxDuration` 0 = no limit for onboard runs (and experiments): the run lasts until STOP / DISABLE,
a trip, a fault, SafeStop or heartbeat loss. App: “No run-length limit”.

**Controller period.** 1 ms never swung up on the rig (noisier speed estimates, jumpier duty, weaker pump), so the
app's default is 4 ms again (the Simulink design); 2 ms also worked.

Measured (12 V off, nothing armed): controller step 205 µs mean, 386 µs worst with the feedforward; no overruns.

## What the firmware does in an onboard run

- Every Ts (1, 2 or 4 ms; every 1st, 2nd or 4th tick of the 1 ms loop): arm angle from the encoder (`armSign`, `countsPerRev`, marked centre), pendulum
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

All steps use the Furuta Lab app (`Start-Furuta-Lab.cmd`), tab **Onboard control**. Its readiness list
shows every item below with ✓ / ✗ / ! (advisory), and the Start button stays disabled until the device
is DISARMED, connected to firmware 1.6 or newer and the 12 V box is ticked. The firmware refuses `START_ONBOARD`
on its own gates as well (see above).

1. Flash firmware 1.9 (`cd firmware && pio run -t upload --upload-port COM11`). Flashing reboots the MCU:
   redo arm centre, pendulum-down reference and current zero on the Live tab (References), and check
   the encoder scale (Setup).
   **Expected:** the app connects; readiness shows “Firmware with onboard control (FurutaWorkbench/1.9)”.
2. Onboard control → **Read from device** (knobs), then **Apply these onboard-run limits to RAM**:
   run length (`maxDuration`, e.g. 20 s), maximum duty 50 %, slew 10000 %/s (= 100 /s; the old default
   is far too slow for feedback) and current trip 4 A (the default; A. Note: the current sensor reaches the top of the ESP32 ADC at about +3.2 A, where ADC_INVALID_OR_CLIPPED stops the motor first; the TB6612 is rated 1.2 A continuous / 3.2 A peak.). Travel stays as commissioned on Motor tests
   (≈ 498 counts, 120°). Optionally Setup → “Save current profile to device NVS”.
   **Expected:** no ✗ or ! left in the readiness list except the 12 V box.
3. Passive sign check (motor unpowered is fine): watch θ0 / θ1 on the Onboard control page and move the
   parts by hand.
   **Expected:** pendulum held upright reads about 0° (the catch-region light turns green within ±20°),
   hanging ±180°; turning the arm in its positive direction gives a positive θ0. **If not:** fix the
   profile (the page offers “Set profile signs to the model’s”) and repeat before any powered run.
4. Stage 1, LQR only. Confirm rod out / 12 V / area clear. Tick the 12 V box, choose **LQR balance
   only**, hold the pendulum lightly near upright (catch-region light green), press **Start onboard
   controller**, then let go.
   **Expected:** state STABILIZE; the pendulum stays up; the arm swings slowly by about ±20° (friction
   limit cycle with `fricGain` 0, report 9.9); the run ends at `maxDuration`. **If not:** STOP / DISABLE.
   The page reports the stop reason (firmware notice or fault) and, when recording, a run summary.
5. Stage 2, full swing-up. Pendulum hanging at rest, arm centred. Confirm rod out / 12 V / area clear.
   Choose **Swing-up + LQR balance** and press Start.
   **Expected (simulation):** SWING-UP, then STABILIZE after about 1.5 s, arm within about 50°, current
   below 1 A while swinging and one short peak near 1.5 A at the catch, then balancing. **If not:**
   STOP / DISABLE.
6. Stop with **STOP / DISABLE** (or untick the 12 V box, or close the app: the heartbeat stops and the
   firmware trips within 300 ms). Recorded runs are in `data/runs/` with `purpose: onboard`, the
   controller, and the knob values in the manifest; the `ctrl` signal (Live tab, group “Onboard”)
   shows the controller state.

Without the app (scripts): `d = Device(); d.connect('COM11'); d.inhibit = False; d.run_onboard(swing_up=False)`
(close the app first: one port owner), knobs with `d.onboard_params({...})`, stop with `d.stop()`.

| Symptom | Likely cause | Check | Fix |
|---|---|---|---|
| `START_ONBOARD` refused: signs | profile `armSign`/`pendSign` differ from the model | Setup page | set +1 / −1 (step 3 must then pass) |
| `PEND_INVALID` | AS5600 read failing or magnet status bad | `pendStatus`, `pendValid` in STATUS | wiring / magnet distance |
| Stage 1 falls immediately the other way | a sign is wrong despite step 3 | arm vs duty direction (estimation check "Kb not positive") | stop; recheck signs |
| Swing-up stalls below the top | push smaller than designed (friction higher than identified) | arm amplitude in telemetry | report section 10.8: push tolerance is about ±5 %; raise `aPush` slightly (Onboard control → knobs → Apply edited values) |
| LQR-only run ends at once with `ONBOARD_PENDULUM_LOST` | pendulum outside the catch region at Start | catch-region light | hold it within ±20° of upright and nearly still, then Start |
| Hanging pendulum does not read ±180° | stale pendulum-down reference (or magnet moved) | θ1 on the page, “magnet” in the header | redo References → pendulum down; check the magnet if the header says weak |
| Arm runs past 100° after the catch | arm too fast at hand-over | `ctrl` and arm speed at the catch | raise `wc` (6.9 also passed all worst cases) |
| `CONTROL_OVERRUN` | the 1 ms loop is too slow | STATUS `controlOverruns` | report it: the onboard step needs optimising |

## HIL mode: what is missing

- Harness: the `Hardware` variant of the Rig (serial link from Simulink to the ESP32 at 4 ms).
- Firmware: a compact binary duty message (the JSON command path with its status reply is far too large
  for 250 Hz at 460800 baud) with a staleness trip of a few controller periods, as a third run source
  next to `Experiment` and `Onboard` in `Controller` (`engine.h`).
- Then measure the USB delay and set `cfg.link.u_delay` / `y_delay` in `harness_config.m`.
