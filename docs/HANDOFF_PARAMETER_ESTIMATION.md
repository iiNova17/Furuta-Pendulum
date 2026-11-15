# Handoff: step-by-step parameter estimation in Furuta Lab

Written 2026-11-14 for the implementing session (Opus); final revision the same day
after Nedal's decisions. Read all of it before starting. Background models:
`report/furuta_model.pdf` (LaTeX `report/furuta_model.tex`); equation
numbers are the report's. The report is Nedal's own draft and will be edited to match
this plan; do not edit it.

**Safety rule for every step: never arm the device, run a sequence, or command motor
motion. Flashing and passive acquisition are allowed. STOP and ask Nedal if anything
would energize the motor.** Every motor experiment in this plan is started by Nedal
from the app. Close the Furuta Lab app before using COM11 from a script.

**Nedal's standing preference:** no new hard caps on what he may enter for motor
tests. Predictions and warnings are fine; refusals are only for the existing safety
gates (12 V box, arm centered, limits commissioned, firmware limits).

## Decisions from Nedal (2026-11-14)

1. The purpose is a model good enough for controller design, stability analysis,
   simulation and a digital twin. The test of success is that the model output,
   driven by the recorded input, closely matches the hardware when nothing disturbs
   the rig.
2. Only the gear ratio (34:1, so 1496 counts per output revolution) and `Vs = 12 V`
   are trusted. `m`, `Lp`, `Larm`, `ηg` are not entered. Parameters that cannot be
   separated may stay lumped.
3. The report's two-stage procedure is not binding.
4. Excitation is a square wave.
5. The UI must show every step, and the final model as state space and transfer
   functions.

## Goal

Replace the "Parameter estimation" tab with a guided, numbered workflow, done on the
assembled rig only, that ends in (a) a nonlinear model, (b) linear state-space models
and transfer functions, (c) an overlay of model against hardware on a recording that
was not used for fitting. Fix the bugs listed below.

## The model and what the data can determine

Divide the arm equation of Eq. 93 by `Kτ` and write `G = m·g·Lp`. The plant is then
fully described by ten numbers, "per `Kτ`" (torque input = current in amperes):

```
i  = (V − Kb·θ̇0)/Rm            while applied duty ≠ 0;   i = 0 when duty = 0 (bridge open)
(Ja + Jp·sin²θ1)·θ̈0 + h·cos θ1·θ̈1 + 2·Jp·sin θ1·cos θ1·θ̇0·θ̇1 − h·sin θ1·θ̇1² + Beq·θ̇0 + τc±·sgn θ̇0 = i
h·cos θ1·θ̈0 + Jp·θ̈1 − Jp·sin θ1·cos θ1·θ̇0² + b1·θ̇1 − G·sin θ1 = 0
parameters: Rm, Kb, Ja, Jp, h, G, Beq, b1, τc+, τc−      (all but Rm, Kb divided by Kτ)
```

- Every entry of the linear models (Eq. 98, 99), every pole, and the nonlinear
  simulation depend only on these ten numbers. They need `Vs` and the gear ratio,
  nothing else.
- `Larm = g·h/G` comes out of the data as a by-product.
- SI values (`Kτ`, `ηg`, `m·Lp`, inertias in kg·m²) need one absolute reference. That
  is optional (Step 9). Until then the app shows SI numbers on the **nominal scale**
  `Kτ = Kb` (`ηg = 1`) and labels them so.
- `m` and `Lp` separately, and the report's Eq. 96 split (`J0`, `J1`, `b0`, `Jmo`),
  cannot be obtained with these sensors and are not needed. They are dropped.
- A motor-only test (the report's Stage 1) is not needed: `Rm`, `Kb` and the friction
  come from the assembled rig.

Estimation chain (reference code: `docs/prototypes/estimation_prototype.py` and
`docs/prototypes/twin_prototype.py`):

1. Free swing, arm held: `sin φ = −(Jp/G)·φ̈ − (b1/G)·φ̇`, `φ = θ1 − π`.
2. Electrical, driven samples of the square-wave runs: `i = V/Rm − (Kb/Rm)·θ̇0`.
3. Pendulum equation: `sin θ1 = (h/G)·cos θ1·θ̈0 + (Jp/G)·(θ̈1 − sin θ1·cos θ1·θ̇0²) + (b1/G)·θ̇1`.
4. Arm equation: `i = Ja·θ̈0 + G·Tr + Beq·θ̇0 + τc+·[θ̇0>0] − τc−·[θ̇0<0]`, with
   `Tr = (Jp/G)·(sin²θ1·θ̈0 + 2·sin θ1·cos θ1·θ̇0·θ̇1) + (h/G)·(cos θ1·θ̈1 − sin θ1·θ̇1²)`.
5. Refinement: adjust only `Ja`, `Beq`, `τc+`, `τc−` to minimise simulated-minus-
   measured arm angle, pendulum angle and current over 4 s windows.
6. Validation on a separate recording.

## Verified facts

Firmware 1.4 (read from `firmware/include/engine.h`, `hardware.h`, `src/main.cpp`):

- Zero applied duty puts the TB6612 in standby (`hw::output`, `hardware.h:59`): the
  motor is **open-circuit**, `i = 0`, no back-EMF braking. Eq. 88 holds only while
  `applied ≠ 0`. While driven, PWM-low is short-brake, so `V = d·Vs` on average.
- The current sensor is in series with the motor (`docs/HARDWARE.md`).
- Mechanics are sampled at 500 Hz internally; the stream sends every
  `500/streamHz`-th record, `streamHz ∈ {50, 100, 250}`. The app sets 100 on connect
  (`app.py:478`). Each record carries the latest ADC value (`current_mv`, a point
  sample) and the MCU-filtered value (`filtered_mv`).
- Each record has per-channel timestamps: `device_us` (encoder count), `pend_us`
  (AS5600), `adc_us` (current), `apply_us` (duty).
- The recorded `applied` duty already includes the slew limit and the 1 ms standby at
  a reversal. Square segments accept `phase` and `duty` (`parseSegment`).

Simulation results (both prototypes; plausible parameters, encoder 1496 counts/rev,
250 Hz stream, current noise 0.12 A per sample; simulated, nothing here was measured
on the motor):

| Quantity | App today | Least squares, assembled rig | After refinement |
| --- | --- | --- | --- |
| `Rm`, `Kb` | −9 %, +2 % | within 0.5 % | unchanged |
| `Ja` | +10 % | +3 % | −1 % |
| `Jp`, `h`, `G` (per `Kτ`) | `Jp` +0.5 % (needs typed geometry) | +2 % (max 3 %) | unchanged |
| `b1` | +7 % | within 2 % | unchanged |
| `τc+`, `τc−` | −41 % | −5 % | +1 % |
| `Beq` | +1040 % | +84 % | +10 % |
| `Larm` | not estimated | −0.2 % | — |
| Validation, 16 s open-loop: arm angle / pendulum angle / current fit | — | 65 / 97 / 85 % | 87 / 98 / 85 % |
| Same with the true parameters | — | 97 / 98 / 85 % | |
| Upright poles (true −19.81, −7.08, 0, 9.21) | — | — | −20.13, −7.09, 0, 9.22 |

- The current fit is capped near 85 % by measurement noise.
- Refinement takes about 12 s. Refining the pendulum group (`Jp`, `h`, `G`, `b1`) as
  well raised the fit to 90 % but moved those parameters 8 % away from the truth, so
  they stay at the least-squares values.
- At 100 Hz the inertia bias roughly doubles; use 250 Hz.
- `Beq` is weakly identifiable because Coulomb friction dominates (0.06 N·m in the
  simulation, chosen because Nedal's arm did not move below 12.5 % duty). The
  back-EMF term `Kb/Rm` is about 40 times larger, so it matters little. Show the ±.
- Centred square sweep at 30 % duty in the simulation: arm ±45°, pendulum ±83°.
- Optional scale reference by added mass (`Δm` ±1 %, `d` ±1 mm): `G`, `Kτ`, `ηg`
  within about ±4 %.
- The report's Eq. 98, 99 and 86 agree with numerical Jacobians of Eq. 93 and 84 to
  1e-10.

## Bugs found

Estimation (`desktop/furuta_lab/analysis.py`):

| # | Where | Problem | Fix |
| --- | --- | --- | --- |
| E1 | `motor_parameters` :119 | Uses motor-shaft angle (`motor_cpr` = 44); the model is written for the output shaft. Results differ by `N` and `N²`. | Use `countsPerRev` and `armSign` everywhere. |
| E2 | :120–122 | Speed smoothed over 80 ms, current over 30 ms. Different filters on the two sides of one equation bias the dynamic terms. | One zero-phase filter for every signal, then a second common filter on the whole equation. |
| E3 | :129, :162 | `sgn(ω)` is taken at every sample, including standstill, where friction is not `τc·sgn(ω)`. | Dead band on `sgn`, exclude sustained standstill. |
| E4 | :123–128 | Electrical fit includes open-circuit samples (`applied = 0`) and regresses on the noisy current. | Driven samples only; solve for the current. |
| E5 | :125–126 | Extra `driver_drop·sgn(i)` regressor; `sgn` of a noisy current near zero is noise. | Remove. |
| E6 | `regression` :93–103 | NNLS forces non-negative parameters, so an inverted current sensor or encoder direction shows as a 0 instead of a clear message. The 70/30 split inside one run often validates on the trailing rest. | Ordinary least squares with sign checks; validate on separate recordings. |
| E7 | `prepared_core` :105–112 | Ignores `pend_us` and `adc_us`; uses only `current_mv`. | Per-channel timestamps; use `filtered_mv` with delay compensation when the MCU filter is active. |
| E8 | `assembly_parameters` :142–170 | Requires typed `m`, `Larm`, `Lp`, motor-side `Kt` and `ηg`; fixes `h = m·Larm·Lp`. | The chain above; nothing typed. |
| E9 | — | No free-swing fit, no forward simulation, no linear models in the Python app. | Add. |
| E10 | `app.py:246` | Setup label defaults to "Arm + pendulum attached"; runs are mislabelled unless the dropdown is changed first. | The step that starts or imports a run sets its role; remove the dropdown. |

General (`desktop/furuta_lab/app.py`):

| # | Where | Problem | Fix |
| --- | --- | --- | --- |
| G1 | :669, :698 | `auto_deadzone` stays set when a pulse run is stopped with STOP or fails to start. Opening any older breakaway recording later then writes that old dead zone to device RAM. | Tie the flag to the run's path, or clear it in `stop()`, `load_recorded()` and on a failed start. |
| G2 | :812 | `refresh()` calls `submit()` from the timer. If a task is busy it raises inside the timer and the finished run is never loaded. | Keep the path pending and load it when not busy. |
| G3 | :681–686 | `start_record()` runs before `submit()`. If `submit()` raises, a recording stays open. | Check `busy` first, or close the recording on failure. |
| G4 | `protocol/SPEC.md` | Firmware 1.4 `deadzonePos`/`deadzoneNeg` are not documented. | Add a short "Firmware 1.4 additions" section. |

## Plan

### 1. New module `desktop/furuta_lab/estimation.py` (pure functions, no Qt)

Lift the algorithms from the two prototypes; they are the reference for filters,
masks, regressors, the simulator and the refinement.

- `prepare(rows, config, opts) -> dict`: longest contiguous valid span (reuse
  `analysis.spans`); channel times as in E7; uniform grid at the median `device_us`
  step; `θ0 = armSign·(count − count[0])·2π/countsPerRev`;
  `θ1 = unwrap(pendSign·(pend_raw − pendDownRaw)·2π/4096 + π)`;
  `i = polarity·(mV − currentZeroMv)/sensitivityMvA`; `V = applied·Vs`.
  Current source: `filtered_mv` when the run's `config.filter.type != 'bypass'`, with
  its low-frequency group delay subtracted from `adc_us` (biquad:
  `scipy.signal.group_delay` near DC at `adcHz`; EMA: `(1−α)/α` samples; moving:
  `(N−1)/2` samples), else `current_mv`.
  Then zero-phase Butterworth (order 2, forward-backward) at `fd` on every signal and
  central differences. Returns raw and filtered arrays plus masks (`edge`, `driven`,
  `moving`, `resting`) so the UI can plot them.
- Fits: `fit_freeswing`, `fit_electrical`, `fit_pendulum_equation`,
  `fit_arm_equation`. Each accepts a **list** of prepared runs (filter each run
  separately, then stack rows), filters the left side and every regressor column with
  the same low-pass `fp`, decimates by `fs/(4·fp)`, and returns coefficients,
  approximate standard errors, R², condition number of the column-normalised design,
  row counts, and the measured and predicted left side for plotting. Option on the arm
  equation: one symmetric `τc` instead of `τc+`, `τc−`.
- `simulate(params, prepared_run, resync_s=0)`: the batched RK4 simulator of
  `twin_prototype.py` (two substeps per sample; friction `τc±·tanh(θ̇0/0.1)`; open
  circuit when duty is 0). `resync_s = 0` simulates the whole run from its first
  state; otherwise it restarts from the measured state every `resync_s` seconds.
  Returns `θ0`, `θ1`, `θ̇0`, `i` and fit % = `100·(1 − ‖y − ŷ‖/‖y − ȳ‖)` for arm
  angle, pendulum angle and current.
- `refine(params, runs, window_s=4, free=('Ja','Beq','tcp','tcn'))`: as in the
  prototype, with a progress callback and a cancel event. Returns the new parameters,
  cost before and after, and the change of each parameter.
- `linear_models(params, Vs, Ts)`: Eq. 99 and Eq. 98 with input volts and with input
  duty (`Bd = Vs·B`); outputs `θ0`, `θ1` and current (`C` row `[0 0 −Kb/Rm 0]`,
  `D = 1/Rm`); poles; transfer functions `θ0/V`, `θ1/V` (`scipy.signal.ss2tf`, also as
  zeros, poles, gain); controllability rank and condition; zero-order-hold discrete
  model at `Ts`; Eq. 86 when `Lm` is entered. Also the equivalent friction voltages
  `Vc± = Rm·τc±` and duties `Vc±/Vs`.
- `scale(params, mode, ...)` (optional Step 9): nominal `Kτ = Kb`; A added mass
  (`added_mass` in the prototype) with propagated ±; B weighed `m` and balance-point
  `Lp`; C assumed `ηg`. Returns SI values, `ηg = Kτ/Kb`, `m·Lp = G/g`.
- `checks(...) -> list[(level, text)]`, with the remedy in the text. At least:
  `Rm > 0`; `Kb > 0` (else flip `armSign` or swap motor leads); current polarity from
  `sign(Σ applied·i)` over driven samples (else set polarity −1); `h/G > 0` (else
  flip `pendSign`); all ten parameters > 0; `Ja·Jp > h²`; `Jp/G` driven within 5 % of
  the free swing; `Larm` plausible (5–40 cm); relative standard error > 30 % marked
  "weakly identified"; refinement changed a parameter by more than 30 %; arm moved
  more than 2 counts during the free swing; pendulum passed ±150° from hanging; duty
  saturated or a fault ended the run; stream rate below 250 Hz or `dropped > 0`;
  current zero missing; fit % below 80 % on the validation run.
- `square_sweep(amplitude, freqs, seconds_each)` → firmware segments: 0.5 s zero;
  one `square` segment per frequency with `phase = π/2`, `duty = 0.5` and a whole
  number of periods; 1 s zero. `predict_arm_swing`: `ω ≈ (A·Vs − Vdead)/Kb`,
  `±ω/(4f)` about the centre (an upper bound).
- Defaults: `fd = 40 Hz`, `fp = 10 Hz` at 250 Hz (`fd = 20` at 100 Hz);
  `ω_min = 0.3 rad/s`; standstill = below `ω_min` for ≥ 0.15 s; edge trim `3/fp`
  seconds. All editable in the UI.

Remove `motor_parameters`, `assembly_parameters`, `regression`, their two tests and
the app code that calls them. Keep everything else in `analysis.py`.

### 2. Session file

`data/estimation/<name>/session.json`, written on every change: `Vs`, current
polarity, filter settings, optional `Lm`, per step the recordings used and the
result with a timestamp, the accepted model, the scale mode. Refitting a step marks
every later step "stale". Buttons: New session, Open session. The last session
reopens on start.

### 3. UI

Put the page in its own file (`desktop/furuta_lab/estimate_page.py`, a class that
takes the `LabApp` and uses its `submit`, `report`, `start_sequence`, `device`), not
in `app.py`. Add `desktop/furuta_lab/fitview.py`: a `pg.GraphicsLayoutWidget` with
stacked, x-linked panels, legends, and grey shading for masked samples; add it to
`right_stack`. Tables go to the Messages pane as markdown: symbol, value,
± (approximate), unit, check.

Every step has the same layout: what to do physically (one or two lines), inputs,
recordings list ("Add the open recording", "Add from folder…", remove), a run button
where a motor run is needed (labelled "— MOVES MOTOR", same gates as
`authorize_run`), "Show prepared signals" (raw and filtered `θ0`, `θ̇0`, `θ̈0`, `θ1`,
`i`, `V`, masks), "Fit", and a one-line status (`✓ Rm 5.48 Ω · Kb 0.391 V·s/rad`,
`stale`, or what is missing). A run started from a step is added to that step's list
automatically and tagged in its manifest (`metadata.estimation_step`).

**Step 0 · Setup.** `Vs` (default 12 V). Encoder scale line from the profile.
Checklist: current zero set, arm centered, pendulum-down set, limits commissioned.
"Apply estimation recording settings" (SET_CONFIG `streamHz = 250`, MCU current
filter = 2nd-order Butterworth 40 Hz at the device `adcHz`) and "Restore normal
settings". Note: the overcurrent trip uses the filtered current. Optional `Lm` (mH).

**Step 1 · Free swing (motor off).** Text: hold the arm, release the pendulum from
30–40°. Button "Record 20 s" (passive recording, no arming). Panels: `φ` measured
against the simulated swing; regression left side against prediction. Table: `ωn`,
`ζ`, `Jp/G`, `b1/G`, period from zero crossings as a cross-check.

**Step 2 · Square-wave runs (pendulum hanging).** Three presets, each editable:
Run A (1.5, 2, 3, 4, 2.5 Hz), Run B (2, 3, 4, 5, 2.5 Hz), Validation (1.75, 2.5,
3.5, 2 Hz); 4 s per frequency; amplitudes default to 30 %, 40 %, 35 %, each raised to
at least twice the stored dead zone. A line shows the predicted arm swing against the
commissioned travel and the slew rate needed; both warn but do not block. "Preview"
plots the duty. One run button per preset. A and B go to the fit list, Validation to
the validation list. If a run ends in `ARM_TRAVEL` the data before the fault is still
usable; say so.

**Step 3 · Electrical fit.** Panels: measured current against `V/Rm − (Kb/Rm)·θ̇0`;
residual; `V` and `θ̇0`; masked samples shaded. Table: `Rm`, `Kb`.

**Step 4 · Pendulum-equation fit.** Panels: `sin θ1` against the prediction with the
three contributions drawn separately; residual. Table: `h/G`, `Jp/G`, `b1/G`,
`Larm`; `Jp/G` and `b1/G` beside the Step 1 values.

**Step 5 · Arm-equation fit.** Panels: measured current against the prediction with
the inertia, pendulum-reaction, viscous and Coulomb contributions drawn separately;
residual. Table: `Ja`, `G`, `Beq`, `τc+`, `τc−` (per `Kτ`), and the friction duties
`Rm·τc±/Vs` beside the stored dead zone.

**Step 6 · Refine by simulation.** Shows the fit-run overlay with the least-squares
model. "Refine" runs with progress and Cancel. Then: overlay before and after, fit %
before and after, a table of what changed. Buttons "Accept refined model" and "Keep
least-squares model".

**Step 7 · Model against hardware.** Recording selector (default: the validation
run; any run can be added, including later closed-loop runs, because the input is the
recorded duty). "Re-sync every [0] s (0 = whole run)". Panels: arm angle, pendulum
angle, arm speed, current, duty; measured and model on the same axes. Fit % per
signal, RMS errors in degrees and amperes. States that an open-loop arm angle drifts.

**Step 8 · Model.** Shown in the Messages pane and a pole-zero plot:
- the ten parameters with ±, nominal-scale SI values, `Larm`;
- upright: `A`, `B` (volts), `Bd` (duty), `C`, `D`; poles with the unstable pole's
  time constant; `θ0(s)/V(s)` and `θ1(s)/V(s)` as polynomials and as zeros, poles,
  gain; controllability; discrete `Ad`, `Bd` at `Ts` (default 1 ms, editable);
- hanging: the same;
- nonlinear equations with the numbers filled in.
Export: `parameters.json`; `furuta_model.m` (parameter struct, both state-space
models as `ss`, `tf`, `c2d`); `furuta_rhs.m` (nonlinear `ẋ = f(x, V, p)` including
the open-circuit rule and friction); `parameters.tex` (table rows).

**Step 9 · Physical units (optional).** Radio: nominal (`ηg = 1`); A known added
mass (inputs `Δm`, `d` and their tolerances; "Record 20 s" with the mass fitted;
warning when `d` is near the centre of oscillation `g·Jp/G`); B weighed `m` and
balance-point `Lp`; C assumed `ηg`. Shows `Kτ`, `ηg`, `m·Lp` and SI values with ±.
States that Steps 7 and 8 do not change.

### 4. General fixes

G1–G4 from the table above. Let the segment builder set `phase` and `duty`.

### 5. Tests (`desktop/test_lab.py`)

Import the simulator from the prototypes (move it to
`desktop/furuta_lab/simbench.py` or keep importing from `docs/prototypes`).

- Free swing: `Jp/G` within 1 %, `b1/G` within 10 %.
- Assembled-rig least squares, 250 Hz, noise 0.12 A: `Rm`, `Kb` within 3 %; `Ja`
  within 8 %; `Jp`, `h`, `G` within 6 %; `b1` within 10 %; `τc±` within 12 %; `Larm`
  within 2 %.
- Refinement: validation arm-angle fit rises and is ≥ 80 %; `Ja` within 4 %; `τc±`
  within 5 %; unstable upright pole within 3 % of the true one.
- Simulator: fit % > 95 % with the true parameters on a noise-free run; re-sync works.
- Open-circuit samples are excluded from the electrical fit.
- MCU-filtered current with delay compensation: `Ja` within 3 % of the unfiltered case.
- Sign checks: negated current → "polarity" message; negated `θ1` → "flip pendSign".
- Linear models equal numerical Jacobians of the simulator (1e-6) and do not change
  when the scale mode changes; `ss2tf` poles equal the eigenvalues; discrete model
  equals `scipy.signal.cont2discrete`.
- Square sweep builder: whole periods, `phase = π/2`, segment count, total time.
- Added mass with exact inputs: `G`, `Jp` within 3 %; insensitivity warning.
- Session file round trip and stale marking.
- GUI: build the page, load synthetic recordings, press Fit on Steps 1, 3, 4, 5,
  Refine, Step 7, Step 8; assert tables, plots and export files; G1 regression test.
- Existing tests keep passing, except the two removed with the old estimators.

### 6. Documentation

Rewrite "Two parameter-estimation datasets" in `docs/FOCUSED_GUIDE.md` for the new
steps. Add the 1.4 fields to `protocol/SPEC.md`.

## Verification — all passive, motor never commanded

1. `.venv/Scripts/python.exe desktop/test_lab.py` passes.
2. Passive hardware check with the app closed: connect, apply the estimation
   recording settings, record 60 s. Report the measured stream rate, `dropped`, link
   errors and ADC overruns at 250 Hz. If samples drop, keep 100 Hz as the default and
   say so.
3. Open the app, walk every step with synthetic recordings, and screenshot or
   describe each step's plot and table, including the Step 8 model text.
4. Run the existing breakaway recording `data/runs/20261107_064740_883200` through
   "Show prepared signals" to confirm real data loads (it has no usable motion; that
   is expected).
5. Report as a table: item, result, verified or not. Label every motor-dependent
   result "unverified — needs Nedal's run".

Then give Nedal the run order: Step 0 → 8.

## Optional extensions (only if Nedal approves)

1. Motor-only test (the report's Stage 1) to separate the motor's own inertia.
2. Stick–slip friction in the simulator, using the measured breakaway duty.
3. Live twin: model output overlaid on the live stream.
4. Full-rate capture during a run (`CAPTURE_BURST` with `withRun`): 500 Hz mechanics
   and up to 2 kHz current, downloaded after the run (about twice the run time).

## Report changes (Nedal edits the report; listed here for reference)

1. Section 5 and Fig. 3: one procedure on the assembled rig — free swing, square-wave
   runs, least squares, refinement, validation. The motor-only stage becomes optional.
2. Section 4.2: replace Eq. 95 by the pendulum-equation and arm-equation regressions
   in ratio form; replace the first line of Eq. 92 by `i = V/Rm − (Kb/Rm)·θ̇0`.
3. State which quantities are identified: the ten per-`Kτ` parameters and `Larm`.
   Remove or mark as not identified: Eq. 96 (`J0`, `J1`, `b0`), `m`, `Lp`, `Jmo`.
   State that the linear models and the simulation do not depend on `Kτ` or `ηg`.
4. Section 2.6: zero duty is driver standby (open circuit); Eq. 88 applies while driven.
5. Section 4 opening: 250 Hz stream, 2 kHz current sampling with a 40 Hz MCU filter
   (not 200 Hz and 500 Hz); data go to the Furuta Lab app, not MATLAB.
6. Section 5: two filters (every signal at 40 Hz, the whole equation at 10 Hz);
   standstill samples excluded; Coulomb friction per direction.
7. Add the refinement step and the fit % definition; report the validation overlay.
8. Section 3.3: friction compensation per direction, `Vc± = Rm·τc±/Kτ`.
