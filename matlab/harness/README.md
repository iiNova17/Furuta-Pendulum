# Controller harness: one simulation, everyone's controllers

A shared Simulink model of the rig for **developing, testing and comparing controllers**, laid out
to match the course brief (MCT411/413s/414 Furuta project). It is development infrastructure: it does
not contain anybody's controller design. The controller template has placeholder gains (TODO) that
each owner fills in.

```
ref --> [Controller] --duty--> [Rig] --y = [theta0; theta1; i]--+
             ^--------------------------------------------------+
Controller: hybrid template = estimator -> PID / pole placement / LQR / swing-up -> Stateflow supervisor
Rig (Variant Subsystem):  Simulation = HIL link -> firmware -> plant (identified model) -> sensors
                          Hardware   = USB link to the ESP32 (placeholder until the bridge is built)
```

## Two ways to run a controller on the rig (report section 8.2, `docs/ONBOARD_CONTROL.md`)

| | 1. Hardware in the loop | 2. Onboard (product mode) |
|---|---|---|
| Controller runs in | Simulink on the laptop, this harness with `cfg.rig = 'Hardware'` | ESP32 firmware 1.12 (`firmware/include/onboard.h`), started from the app's Onboard control tab |
| Loop | USB both ways (delay unmeasured) | inside the 1 kHz firmware loop, 4 ms period |
| Status | not built (Hardware variant + firmware duty message) | built and run on the rig: firmware 1.12 with the model observer (2026-11-20), equal to the harness to 1.2e-14 (observer in double) / 3e-6 duty (float, as flashed); results in `docs/ONBOARD_CONTROL.md` |

Breakaway friction (stick-slip; 6 % duty in the 20 Nov model, 13 % / 14 % in Table 6) is everywhere: the Simscape plant ("arm friction" block =
`lib/arm_friction.m` = `furuta_rhs`), the swing-up's motor inverse (`swingup_energy.slx`, `D.swing.stick_gain`), the
balance friction compensation (`ctrl_hybrid.slx`, `D.fric.gain` / `D.fric.stick`) and firmware 1.8 / 1.9. A plant
without it: `run_sim(..., 'Plant', struct('tsp', 0, 'tsn', 0))`.

Estimator (`ctrl_hybrid.slx`, switch on `ctrlP.est.kind`): 0 = filtered differences, 1 = the model-assisted observer
(`controllers/hybrid/est_observer.slx`, reference `est_observer_ref.m`, test `tests/test_observer.m`), 2 = its
tracking-loop fallback. The observer predicts the state 3 ticks (12 ms) forward through the sent duties: the rig's
pendulum reading lags the motor encoder by 9-15 ms (shaft coupling). Harness comparison on the 20 Nov plant, lag
9 / 12 / 15 ms: old controller 50-65 % of the time at the duty limit, 1.65 A rms; observer + retuned LQR 0.24-0.97° rms,
no saturation, about 0.2 A (`controllers/hybrid/observer_compare.m`). The pendulum lag of the simulated sensor
comes from the parameter set (`p.pend_delay`, 10 ms for the 20 Nov model, 7 ms for Table 6). Firmware current trip:
4 A, as on the rig.

One design, two copies: `controllers/hybrid/export_onboard.m` writes `firmware/include/onboard_design.h`
from `ctrl_hybrid_design.m`, plus replay files that `firmware/test/native_onboard.cpp` uses to require the
same duty as Simulink. Re-run it after any change to the design file, then rebuild and flash.

## Where to edit what (everything is a parameter)

| What | File | Examples |
|---|---|---|
| Rig, rates, HIL link, sensors, firmware limits, metric bands | `harness_config.m` | `cfg.Ts` (controller period, 4 ms), `cfg.fw.Ts` (firmware, 1 ms), `cfg.link.y_delay`, `cfg.rig`, `cfg.act.max_duty` |
| Controller knobs, one section per owner | `controllers/hybrid/ctrl_hybrid_design.m` | `stab`, `pid.*`, `pp.poles`, `lqr.Q/R`, `swing.*`, `sup.*` (Stateflow guards) |
| Test cases | `furuta_scenario.m` | `balance`, `push` (standard disturbance), `arm_step`, `swingup` |
| Plant | `matlab/model/furuta_params_20261120.m` (default) or `furuta_params.m` (Table 6): `harness_config(mode, 'Model', '20261120' / 'table6')` | never edited here |

Per run without editing files: `run_sim(..., 'Params', struct('lqr', struct('R', 50)))` (nested overrides
merge) and `cfg = harness_config(); cfg.Ts = 2e-3; run_sim(..., 'Cfg', cfg)`.

## How it maps to the brief

| Brief | In the harness |
|---|---|
| 4.1 nonlinear model, Simscape model, "validate that the two are consistent" | Plant = Simscape Multibody whose equations are exactly report eq. 89 (see below). `validate_open_loop` plots Simscape vs nonlinear (eq. 84/88) vs linear (eq. 106); `tests/` prove Simscape = equations to ~1e-8 deg |
| 4.2 eigenvalues, controllability, observability | `validate_open_loop` prints them (from report eq. 104/106) |
| 4.3 PID, pole placement, LQR + energy swing-up | one Subsystem Reference file each: `stab_pid.slx`, `stab_pole_placement.slx`, `stab_lqr.slx`, `swingup_energy.slx` |
| 4.4 hybrid Stateflow supervisor, guards on theta1 and its rate, safety fallback | `ctrl_hybrid.slx / Supervisor`: SwingUp <-> Stabilize, any -> SafeStop (priority 1), guards from `ctrlP.sup` |
| 4.6 dashboard shows the active hybrid state | controller output `mode` (0 SafeStop, 1 SwingUp, 2 Stabilize), logged with every run |
| 4.7 metrics | `R.metrics` / `compare_controllers`: settling, rise and peak time, overshoot, IAE/ISE/ITAE, recovery, max deviation, steady-state errors, peak/RMS u, saturation; robustness via `'MonteCarlo'` |
| 5 HIL, the controller incl. Stateflow runs against the real rig | same `ctrl_hybrid.slx`, Rig variant `Hardware` (`cfg.rig = 'Hardware'`) once the USB bridge exists |
| 7 clearly named subsystems, notation | Controller / Rig / Plant / Supervisor / per-law files; states theta0, theta1 as in Figure 1 |
| 8.1 Phase 1: model + open-loop validation | `validate_open_loop('SaveDir', 'validation')` |

## The hybrid controller template (`controllers/hybrid/`)

```
ctrl_hybrid.slx         integration owner + Stateflow owner
  estimator             xh = [theta0; theta1 (wrapped, 0 up); dtheta0; dtheta1] (filtered differences)
  PID                -> stab_pid.slx             PID owner          in: xh, r, active   out: u
  Pole placement     -> stab_pole_placement.slx  pole-placement owner   (same interface)
  LQR                -> stab_lqr.slx             LQR owner              (same interface)
  select stabilizer     ctrlP.stab = 1 / 2 / 3, then friction compensation (ctrlP.fric)
  Swing-up           -> swingup_energy.slx       swing-up owner     in: xh, r   out: u, E (pendulum energy)
  Supervisor            Stateflow: SwingUp / Stabilize / SafeStop -> duty, mode
ctrl_hybrid_design.m    all knobs; gains derived from them (K = dlqr / place on report eq. 104 at cfg.Ts)
```

- Every law file is edited by one person only (`.slx` files do not merge in git).
- `active` = "the supervisor was in Stabilize one period ago": a rising edge resets the PID integrator
  when the pendulum is caught (the hybrid reset the brief's MCT411 part is about).
- The template outputs zero duty until gains are filled in. `new_hybrid_controller('Overwrite', true)`
  regenerates all five files (your edits are lost).
- Swing-up design tools (report section 10): `swingup_knobs` (knobs from physics), `swingup_catch_region`
  (where the LQR can take over), `swingup_workbench` (one design -> numbers + energy plot), `swingup_sweep`
  (one knob or a 2D grid, `'Part'` for parallel MATLABs), `swingup_corner_test` (12 worst cases),
  `swingup_report_data` (-> `report/figures/swingup_data.mat`, then `make_swingup_figures.py`).

```matlab
run_sim('ctrl_hybrid.slx', 'swingup', 'Plot', true)
c = repmat({'ctrl_hybrid.slx'}, 1, 3);                            % PID vs PP vs LQR, brief 4.7
T = compare_controllers(c, {'push'}, 'Params', {struct('stab', 1), struct('stab', 2), ...
    struct('stab', 3)}, 'Labels', {'PID', 'PP', 'LQR'})
```

## The Rig

| Block | What it is | Rate | Source |
|---|---|---|---|
| HIL link | command / measurement delays (`cfg.link`, 0 until measured) and the controller <-> firmware rate change | `cfg.Ts` / `cfg.fw.Ts` | |
| Firmware | `FirmwareActuator`: travel trip 120 deg, overcurrent 1.5 A (filtered, 10 ms), duty clip 0.5, slew limit, through zero before reversing, 10-bit PWM | 1 ms | `firmware/include/engine.h`, report 5.1 |
| Plant | motor + driver (eq. 71, 88) and friction (eq. 78) as Simulink blocks, arm + pendulum as Simscape Multibody (eq. 89) | RK4 0.5 ms | report 2-6, Table 6 |
| Sensors | `FurutaSensors`: encoder 1496 counts/rev, AS5600 12 bit at 500 Hz with 7 ms lag, current noise through the 40 Hz Butterworth | 1 ms | report Table 1, 5.2 |

### Why the Simscape plant can be trusted

The report identifies lumped parameters divided by the unknown torque constant K_tau (section 4.1.1).
With K_tau = 1 N*m/A they are SI values and the motor current is the joint torque. An arm plus a
slender pendulum (no inertia about its own long axis, report eq. 9) with

    G = m g Lp,   Jp = J1 + m Lp^2,   h = m Larm Lp,   Ja = Ia + m Larm^2

has exactly eq. 89 as its equations of motion (`lib/plant_bodies.m`; Larm comes out as the identified
6.81 cm). Simscape is the report's model drawn as bodies, and the tests prove it (agreement with
`furuta_rhs` to round-off). The CAD meshes (`plant3d/`) are visual only; the pendulum sits at the
identified 68.1 mm, ~3 mm beyond the arm mesh's hinge hole (CAD 65 mm).
3D view: `run_sim(..., 'Show3D', true)`.

## Simulation vs measured (real rig recordings)

```matlab
% once, in a shell:  python matlab/harness/replay/export_runs.py
addpath replay
compare_to_run('20261115_100519_238300')              % validation run 1: plot + fit %
compare_to_run('20261115_101315_017000', 'Resync', 2) % restart from the measured state every 2 s
```

`export_runs.py` converts the recordings in `data/runs` (read-only) with the team's own code and the
final estimation session's settings (`data/estimation/claude_loop`). `compare_to_run` drives the
Simscape plant with the duty the real firmware applied, from the measured state, and plots measured vs
simulated theta0, theta1 and current with the report's fit %. It reproduces Table 4 (validation runs 1-3,
open loop: arm 89 / -3 / 42 %, pendulum 90 / 84 / 85 %, current 88 / 89 / 88 %); a test checks run 1.
Runs available: 2 free swings, 6 fit runs, 3 validation runs, 1 stall test (roles in each .mat).

## Running

```matlab
cd matlab/harness
harness_setup                                         % paths
R = run_sim('ctrl_hybrid.slx', 'push', 'Plot', true); % one run: R.t, R.x, R.y, R.u, R.mode, R.metrics ...
harness_load('ctrl_hybrid.slx', 'swingup')            % or: load + press Run in Simulink
validate_open_loop('SaveDir', 'validation')           % Phase 1 figure
runtests('tests/test_harness.m')                      % harness self-checks
```

`furuta_harness.slx` is generated by `build_harness.m`: rebuild rather than hand-edit it. Opening it by
hand loads its variables automatically (no red blocks). A run takes a few seconds.

Other controller forms also fit the slot (same interface: in `y`, `r`; out `duty`, `mode`, `dbg`):
a MATLAB System object subclassing `FurutaController` (template `controllers/template/MyController.m`),
or any Simulink subsystem file (`new_subsystem_controller('name')`). `OpenLoopDuty` replays a duty
sequence. `LqrBalance` / `EnergySwingup` are early sketches from setting the harness up, not team designs.

| Option of `run_sim` | Effect |
|---|---|
| `'Mode', 'ideal'` | perfect sensors and actuator |
| `'Perturb', 1, 'Seed', n` | plant parameters drawn from the report's uncertainties (Table 6) |
| `'Plant', struct(...)` | specific plant parameters (model mismatch on purpose) |
| `'Cfg', cfg` | any `harness_config` change for this run |
| `'Show3D', true` | Mechanics Explorer |

## Sharing with older MATLAB (R2023a / R2023b)

```matlab
export_release()            % -> dist/furuta_harness_R2023a.zip   (or export_release('R2023b'))
```

Copies `matlab/harness` + `matlab/model` (same layout), re-saves every `.slx` in the R2023a format
(`Simulink.exportToVersion`, no blocks replaced), and zips it; the repo files are not touched. The zip
includes the exported rig recordings, so `compare_to_run` works without the repo. Recipient: unzip,
`cd harness`, `harness_setup`, then as in Running. Needs Simulink, Simscape, Simscape Multibody and the
Control System Toolbox. The package passes `tests/test_harness.m` unzipped on its own (checked in
R2026a; not yet run in an actual R2023 install). Re-export after changing controllers or the harness.
If a recipient runs `build_harness` themselves, it regenerates `furuta_harness.slx` in their release.

## Assumptions to replace with measurements

| Setting | Value | Status |
|---|---|---|
| `cfg.Ts` | 4 ms | what the HIL link achieves is not measured yet |
| `cfg.link.y_delay`, `u_delay` | 0 | measure the USB round trip |
| `cfg.sens.i_noise_sd` | 0.05 A before the MCU filter | assumed |
| `cfg.act.slew` | 100 duty/s (firmware maximum) | the profile default is 0.5/s: raise it on the rig before closed-loop tests |
| pendulum lag | 7 ms as a pure delay (in 1 ms steps) | report 5.2.1; cause not known |

Not modelled: gearbox play (< 2 deg), motor inductance (report 7.3), friction flattening above ~2 rad/s
(report 5.4.4).
