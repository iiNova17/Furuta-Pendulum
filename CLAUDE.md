# Notes for AI coding agents (Claude, Codex, ...)

Project: Control II Furuta (rotary inverted) pendulum. ESP32-S3 firmware, a Python/Qt lab app
(Furuta Lab) for experiments and parameter estimation, MATLAB/Simulink models, and the LaTeX report.
See README.md for the folder map.

## Safety: motor motion

- Never arm the device, run a sequence or command motor motion on your own.
- Flashing firmware and passive acquisition (sensors only, motor unpowered) are fine without asking.
- Powered experiments only after the person at the rig confirms in chat, for that batch, that the
  center rod is removed, 12 V is on and the area is clear. The rig has a physical rod that locks the arm
  at center; it must be out for powered runs (it doubles as the "arm held" fixture for free swings).
- Keep the firmware travel limit, host heartbeat and averaged-current check active in any script,
  and stop on any exception. Close the Furuta Lab app before opening the serial port.
- Flashing reboots the MCU: arm center, pendulum-down reference, current zero and scale must be redone.

## App behavior the team asked for

- No artificial caps or refusals on motor-test inputs (duty, duration, levels, frequency) in the app.
  Show predictions and warnings instead. Keep only the real safety gates (12 V box, arm centered,
  commissioned limits, firmware validation).

## Modeling goal

- The model must reproduce the hardware closely enough for controller design, stability analysis,
  simulation and a digital twin. Lumped (per-torque-constant) parameters are fine; m, Lp, J0, J1 do not
  need to be separated. Only the gear ratio (34:1, 1496 counts/rev) and Vs = 12 V are trusted inputs.
- Final model (since 2026-11-20, after the rig was moved): `data/estimation/suite_20261120`, identified with
  `desktop/reidentify.py` / `reidentify_variants.py` (the claude_loop chain of `estimation.py`, arm terms also refined
  by simulation, pendulum timing 10 ms), exported to `matlab/model/furuta_params_20261120.m`. The earlier model
  (`data/estimation/claude_loop`, report Table 6) stays as `matlab/model/furuta_params.m`. Report sections 4 to 7
  match the code in `desktop/furuta_lab/estimation.py`.
- The rig has play and flex in the shaft coupling (motor encoder to arm): the pendulum reading lags the encoder by
  9-15 ms (14.7 ms for small, 9.1 ms for large arm motion). Not in the model; the controller compensates it.

## Workflow

- Larger changes are planned in `docs/HANDOFF_*.md` (self-contained: verified facts, bugs with
  file:line, design, tests, passive verification, a results table labelling unmeasured items as
  unverified), then implemented in a separate session ("Read docs/HANDOFF_X.md and do it").
- Tests: `.venv/Scripts/python.exe desktop/test_lab.py` and `desktop/test_estimation.py`.
- Report figures and numbers: `python report/figures/make_report_data.py` (needs numpy, scipy,
  matplotlib), then `latexmk -pdf furuta_model.tex` in `report/`.
- Recorded hardware runs live in `data/runs/` and are committed; do not delete or rewrite them.
- Rig acquisition suite (2026-11-20): `desktop/rig_suite.py` (passive steps), `rig_suite_batch1.py` (open loop),
  `rig_suite_batch2.py` (onboard closed loop, power probe first), `rig_suite_dryrun.py`, analyses
  `rig_suite_analysis.py`, `rig_suite_lag.py`, `rig_suite_backlash.py`. The firmware cannot sense the 12 V bus:
  runs without motor power are flagged by the scripts (`_nopower` runs + INVALID.txt).

## Simulation harness and controllers (matlab/harness)

- The plant is the identified model, never re-derived: `matlab/model/furuta_rhs.m` + the parameter file chosen by
  `harness_config(..., 'Model', ...)`: '20261120' (default, `furuta_params_20261120.m`) or 'table6' (`furuta_params.m`). The Simscape plant uses bodies from `lib/plant_bodies.m` whose equations are exactly
  eq. 89; `tests/test_harness.m` checks Simscape against `furuta_rhs` to round-off. Run the tests after
  any change to the plant, the rig blocks or the builders.
- `furuta_harness.slx` and the hybrid template files are generated (`build_harness.m`,
  `new_hybrid_controller.m`). Rebuild `furuta_harness.slx` instead of hand-editing it. The hybrid
  template files are hand-edited by their owners after generation: never regenerate them with
  'Overwrite' unless asked.
- Controller design is the team's work: do not fill in or tune gains in
  `controllers/hybrid/ctrl_hybrid_design.m` unless asked; harness changes only.
- All settings are parameters: rig/rates/limits in `harness_config.m`, controller knobs in the
  design file. Keep it that way (no constants buried in blocks).
- Two ways to run the controller on the rig (`docs/ONBOARD_CONTROL.md`): HIL (Simulink on the laptop,
  not built yet) and onboard (ESP32 firmware 1.12, `firmware/include/onboard.h`; app tab Onboard control). The
  onboard numbers are generated: `export_onboard.m` writes `firmware/include/onboard_design.h` (never hand-edit)
  and the replay files for `firmware/test/native_onboard.cpp`, which must keep passing (same duty as Simulink);
  `firmware/tools/onboard_rates.py` writes `onboard_rates.h` (LQR gain per controller period 1/2/4 ms; it reads the
  design model file). The native test runs twice: with `-DONBOARD_OBSERVER_DOUBLE` (must equal Simulink to 1e-9) and
  without (the observer's model in float as on the ESP32, tolerance 1e-4).
- Estimator: the model-assisted observer (`controllers/hybrid/est_observer.slx`, reference `est_observer_ref.m`,
  `tests/test_observer.m`; knobs `ctrlP.est.kind/nd/f_ab/f_trk/sub`, kind 0 = filtered differences). The design file
  designs on `D.design_model` (20261120); `D.fric.gain` 0 since 2026-11-20 (reasons in the design file).
- Arm breakaway (stick-slip) friction, 6 % / 6 % duty since 2026-11-20 (13 % / 14 % on 2026-11-07): in the model (`furuta_rhs`, `estimation.py`),
  the Simscape plant (`lib/arm_friction.m`, one function shared with `harness_plant_rhs`), the Simulink blocks
  (`swingup_energy.slx` `stick_gain`, `ctrl_hybrid.slx` friction compensation `fric.gain` / `fric.stick`) and
  firmware 1.8 / 1.9. The replay test runs with both terms on. Default current trip is 4 A (firmware and harness).
  Build that test with `-static` (Git's libstdc++ on PATH crashes it).
- Digital twins only render: the Furuta Lab tab Digital twin (`desktop/furuta_lab/twin_*.py`, `docs/TWIN.md`;
  meshes copied from `twin/`) and the Unreal one (`twin/`); packet format and conventions in `twin/README.md`
  (q_pend = theta1 - pi).

