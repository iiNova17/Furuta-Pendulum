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
- Final model: `data/estimation/claude_loop` (Furuta Lab session), exported to `matlab/model/`.
  The report (`report/furuta_model.tex`) documents it; report sections 4 to 7 match the code in
  `desktop/furuta_lab/estimation.py`.

## Workflow

- Larger changes are planned in `docs/HANDOFF_*.md` (self-contained: verified facts, bugs with
  file:line, design, tests, passive verification, a results table labelling unmeasured items as
  unverified), then implemented in a separate session ("Read docs/HANDOFF_X.md and do it").
- Tests: `.venv/Scripts/python.exe desktop/test_lab.py` and `desktop/test_estimation.py`.
- Report figures and numbers: `python report/figures/make_report_data.py` (needs numpy, scipy,
  matplotlib), then `latexmk -pdf furuta_model.tex` in `report/`.
- Recorded hardware runs live in `data/runs/` and are committed; do not delete or rewrite them.

## Simulation harness and controllers (matlab/harness)

- The plant is the report's model, never re-derived: `matlab/model/furuta_rhs.m` + `furuta_params.m`
  (Table 6). The Simscape plant uses bodies from `lib/plant_bodies.m` whose equations are exactly
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
  not built yet) and onboard (ESP32 firmware 1.9, `firmware/include/onboard.h`; app tab Onboard control). The
  onboard numbers are generated: `export_onboard.m` writes `firmware/include/onboard_design.h` (never hand-edit)
  and the replay files for `firmware/test/native_onboard.cpp`, which must keep passing (same duty as Simulink);
  `firmware/tools/onboard_rates.py` writes `onboard_rates.h` (LQR gain per controller period 1/2/4 ms).
- Arm breakaway (stick-slip) friction, measured 13 % / 14 % duty: in the model (`furuta_rhs`, `estimation.py`),
  in the design (`ctrl_hybrid_design.m`: swing-up `stick_gain`, balance `fric.gain` 1 / `fric.stick` 1) and in
  firmware 1.8 / 1.9, but not yet in the Simscape plant or the Simulink blocks: `harness_config.m` sets
  `cfg.p.tsp/tsn = 0` for the plant (keeps `cfg.breakaway` for the design) until `docs/HANDOFF_BREAKAWAY.md` is done.
  The replay test runs with both breakaway terms off. Default current trip is 4 A.
  Build that test with `-static` (Git's libstdc++ on PATH crashes it).
- The digital twin (`twin/`) only renders; packet format and conventions in `twin/README.md`
  (q_pend = theta1 - pi).

