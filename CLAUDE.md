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
