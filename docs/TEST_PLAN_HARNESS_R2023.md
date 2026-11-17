# Test plan: simulation harness on MATLAB R2023a / R2023b

For an AI agent (or a person) on a teammate's machine. Read this whole file, then do it:
"Read docs/TEST_PLAN_HARNESS_R2023.md and do it."

## Goal

Check that the harness (`matlab/harness`), built and tested in R2026a, works in R2023a/b.
Simulation only: **no hardware, no serial port, no Furuta Lab app, no motor**. Nothing here moves the rig.

## Rules for the agent

- Work on branch `harness-r2023a`. On this branch every `.slx` is saved in the R2023a format.
- Do not edit, re-save, regenerate or "fix" any repo file. In particular, do not run `build_harness`,
  `new_hybrid_controller` or `export_release`, and do not save any model you open (close with
  `close_system(name, 0)` or `bdclose('all')`). The only file you write is the results file (step 9).
- If a step fails, record the exact error text (the full message plus the identifier from `ME.identifier`)
  and go on to the next step. Do not try workarounds that change files.
- Do not use `restoredefaultpath`.

## Facts (verified in R2026a, 2026-11-18)

- The six models (`matlab/harness/furuta_harness.slx`, and in `matlab/harness/controllers/hybrid/`:
  `ctrl_hybrid`, `stab_lqr`, `stab_pid`, `stab_pole_placement`, `swingup_energy`) were exported with
  `Simulink.exportToVersion(..., 'R2023A')`. `Simulink.MDLInfo` reports `R2023a` for each, and
  `slexportprevious.utils.displayReplacedBlocks` finds no replaced blocks.
- A copy of only `matlab/harness` + `matlab/model` passes all 9 tests in `tests/test_harness.m` in R2026a.
- The `.m` code was scanned for functions newer than R2023a (e.g. `clip`, `isuniform`, `paddata`) and none
  were found. **Nothing has been run in an actual R2023 install yet: that is what this plan checks.**
- Required products: MATLAB, Simulink, Simscape, Simscape Multibody, Control System Toolbox.
- Expected test runtime: a few minutes (the hybrid template test is the slowest, about 1 min).

## Steps

1. **Get the branch.** `git clone -b harness-r2023a https://github.com/iiNova17/Furuta-Pendulum.git`, or
   `git fetch && git checkout harness-r2023a && git pull` in an existing clone. Record `git log --oneline -1`.
2. **Environment.** In MATLAB, record `version` and, from `ver`, whether each required product is present.
   If one is missing, note it, still run the steps that don't need it, and expect the others to fail.
3. **Model files.** For each of the six `.slx` files, record `Simulink.MDLInfo(file).ReleaseName`
   (expected: `R2023a`).
4. **Setup.** `cd matlab/harness`, `harness_setup`. Check that `which('furuta_rhs')` points into
   `matlab/model` of this clone.
5. **Test suite.** `r = runtests('tests/test_harness.m'); disp(table(r))`. Record pass/fail per test plus
   any error text. The replay test prints a line; record it (expected:
   `fit arm 89 %, pendulum 90 %, arm speed 92 %, current 88 %`).
   If `test_hybrid_template_plumbing` creates `controllers/ctrl_harness_selftest/`, that is normal: it is
   temporary and the test removes it.
6. **Open each model.** `bdclose('all'); harness_setup;` then for each of the six files run
   `load_system(file)`, record any warnings (`lastwarn`, or the command window output) and errors, then
   `update_diagram` it with `set_param(name, 'SimulationCommand', 'update')` (for `furuta_harness`, first run
   `harness_load('ctrl_hybrid.slx', 'push')` so its variables exist; the controller files compile inside
   the harness, so for those a load is enough). Close without saving.
7. **Replay.** `addpath replay; C = compare_to_run('20261115_100519_238300');` Record the printed fit line
   and whether the figure shows measured and simulated traces mostly on top of each other (expected:
   same numbers as step 5).
8. **Closed-loop run.** `R = run_sim('ctrl_hybrid.slx', 'push', 'Plot', true);` Record whether it runs
   without error. Whether the pendulum stays up does **not** matter: the team has not finished the
   controller gains. Then `git status`: expected clean (apart from the results file). If any `.slx` shows as
   modified, record which one, then restore it with `git checkout -- <file>`.
9. **Results.** Write `docs/TEST_RESULTS_HARNESS_R2023.md` with the table below filled in (MATLAB release,
   OS, commit, date at the top), commit only that file on `harness-r2023a`, and push. If you cannot push,
   paste the file contents back to the person.

## Results table (to fill in)

| # | Step | Expected | Result (pass / fail / not run) | Notes, exact error or warning text |
|---|------|----------|-------------------------------|------------------------------------|
| 2 | Products present | all 5 | | |
| 3 | Model files saved in | R2023a (x6) | | |
| 4 | `harness_setup`, `furuta_rhs` found | yes | | |
| 5a | test_linearization_matches_report_eq104 | pass | | |
| 5b | test_simscape_plant_is_the_report_model | pass | | |
| 5c | test_bodies_reproduce_lumped_parameters | pass | | |
| 5d | test_hanging_stays_hanging | pass | | |
| 5e | test_firmware_actuator | pass | | |
| 5f | test_subsystem_controller_slot | pass | | |
| 5g | test_hybrid_template_plumbing | pass | | |
| 5h | test_metrics_on_known_signal | pass | | |
| 5i | test_replay_matches_report_table4 | pass, 89 / 90 / 92 / 88 % | | |
| 6 | All six models load + update, no errors | yes | | |
| 7 | `compare_to_run` fit line + figure | 89 / 90 / 92 / 88 %, traces overlap | | |
| 8 | `run_sim('ctrl_hybrid.slx', 'push')` runs | no error | | |
| 8 | `git status` clean after testing | yes | | |

Anything not run, mark "not run" and say why; do not mark it pass.
