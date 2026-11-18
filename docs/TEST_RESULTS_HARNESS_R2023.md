# Test results: simulation harness on MATLAB R2023b

Plan: `docs/TEST_PLAN_HARNESS_R2023.md`. Simulation only (no hardware, serial port, app or motor).

- MATLAB: 23.2.0.2515942 (R2023b) Update 7
- OS: Windows 11 Home 10.0 (build 26200)
- Commit tested: `1a245ac` (branch `harness-r2023a`), "Merge sim-harness into harness-r2023a: tidy layout, export fix"
- Date: 2026-11-18 (second run, after the export fix)

## Summary

**Everything passes in R2023b.** All 9 tests pass in one run from a fresh session, all six models load and
update, the replay gives the report's fits (89 / 90 / 92 / 88 %) on its first call, and the closed-loop run
works. The first-run failure from the previous test (see History) is gone: none of the six `.slx` files
contains `SimscapeInstrumentationLogging` any more (checked in the files' XML), and the first `OpenLoopDuty`
simulation after a fresh load now runs.

Output seen on every load of the exported models (not errors, harmless):

- `Model '<name>' was exported from R2026a to R2023a. ... Save the model to remove this notification.`
  (all six models)
- `Warning: furuta_harness (configSet0.xml, line 767): Property 'SimMechanicsInvalidVisualProperty' is already
  registered for parent of configuration component ''.` (identifier `configset:diagnostics:DuplicatePropName`)

## Results table

| # | Step | Expected | Result (pass / fail / not run) | Notes, exact error or warning text |
|---|------|----------|-------------------------------|------------------------------------|
| 2 | Products present | all 5 | pass | MATLAB, Simulink, Simscape, Simscape Multibody, Control System Toolbox: all R2023b (23.2) |
| 3 | Model files saved in | R2023a (x6) | pass | `Simulink.MDLInfo(...).ReleaseName` = R2023a for `furuta_harness`, `ctrl_hybrid`, `stab_lqr`, `stab_pid`, `stab_pole_placement`, `swingup_energy` |
| 4 | `harness_setup`, `furuta_rhs` found | yes | pass | `which('furuta_rhs')` = `<clone>\matlab\model\furuta_rhs.m` |
| 5a | test_linearization_matches_report_eq104 | pass | pass | 2.6 s |
| 5b | test_simscape_plant_is_the_report_model | pass | pass | 23.6 s; first test to load `furuta_harness` in a fresh session |
| 5c | test_bodies_reproduce_lumped_parameters | pass | pass | 0.1 s |
| 5d | test_hanging_stays_hanging | pass | pass | 3.1 s |
| 5e | test_firmware_actuator | pass | pass | 0.1 s |
| 5f | test_subsystem_controller_slot | pass | pass | 7.8 s |
| 5g | test_hybrid_template_plumbing | pass | pass | 32.9 s; created and removed `controllers/ctrl_harness_selftest/` as described |
| 5h | test_metrics_on_known_signal | pass | pass | 0.05 s |
| 5i | test_replay_matches_report_table4 | pass, 89 / 90 / 92 / 88 % | pass | `20261115_100519_238300 (validation): fit arm 89 %, pendulum 90 %, arm speed 92 %, current 88 %  \|  RMS arm 1.5 deg, pendulum 1.4 deg, current 0.043 A`. Whole suite: 9 passed, 80 s |
| 6 | All six models load + update, no errors | yes | pass | All six load. `furuta_harness` update (after `harness_load('ctrl_hybrid.slx', 'push')`) OK. Only the export notifications and the `DuplicatePropName` warning above |
| 7 | `compare_to_run` fit line + figure | 89 / 90 / 92 / 88 %, traces overlap | pass | First call after a fresh load: `fit arm 89 %, pendulum 90 %, arm speed 92 %, current 88 %` (RMS 1.5 deg / 1.4 deg / 0.043 A); measured and simulated traces overlap in all three panels |
| 8 | `run_sim('ctrl_hybrid.slx', 'push')` runs | no error | pass | Ran from a fresh load with `'Plot', true`; 6 s scenario, no fault. Balance quality not assessed (gains unfinished) |
| 8 | `git status` clean after testing | yes | pass | Clean; no `.slx` modified. Only ignored build caches (`*.slxc`, `slprj/`) were created |

## History

- **First run, commit `3e5bf5a` (2026-11-18):** 8 of 9 tests passed. The first `OpenLoopDuty` simulation after
  loading `furuta_harness` failed (test 5b and the first `compare_to_run` call) with
  `<block> (mask) does not have a parameter named 'SimscapeInstrumentationLogging'` on all 13 Simscape
  Multibody blocks; a second call in the same session passed. Cause: the R2026a-only block parameter was
  kept by `Simulink.exportToVersion`. Fixed in `2f1a115` (`export_release` strips it), re-exported in `1a245ac`.
