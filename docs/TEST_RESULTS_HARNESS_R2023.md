# Test results: simulation harness on MATLAB R2023b

Plan: `docs/TEST_PLAN_HARNESS_R2023.md`. Simulation only (no hardware, serial port, app or motor).

- MATLAB: 23.2.0.2515942 (R2023b) Update 7
- OS: Windows 11 Home 10.0 (build 26200)
- Commit tested: `3e5bf5a` (branch `harness-r2023a`), "Add test plan for running the harness on MATLAB R2023a/b"
- Date: 2026-11-18

## Summary

Everything works in R2023b except one problem, which is reproducible and has one clear cause:
**the first simulation of `furuta_harness` that uses the `OpenLoopDuty` controller after the model is
loaded fails**. The error comes from all 13 Simscape Multibody blocks:

```
File Solid block (mask) does not have a parameter named 'SimscapeInstrumentationLogging'
File Solid block (mask) parameter 'SimscapeInstrumentationLogging' is read-only
```

The same pair of messages appears for Inertia, Brick Solid, Mechanism Configuration, Revolute Joint (x2),
World Frame and Rigid Transform (x4) blocks. The top-level identifier is `MATLAB:MException:MultipleErrors`,
raised from `run_sim` (line 46, `sim(in)`) via `Simulink.Simulation.internal.DesktopSimHelper`.

A second identical call in the same MATLAB session runs, and gives the expected results (diagnostic re-runs,
no files changed). This affected test 5b and step 7. The `ctrl_hybrid` closed-loop run (step 8) worked
on its first call from a fresh load.

Likely cause: `furuta_harness.slx` still stores `<P Name="SimscapeInstrumentationLogging">off</P>` on
13 Multibody blocks (an R2026a block parameter that R2023b's Simscape Multibody masks do not have;
`Simulink.exportToVersion` kept it). Found by reading the `.slx` XML; not changed.

Other output seen on every load of the exported models (not errors):

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
| 5a | test_linearization_matches_report_eq104 | pass | pass | 4.9 s |
| 5b | test_simscape_plant_is_the_report_model | pass | **fail** | Errored (it is the first `OpenLoopDuty` sim after load). `MATLAB:MException:MultipleErrors` from `run_sim` line 46: `<block> (mask) does not have a parameter named 'SimscapeInstrumentationLogging'` / `... parameter 'SimscapeInstrumentationLogging' is read-only` for all 13 Multibody blocks (see Summary). Diagnostic: running only this test twice in one session gives run 1 errored, run 2 **passed** (8 s). |
| 5c | test_bodies_reproduce_lumped_parameters | pass | pass | 0.2 s |
| 5d | test_hanging_stays_hanging | pass | pass | 15 s |
| 5e | test_firmware_actuator | pass | pass | 0.2 s |
| 5f | test_subsystem_controller_slot | pass | pass | 19 s |
| 5g | test_hybrid_template_plumbing | pass | pass | 122 s; created and removed `controllers/ctrl_harness_selftest/` as described |
| 5h | test_metrics_on_known_signal | pass | pass | 0.1 s |
| 5i | test_replay_matches_report_table4 | pass, 89 / 90 / 92 / 88 % | pass | `20261115_100519_238300 (validation): fit arm 89 %, pendulum 90 %, arm speed 92 %, current 88 %  \|  RMS arm 1.5 deg, pendulum 1.4 deg, current 0.043 A`. Whole suite: 8 passed, 1 failed, 268 s |
| 6 | All six models load + update, no errors | yes | pass | All six load. `furuta_harness` update (after `harness_load('ctrl_hybrid.slx', 'push')`) OK. Only the export notifications and the `DuplicatePropName` warning above |
| 7 | `compare_to_run` fit line + figure | 89 / 90 / 92 / 88 %, traces overlap | **fail** on first call, pass on retry | First call after a fresh load: same `SimscapeInstrumentationLogging` error as 5b (13 causes). Second call in the same session: `fit arm 89 %, pendulum 90 %, arm speed 92 %, current 88 %` (RMS 1.5 deg / 1.4 deg / 0.043 A); measured and simulated traces overlap in all three panels |
| 8 | `run_sim('ctrl_hybrid.slx', 'push')` runs | no error | pass | Ran on the first attempt from a fresh load, with `'Plot', true`. Balance quality not assessed (gains unfinished) |
| 8 | `git status` clean after testing | yes | pass | Clean; no `.slx` modified, no leftover files |

## Notes for whoever fixes it

- Workaround for now (no file change): in a fresh MATLAB session, run any `OpenLoopDuty` simulation once
  and ignore the error, or simply re-run the command; later calls work.
- A permanent fix probably means clearing `SimscapeInstrumentationLogging` from the 13 Multibody blocks
  before export (or post-processing the exported `.slx`), then re-exporting. Not attempted here (the plan
  forbids changing repo files).
