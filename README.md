# Furuta Pendulum: R2023a Simulink model (controller development)

This branch holds only the Simulink model of the whole system, saved in MATLAB R2023a (opens in R2023a and
newer), and the files it needs. It is where new controllers (PID, pole placement) are designed before they are
brought into `main`. The app, firmware, twin, CAD, data and report live on `main`; the app and firmware alone on
`onboard-control`.

## Contents

| Path | What |
|---|---|
| `matlab/model/` | The identified plant: `furuta_rhs.m` (nonlinear equations), `furuta_params_20261120.m` (current model, the default), `furuta_params.m` (report Table 6) |
| `matlab/simulink_r2023a/furuta_harness.slx` | The whole closed loop: Controller slot + Rig (firmware logic, Simscape plant, sensors) |
| `matlab/simulink_r2023a/controllers/hybrid/` | The hybrid controller `ctrl_hybrid.slx` and its referenced parts: `est_observer.slx`, `stab_pid.slx`, `stab_pole_placement.slx`, `stab_lqr.slx`, `swingup_energy.slx`; every knob in `ctrl_hybrid_design.m`; LQR and swing-up design tools |
| `matlab/simulink_r2023a/lib/` | Rig blocks (firmware actuator, sensors, friction), metrics, corner plants, `OpenLoopDuty` |
| `matlab/simulink_r2023a/tests/` | `test_harness.m` (plant = model equations, rig, plumbing, replay of a recorded run), `test_observer.m` |
| `matlab/simulink_r2023a/replay/` | Recorded hardware runs as `.mat` and `compare_to_run.m` (simulation vs measurement) |

## Use

```matlab
cd matlab/simulink_r2023a
harness_setup
open_system('furuta_harness')                                   % whole system, hybrid controller in the slot
R = run_sim('ctrl_hybrid.slx', 'push', 'Params', struct('stab', 1), 'Plot', true);   % 1 PID, 2 PP, 3 LQR
runtests({'tests/test_harness.m', 'tests/test_observer.m'})     % after any change to the plant or rig
```

Scenarios: `balance`, `push`, `arm_step`, `swingup`. Comparison (brief 4.7 metrics):

```matlab
c = repmat({'ctrl_hybrid.slx'}, 1, 3);
T = compare_controllers(c, {'push'}, 'Params', {struct('stab', 1), struct('stab', 2), struct('stab', 3)}, ...
    'Labels', {'PID', 'PP', 'LQR'})
```

## Where the PID and pole-placement work goes

- `controllers/hybrid/stab_pid.slx`: PID on theta1 plus a PD on the arm (`u = PID(theta1) + Kp0 (theta0 - r) + Kd0 dtheta0`),
  integrator reset when the supervisor enters Stabilize. Inputs `xh` (observer state, theta1 = 0 upright), `r`, `active`; output `u`.
- `controllers/hybrid/stab_pole_placement.slx`: `u = -K (xh - [r 0 0 0])`, `K = place(Ad, Bd, exp(poles Ts))`.
- `controllers/hybrid/ctrl_hybrid_design.m`: gains in `D.pid`, poles in `D.pp.poles`, the active stabilizer in `D.stab`.
- Save every `.slx` in R2023a format. The design model is the 20 Nov model (`D.design_model`), Ts = 4 ms.

## Bringing a design into main

On `main`, copy this branch's model folder over the R2023a copy (files only present on `main`, such as
`export_onboard.m`, are kept):

```powershell
git switch main
git checkout r2023-model -- matlab/simulink_r2023a matlab/model
```

`main` also keeps the same model saved in R2026a (`matlab/simulink_r2026a`). R2026a opens R2023a files, so to
keep the two identical, open the changed `.slx` files in R2026a and save them into `simulink_r2026a`, and copy the
changed `.m` files there. For the ESP32, run `export_onboard.m` on `main` and port the new law to
`firmware/include/onboard.h` (on `onboard-control` as well).
