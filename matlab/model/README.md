# Identified Furuta pendulum model

Two identified parameter sets, same struct, fields and linear models, both simulated by `furuta_rhs.m`:

| File | Model | Use |
|---|---|---|
| `furuta_params_20261120.m` | re-identified on 2026-11-20 after the rig was moved (`data/estimation/suite_20261120`, `desktop/reidentify_variants.py` variant E: the claude_loop chain with the arm terms also refined by simulation, pendulum timing 10 ms) | **current model**: harness default, controller design (`D.design_model`) |
| `furuta_params.m` | 2026-11-15, `data/estimation/claude_loop`, report Table 6 | the report's Table 4 / Table 6 results; `harness_config(..., 'Model', 'table6')` |

On the three validation runs of 2026-11-20 (never fitted) the 20 Nov model fits arm angle 62.6 %, pendulum 78.8 %,
arm speed 90.1 %, current 72.0 %; Table 6 on the same runs 54.7 / 45.3 / 76.0 / 71.4 %. Not in either model: the
shaft-coupling play and flex (the pendulum reading lags the motor encoder by 9-15 ms on the rig).

| File | Contents |
|---|---|
| `furuta_params.m`, `furuta_params_20261120.m` | Scripts that create the parameter struct `p`, the initial states and the linear models (`sys_up`, `sys_down`, ...) |
| `furuta_rhs.m` | Nonlinear model `[dx, i] = furuta_rhs(x, d, p)`, report equation for the nonlinear state equations |

- State `x = [theta0; theta1; dtheta0; dtheta1]` (rad, rad/s). `theta1 = 0` is upright and `theta1 = pi` is hanging.
- Input `d` is the duty cycle in [-1, 1] (armature voltage `V = d*p.Vs`, `p.Vs = 12`).
  At `d = 0` exactly, the driver is in standby: the motor is open-circuit and `i = 0`.
- Torque-like parameters are divided by the torque constant, so torques are in amperes of motor current.
- The arm has stick-slip friction: at rest it holds until the torque exceeds the breakaway `p.tsp` / `p.tsn`
  (Table 6: 0.455 / 0.490 A = the 13 % / 14 % duty dead zone of 2026-11-07; 20 Nov model: 0.2675 A = 6 %), then slips with the identified Coulomb + viscous
  friction. The harness's Simscape plant has it too (`matlab/harness/lib/arm_friction.m`, tested against `furuta_rhs`).

## MATLAB

```matlab
run('furuta_params.m')
duty = @(t) 0.25*sign(sin(2*pi*1.4*t));            % example square wave
[t, x] = ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], x0_hanging);
```

## Simulink (nonlinear plant)

1. Run `furuta_params.m` so that `p` and `x0_hanging` are in the base workspace.
2. Add a **MATLAB Function** block with this code:

   ```matlab
   function [dx, i] = plant(x, d, p)
   [dx, i] = furuta_rhs(x, d, p);
   ```

   In the Symbols pane (Model Explorer), set `p` to scope **Parameter**, so the block reads the struct from the workspace.
   Keep `matlab/model` on the MATLAB path.
3. Feed `dx` into an **Integrator** block (initial condition `x0_hanging`, or `x0_upright` for balancing),
   and feed the integrator output back into `x`. Use a Demux to get theta0, theta1, dtheta0 and dtheta1.
4. Drive `d` with your controller output, saturated to [-1, 1] (the firmware limits it to 0.5).
5. Solver: the validation used fixed-step RK4 (`ode4`) with a 2 ms step. Use 2 ms or smaller, or a variable-step solver.

The model was checked against the Python implementation in `desktop/furuta_lab/estimation.py`
(`rhs`): the derivatives agree to 1e-9, and linearizing it gives `A_up`/`B_up` and `A_down`/`B_down`.
