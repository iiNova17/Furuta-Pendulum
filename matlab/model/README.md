# Identified Furuta pendulum model

The final model from the parameter estimation (report section 6). It is the same model the
Furuta Lab app simulates, so it reproduces the validation fits in the report.

| File | Contents |
|---|---|
| `furuta_params.m` | Script that creates the parameter struct `p`, the initial states and the linear models (`sys_up`, `sys_down`, ...) |
| `furuta_rhs.m` | Nonlinear model `[dx, i] = furuta_rhs(x, d, p)`, report equation for the nonlinear state equations |

- State `x = [theta0; theta1; dtheta0; dtheta1]` (rad, rad/s). `theta1 = 0` is upright and `theta1 = pi` is hanging.
- Input `d` is the duty cycle in [-1, 1] (armature voltage `V = d*p.Vs`, `p.Vs = 12`).
  At `d = 0` exactly, the driver is in standby: the motor is open-circuit and `i = 0`.
- Torque-like parameters are divided by the torque constant, so torques are in amperes of motor current.

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
