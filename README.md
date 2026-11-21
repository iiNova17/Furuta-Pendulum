# Furuta Pendulum

A rotary inverted pendulum built, modeled and controlled from scratch for the Control II course. A DC gearmotor
turns a horizontal arm, and a free pendulum at the arm tip is swung up from hanging and balanced upright. The
project covers the mechanical design, the electronics and firmware, a desktop lab application, a nonlinear model
identified from measurements, a Simulink/Simscape simulation of the whole system with a hybrid controller, a
digital twin, and the technical report.

**Final results on the hardware** (onboard controller, 20 November 2026):

| | |
|---|---|
| Swing-up from rest to the catch | 1.42–1.43 s |
| Pendulum while balancing | 1.22–1.33° rms, duty cycle never at its limit |
| Motor current while balancing | about 0.3 A rms (driver rating 1.2 A continuous) |
| Recovery from a 13.5° push | back within 3° after 0.70 s |
| Model validation (open loop, separate recordings) | pendulum angle 84–90 % fit, current 88–89 % |

The full derivation, identification and controller design are in [`report/furuta_model.pdf`](report/furuta_model.pdf).

## Repository layout

| Folder | Contents |
|---|---|
| [`report/`](report) | Technical report (`furuta_model.tex`, compiled `furuta_model.pdf`) and the scripts that regenerate every figure and number from the recordings (`figures/make_report_data.py`) |
| [`matlab/model/`](matlab/model) | Identified model: `furuta_rhs.m` (nonlinear equations), `furuta_params_20261120.m` (current parameters), `furuta_params.m` (parameters of report Table 6) |
| [`matlab/simulink_r2026a/`](matlab/simulink_r2026a) | Simulink model of the whole system, saved in MATLAB R2026a |
| [`matlab/simulink_r2023a/`](matlab/simulink_r2023a) | The same Simulink model saved in R2023a (opens in R2023a and newer); all other files are identical |
| [`desktop/`](desktop) | Furuta Lab, the Python/Qt lab application (`furuta_lab/`), its tests, and the rig and analysis scripts (`scripts/`) |
| [`firmware/`](firmware) | ESP32-S3 firmware (PlatformIO): sensors, safety, telemetry and the onboard controller |
| [`twin/`](twin) | Unreal Engine 5.3 digital twin and its MATLAB senders |
| [`hardware/`](hardware) | SolidWorks assembly and parts (`cad/`), files for 3D printing (`print/`) |
| [`data/`](data) | Raw hardware recordings (`runs/`), still captures and the estimation sessions behind both models |
| [`protocol/`](protocol), [`config/`](config) | Serial protocol specification and golden vectors; device profile and schema |
| [`docs/`](docs) | Course project description and the in-app guide of Furuta Lab |

## Hardware

| Part | Component |
|---|---|
| Controller | ESP32-S3-DevKitC-1 (N16R8) |
| Motor | JGA25-370 DC gearmotor, 34:1, 11-pulse quadrature encoder (1496 counts per arm revolution) |
| Driver | TB6612FNG H-bridge, 20 kHz PWM, 12 V supply |
| Pendulum angle | AS5600 12-bit magnetic encoder with a diametric magnet on the pendulum shaft |
| Current | ACS712-05B Hall sensor on the ESP32 ADC |
| Mechanics | 3D-printed base, arm and lid; 608ZZ bearings; a removable pin that locks the arm at center |

## Getting started

### Furuta Lab (lab application)

Requires Windows and Python 3.12 or newer.

```powershell
.\Setup-Furuta-Lab.ps1      # once: creates .venv and installs desktop/requirements.txt
.\Start-Furuta-Lab.cmd      # starts the app
```

Choose the serial port and press **Connect USB**; the motor stays disabled until the 12 V box is ticked and finite
limits are commissioned. The tabs cover live signals and recordings, sensor noise and filters, motor tests,
step-by-step parameter estimation, the onboard controller, and the digital twin. Tests:
`.venv\Scripts\python.exe desktop\test_lab.py`.

### Firmware

Open `firmware/` with PlatformIO and upload, or from a terminal:

```powershell
cd firmware; pio run -t upload --upload-port COM11
```

Flashing reboots the board, so the arm center, the pendulum-down reference and the current zero are set again in
the app. The onboard controller can be checked against Simulink without hardware with the replay test
`firmware/test/native_onboard.cpp` (same duty cycle as the Simulink controller).

### Simulink model

Open MATLAB in the folder that matches your release (`simulink_r2026a` or `simulink_r2023a`) and run:

```matlab
harness_setup                                         % paths: model, library, controllers
open_system('furuta_harness')                         % the whole system: controller + rig
R = run_sim('ctrl_hybrid.slx', 'swingup', 'Plot', true);
```

Required toolboxes: Simulink, Simscape, Simscape Multibody, Stateflow, Control System Toolbox.
Tests: `runtests({'tests/test_harness.m', 'tests/test_observer.m'})`.

## The model

The state is $x = [\theta_0, \theta_1, \dot\theta_0, \dot\theta_1]$ (arm angle, pendulum angle with $\theta_1 = 0$
upright) and the input is the PWM duty cycle. The equations come from a Lagrangian derivation of the arm and
pendulum combined with a DC motor model, with Coulomb, viscous and breakaway friction on the arm and dry and viscous
friction on the pendulum. All torque-like parameters are identified per unit of the motor torque constant, so they
are expressed in amperes of motor current. They were identified from a free swing of the pendulum and square-wave
experiments with the pendulum hanging, followed by a refinement that minimizes the simulation error, and validated
on recordings not used in the fit. After the rig was moved, the model was identified again on 20 November 2026;
`furuta_params_20261120.m` is the current model and `furuta_params.m` the one of the report's Table 6.

## The Simulink model and controller

`furuta_harness.slx` contains the complete closed loop:

- **Controller** (`controllers/hybrid/ctrl_hybrid.slx`): a model-based observer that compensates the 9–15 ms lag of
  the pendulum reading; the stabilizing laws, each in its own file (`stab_lqr.slx`, `stab_pid.slx`,
  `stab_pole_placement.slx`); the energy-based swing-up (`swingup_energy.slx`); and a Stateflow supervisor that
  switches between swing-up, stabilize and safe stop. Every gain and threshold is set in `ctrl_hybrid_design.m`.
- **Rig**: the firmware's safety and output logic (duty limit, slew limit, travel and current trips, PWM
  resolution), the identified plant as a Simscape Multibody model, and the sensors (encoder quantization, pendulum
  lag, current noise and filter).

`run_sim` simulates any scenario (`balance`, `push`, `arm_step`, `swingup`), and `compare_controllers` scores the
stabilizers side by side, for example PID, pole placement and LQR selected with `struct('stab', 1)`, `2` or `3`.
The same controller runs on the ESP32 (firmware 1.12), generated from the same design file by
`controllers/hybrid/export_onboard.m`, so the hardware runs the design that was simulated.

## Digital twin

- **In Furuta Lab** (tab *Digital twin*): a 3D view of the rig posed from the serial stream in real time, with the
  identified model running next to it as a translucent ghost, and playback of recorded runs.
- **Unreal Engine 5.3** (`twin/`): a standalone renderer fed by UDP. `twin/run_twin.ps1` builds and opens it,
  `twin/package_twin.ps1` builds a standalone executable, and `twin/matlab/twin_play.m` streams any simulated or
  recorded motion. Packets (UDP port 7400, 16 bytes): `RIP1`, source byte (0 rig, 1 model), 3 padding bytes,
  `float32 q_arm`, `float32 q_pend` (radians, $q_{pend} = \theta_1 - \pi$, zero hanging).

## Report

```powershell
python report/figures/make_report_data.py   # figures and numbers from data/runs (numpy, scipy, matplotlib)
cd report; latexmk -pdf furuta_model.tex
```

## Safety

The arm must never be driven without confirming that the center pin is removed, the 12 V supply is on and the area
around the arm is clear. The firmware keeps a travel limit, a current trip and a heartbeat from the computer active
in every powered run.

## Branches

| Branch | Contents |
|---|---|
| `main` | The complete project as presented here |
| `onboard-control` | Furuta Lab and the ESP32 firmware only |
| `r2023-model` | The R2023a Simulink model and its dependencies only, for controller development |

Earlier development branches are kept as tags under `archive/`.

## License

MIT, see [LICENSE](LICENSE).
