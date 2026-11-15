# Furuta Pendulum — Project Brief, Components and Current Wiring

**Updated: 20 October 2026**

This is a standalone record of the project and the latest agreed wiring.
It distinguishes course requirements, user choices, proposed additions, and
unverified assumptions. It does not establish that the circuit has passed
electrical tests.

## 1. What we are building

A **Furuta pendulum**, also called a rotary inverted pendulum:

- A motor rotates a horizontal arm about a vertical axis.
- A free pendulum pivots at the end of that arm.
- The motor encoder measures motor rotation; the gearbox ratio is needed to
  infer arm rotation.
- A separate magnetic pendulum encoder measures the free pendulum angle.
- Control software swings the pendulum upward, captures it near upright,
  and balances it despite disturbances.
- A MATLAB dashboard displays measurements, actuator commands, controller
  mode, and a synchronized animated representation of the apparatus.

There is one actuator and two mechanical degrees of freedom. The upright
pendulum is unstable without feedback.

Suggested consistent state convention:

```text
x = [theta0, theta1, omega0, omega1]
theta0 = arm angle
theta1 = pendulum angle, zero at upright
omega0, omega1 = corresponding angular velocities
Angles in radians; angular velocities in rad/s
```

Sensor zero offsets, rotation signs and angle unwrapping still need calibration.

## 2. Course requirements

The controlling source is the seven-page Fall 2026 Ain Shams University brief,
“Furuta Pendulum — Modelling, Hybrid & Control, and Real-Time Dashboard,” for
MCT411 / MCT413s / MCT414. The earlier conversation handoff is background, not
an approved design or bill of materials.

Required work includes:

1. Derive nonlinear equations of motion and identify the actual hardware's
   geometry, masses, inertias, friction and motor/gearbox parameters.
2. Linearize about upright, analyze equilibrium stability, and check
   controllability and observability.
3. Design and simulate **PID, pole-placement and LQR** stabilization controllers.
4. Implement **energy-based or equivalent swing-up**. The grading table
   specifically emphasizes energy-based swing-up on real hardware.
5. Implement **MATLAB Stateflow** supervisory logic with swing-up, stabilization,
   capture guards using angle and angular velocity, and safety/fallback behavior.
6. Run the controller, including Stateflow, against real hardware in real time,
   as required by the brief's HIL wording.
7. Build a MATLAB dashboard showing both angles, actuator command and active
   mode, with a synchronized animated 2D/3D twin embedded in the dashboard.
8. Compare controllers using transient response, steady-state error, error
   integrals, disturbance recovery, control effort and saturation. Repeat key
   robustness tests at least twice and compare simulation with experiments.
9. Provide a technical report, individual contribution records, oral defense,
   and a demonstration video of at most five minutes.

Simscape/SimMechanics is preferred in the brief, not an absolute requirement.
The Figure 2 design process is: physical system → control requirements →
modelling → apply control theory → verify results → test the control loop.

| Milestone | Required outcome |
| --- | --- |
| Week 6 | MATLAB plant model and open-loop validation |
| Week 9 | Stability analysis, three stabilization controllers and simulations |
| Week 13 | Physical prototype, real-time control, Stateflow, dashboard/twin and report |

Grading: hardware/dashboard 20%; digital twin 10%; report 20%; oral defense
15%; simulation/controllers/Stateflow 20%; video 15%.

Unresolved course-compliance questions:

- The user planned three members; the brief requires 6–8 in one section.
- The user wants custom ESP32 firmware, not MATLAB-generated deployment.
  The final architecture must still satisfy the executing MATLAB/Simulink/
  Stateflow requirement; a dashboard watching a firmware-only controller
  does not by itself demonstrate that.
- The brief calls for a separate pendulum encoder of at least 360 pulses/rev.
  A 12-bit absolute digital encoder has 4096 positions/rev, but acceptance
  against that literal pulse-output wording remains to be confirmed.

## 3. Components currently reported by the user

Use functional names such as “motor driver” and “current sensor” in discussion.
Exact models are recorded here for electrical and software compatibility.

| Component | Reported item | Known details / remaining uncertainty |
| --- | --- | --- |
| Controller board | ESP32-S3 N16R8, board marking YD-ESP32-23 2022-V1.3 | Expected 16 MB flash and 8 MB PSRAM; has COM and USB connectors. Exact board circuitry has not been independently verified. |
| Motor and gearbox | Brushed DC geared motor with quadrature encoder | Motor constants, gear ratio, rated voltage, speed, torque, backlash and inertia remain unknown. User reports approximately 2.2 A stall current and successful operation from the 12 V adapter. |
| Motor encoder | Integrated A/B encoder, 11 PPR per channel | Supplied pinout marks encoder power as 3.3 V / 5 V. Nominal x4 decoding gives 44 counts/motor revolution if PPR denotes channel cycles. |
| Motor driver | TB6612FNG dual-channel breakout from MicroOhm Egypt | Use channel A. Typical stated average limit 1.2 A/channel; 3.2 A is a short-pulse rating. Recommended VM operating range 2.5–13.5 V; 15 V is an absolute maximum. |
| Current sensor | ACS712, 5 A variant | 5 V supply; nominal zero output 2.5 V and sensitivity 185 mV/A. Bidirectional measurement, not an automatic current limiter. |
| Pendulum encoder | AS5600 magnetic absolute encoder | 12-bit single-turn angle over I2C; requires suitable magnet/alignment. DIR is exposed and is assigned to GND. |
| Power adapter | 12 V, 2 A | Supplies the motor-power rail only. Adapter rating is not a guaranteed protective current limit for the driver. |
| DC barrel jack | User-provided | Verify polarity and switched-contact identity with a meter. |
| Switch | User-provided | Must be suitable for the DC motor-supply current; exact contact arrangement/rating not established. |

Driver listing supplied by the user:
https://microohm-eg.com/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/

The listing screenshot showed a standard two-channel breakout, but readable
header labels and a complete board schematic were not available. Use actual
printed labels, including on the underside, rather than inferred pin positions.

Earlier NEMA17/stepper-driver and two-magnetic-encoder proposals are historical;
they are **not the current component set**.

### Supporting parts specified for the circuit

These were recommended; ownership/installation is not confirmed:

- 10 kohm resistor from motor-driver STBY to GND.
- 100–470 uF electrolytic capacitor rated at least 25 V, across VM and GND.
- 100 nF ceramic capacitor across VM and GND near the motor driver.
- Confirm motor-driver VCC bypassing; add 100 nF VCC-to-GND if absent.
- 4.7 kohm I2C pull-ups to 3.3 V if not already present on the pendulum module.
- 4.7–10 kohm motor encoder pull-ups to 3.3 V if required by its output circuit.
- Wires/terminals suitable for motor current.

**The current-sensor output divider is explicitly omitted by user choice.**
Do not confuse it with the separate STBY resistor, which remains in the wiring.

## 4. Latest power wiring

The ESP32 and sensors are powered from **USB**. The motor receives power from
the **12 V adapter through the motor driver**. Their grounds are connected.

| From | To |
| --- | --- |
| Barrel jack positive | Switch input |
| Switch output | Motor-driver VM |
| Barrel jack negative | Common ground junction |
| Motor-driver GND pins | Common ground junction |
| ESP32 GND | Common ground junction |
| ESP32 3V3 | Motor-driver VCC |
| Computer USB | Board COM connector for the current test firmware |
| Electrolytic capacitor positive | Motor-driver VM |
| Electrolytic capacitor negative | Motor-driver GND |
| 100 nF ceramic capacitor | Across motor-driver VM and GND |

If the switch has more than two terminals, identify its actual contacts before
wiring. A barrel jack's third terminal may be a switched contact.

**12 V must not connect to motor-driver VCC, ESP32 5V or ESP32 3V3.**
The driver power-ground return should run directly to the common junction,
not through ESP32 ground traces or thin sensor wiring. Do not carry motor
current through breadboard power rails.

## 5. Latest motor-driver control wiring

All numbers below are **GPIO numbers**, not header pin positions.

| Motor-driver label | Connection |
| --- | --- |
| PWMA | ESP32 GPIO15 |
| AIN1 | ESP32 GPIO16 |
| AIN2 | ESP32 GPIO17 |
| STBY | ESP32 GPIO18 |
| STBY, additionally | External 10 kohm resistor to GND |
| PWMB | GND |
| BIN1 | GND |
| BIN2 | GND |
| BO1 / B01 | Unconnected |
| BO2 / B02 | Unconnected |

```text
ESP32 GPIO18 ───────── Motor-driver STBY
                            │
                           10 kohm
                            │
                           GND
```

The user measured about 170 kohm between STBY and GND. This does not establish
the internal pull-down arrangement; semiconductor circuitry affects resistance
measurements. Keep the external resistor. Resistance measurements are made
with all supplies disconnected.

STBY LOW disables the driver. Keep it LOW during boot and the current USB-only
test. Do not tie STBY directly to VCC.

## 6. Motor and current-sensor current path

Motor connector numbering follows the user's original supplied pinout:

| Motor pin | Function |
| --- | --- |
| 1 | Motor minus |
| 2 | Encoder ground |
| 3 | Encoder A phase |
| 4 | Encoder B phase |
| 5 | Encoder supply, marked 3.3 V / 5 V |
| 6 | Motor plus |

Wire the current sensor in series with one motor lead:

```text
Motor-driver AO1 ── Current-sensor IP+
                   Current-sensor IP− ── Motor pin 6 (Motor +)

Motor-driver AO2 ──────────────────────── Motor pin 1 (Motor −)
```

AO1/AO2 may be printed A01/A02. Neither motor lead connects to ground.
The motor's plus/minus labels establish a reference direction; the bridge
reverses their voltage during bidirectional operation.

The current sensor's high-current terminals are distinct from its VCC/GND/OUT
signal header. Motor-lead placement measures bidirectional winding current,
including behavior that a sensor only in the adapter lead would not capture
in the same way.

## 7. Current-sensor signal wiring — latest user-selected configuration

| Current-sensor pin | Connection |
| --- | --- |
| VCC | Verified USB-powered 5 V rail on the ESP32 board |
| GND | Common GND |
| OUT | **ESP32 GPIO4 directly; no voltage divider** |

Verify the board's 5V pin actually supplies approximately 5 V when USB-powered.
Do not assume every VIN-labeled pin is a usable USB-derived 5 V output.

GPIO4 is ADC1 channel 3. Use the highest available ADC attenuation and calibrate
the zero-current voltage. Nominal conversion with this direct connection:

```text
Current [A] = (measured voltage [V] - zero-current voltage [V]) / 0.185
```

Zero is nominally 2.5 V, but must be measured. At nominal sensor specifications,
+2.2 A gives about 2.907 V, -2.2 A gives about 2.093 V, and +5 A gives about
3.425 V. Supply/offset variation and transients affect actual values.

**Electrical limitation:** this direct connection has no ADC overvoltage
protection. The 5 V-powered sensor can output beyond the ESP32 ADC's usable
range and pin limits under larger current/fault conditions. An absolute-maximum
rating is not a recommended operating target. Software calibration or
attenuation cannot protect an overdriven pin. The earlier divider recommendation
was not withdrawn; omission is the user's explicit choice.

Powering this sensor at 3.3 V is outside its specified 4.5–5.5 V supply range
and does not provide a reliable substitute for output scaling. The current
sensor does not independently shut off excess motor current.

## 8. Motor encoder wiring

| Motor encoder connection | ESP32 connection |
| --- | --- |
| Motor pin 2, encoder ground | GND |
| Motor pin 3, encoder A | GPIO5 |
| Motor pin 4, encoder B | GPIO6 |
| Motor pin 5, encoder supply | 3V3 |

Hardware PCNT performs x4 quadrature counting. If open-collector outputs lack
onboard pull-ups, use external 4.7–10 kohm pull-ups to **3.3 V**, not 5 V.
The test firmware enables weak internal pull-ups, which may not suffice for
long/noisy wiring.

For a gear ratio G = motor revolutions / output revolution, ideal output-shaft
counts per revolution would be 44G under the stated PPR interpretation.
Gearbox backlash means motor-shaft sensing is not an exact measurement of arm
position during reversals. No gear ratio or arm conversion is yet confirmed.

## 9. Pendulum encoder wiring

This is the intended full-circuit wiring. The encoder is **included in the
fresh USB-only test** requested on 20 October 2026.

| Pendulum encoder label | Connection |
| --- | --- |
| Documented 3.3 V-compatible supply input | ESP32 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |
| DIR | GND |
| OUT | Unconnected |
| PGO, if exposed | Unconnected |

If the module has separate 5V and 3V3 supply terminals, follow its documented
3.3 V connection. Verify that I2C pull-ups go to 3.3 V; add 4.7 kohm pull-ups
only if needed. The normal I2C address is 0x36.

DIR fixes the sensor's rotation convention; it does not control motor direction.
Confirm physical sign during calibration rather than relying on an ambiguous
viewing direction. DIR is assigned to GND instead of left floating.

Mount the sensor/magnet coaxially at the free pendulum pivot. Check magnet type,
alignment, gap and field diagnostics. Angular velocity must be estimated from
angle with wrap handling and appropriate filtering or observation.

## 10. Complete GPIO allocation

| GPIO | Function |
| --- | --- |
| 4 | Current-sensor analog OUT, direct ADC1 input |
| 5 | Motor encoder A |
| 6 | Motor encoder B |
| 8 | Pendulum encoder SDA |
| 9 | Pendulum encoder SCL |
| 15 | Motor-driver PWMA |
| 16 | Motor-driver AIN1 |
| 17 | Motor-driver AIN2 |
| 18 | Motor-driver STBY |

This allocation avoids boot-strapping pins and the usual N16R8 memory pins.
GPIO19/20 are reserved for native USB and GPIO43/44 for UART0/COM communication.
Confirm actual board header labels and onboard uses.

## 11. Firmware and current test stage

The user requested a fresh baseline on 20 October 2026. The old PlatformIO
project, its hardware-test logs, and its handoff folder/archive were removed
from the workspace into the Windows Recycle Bin. Earlier test results are
not carried forward. Compiler packages were retained as development tools.

The active project is now **`Furuta-USB-Test/`**:

- `platformio.ini` and `src/main.cpp`
- `include/bench_math.h`
- `README.md` for wiring, commands and expected observations
- `CHECKLIST.md` for fresh measurements; all hardware checks start pending
- `bench.ps1` for build/upload/monitor/port listing

Configuration: PlatformIO espressif32 6.12.0; Arduino framework; generic
esp32-s3-devkitc-1 with 16 MB flash, qio_opi memory configuration and PSRAM
enabled. USB CDC on boot is disabled: connect through **COM**, at 115200 baud.

The USB-only firmware holds STBY LOW and has no enable command. It reads
the motor encoder using PCNT, the pendulum encoder over I2C (including magnet
diagnostics), and current-sensor voltage with optional zero calibration.
No balance/current control is implemented. The driver-input test uses LEDC.

| Serial command | Action |
| --- | --- |
| h / b | Help / board and memory information |
| i | I2C scan |
| a | Set current pendulum position as relative zero, RAM only |
| z | Collect fresh one-second zero-current calibration |
| e | Zero motor encoder count |
| t | Twelve-second driver-input signal test, STBY LOW throughout |
| s or x | Stop signal test; outputs LOW; cancel active calibration |
| p | Pause/resume reports while sensing continues |

The `t` sequence holds AIN1 HIGH for 4 seconds, AIN2 HIGH for 4 seconds,
then emits 1 kHz / approximately 50% PWM for 4 seconds with both direction
pins LOW. Outputs then return LOW. Physical verification needs a meter/scope;
serial messages alone do not prove the pins or H-bridge work.

### Immediate test conditions and status

Finish wiring with USB disconnected. Include both encoders and the current
sensor. Keep the 12 V adapter physically unplugged, retain the STBY pull-down,
then power the board through COM USB after wiring is complete.

The fresh firmware built successfully on 20 October 2026. Its angle-wrap and
calibration-statistics software checks passed, and its ESP32-S3 image checksum
and 16 MB flash header were verified. The fresh firmware was subsequently
uploaded on COM11 with verified flash hashes. Runtime confirmed 16 MiB flash
and 8 MiB PSRAM. Initial stationary checks found successful pendulum I2C
communication but weak magnetic field (MD=1, ML=1, MH=0), and current calibration
was rejected due to ADC-path noise (63 mV sample SD around 2.5065 V).
Motion, physical supply/STBY measurements and driver-input measurements
remain pending. See `Furuta-USB-Test/CHECKLIST.md` and its fresh session logs.

Check actual 3.3 V and 5 V supplies at the modules, current-sensor voltage near
2.5 V, pendulum magnet/angle response, and motor counts if gentle shaft movement
is possible. Do not force the gearbox. Driver control signals require physical
measurement.

Without 12 V, motor operation, torque, bridge output performance and loaded
current accuracy cannot be validated. A later, separate 12 V test remains
pending; USB-only results do not authorize assuming power-stage suitability.

## 12. Control architecture and remaining engineering work

The user wants custom ESP32 firmware communicating with MATLAB/Simulink
over USB serial or possibly Wi-Fi; the PC/MCU controller split is not finalized.
Host-loop latency and jitter need measurement if the PC closes the balance loop.
Serial packet format, control sample rate, watchdog and Stateflow execution
location remain to be selected.

For the present DC motor:

- PWM is a voltage command, not a direct torque measurement/command.
- Torque depends on winding current; voltage also interacts with winding
  resistance/inductance and speed-dependent back EMF.
- A current sensor alone does not implement a current-control loop.
- A voltage-input plant can support swing-up, pole placement and LQR without
  an inner current loop, provided the actuator is modelled consistently.
- A current loop, if later implemented, requires appropriate sampling,
  feedback regulation, limits and validation.
- Gear ratio, inertia, backlash and friction must be included consistently.
- Controller gains for torque input must not be reused unchanged as voltage
  or acceleration gains.

Before powered testing, resolve motor/driver current compatibility. A lightweight
mechanism does not remove starting, reversing or stall-current demands. The
reported 2.2 A stall exceeds the driver's stated average-current rating, and a
successful adapter test does not verify driver suitability.

Before balancing, establish mechanics and parameters, reliable encoder signs/
zeros, real actuator behavior, sampling latency, safe disable/watchdog behavior,
and a course-compliant control architecture. Those steps are pending rather
than implied by the current wiring.
