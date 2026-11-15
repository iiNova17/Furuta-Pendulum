# CONTROL II / FURUTA PENDULUM — ASTRA HANDOFF

**Purpose:** Give Astra sufficient context to continue an ongoing, technically detailed engineering discussion without repeating earlier decisions. This handoff summarizes the conversation and the actual seven-page Fall 2026 course brief. **This is not a completed design or an approved BOM.**

**Project:** MCT411 / MCT413s (also listed as MCT414) — *Furuta Pendulum (Rotary Inverted Pendulum): Modelling, Hybrid & Control, and Real-Time Dashboard*, Faculty of Engineering, Ain Shams University, Fall 2026.

**Original attached brief:** `Project Description F26_3faaec4d6f9dd5653c19bb021509ddef.pdf` (7 pages). If the file is available to Astra, use it as the controlling source for assessment requirements. A local copy accompanied the original conversation; it might need to be uploaded again in Astra.

**Team:** User intends to work with **3 members**. **Conflict:** Original brief §3 explicitly requires **6–8 students in one section**, submitted via team formation sheet (name, ID, section, designated leader). Three students is the user's plan, *not* the official course requirement; instructor approval/exemption needs confirmation.

**Last user question already addressed:** “If I don't use current control how will I do the energy swing up and how would I control the system through torque? Wouldn't a stepper be better against a DC motor with no current control?” Previous response: a **voltage-controlled brushed DC motor can be used to implement swing-up and state-feedback control without a current loop** if the controller and plant are designed for voltage/PWM input rather than pretending PWM equals torque; a **STEP/DIR stepper can implement an acceleration-command approach** if tracking is adequate. User has **not made a final motor/driver selection**.

---

## 1. Course requirements — from the official brief (must distinguish these from our recommendations)

### 1.1 Plant and theory (§4.1)
1. Derive nonlinear Furuta pendulum equations of motion, e.g. Euler–Lagrange; state assumptions on rigid links, friction, motor dynamics.
2. Identify/estimate **hardware-specific** parameters, including arm length `L0`, pendulum length `l1`, masses, moments of inertia `J0/J1`, motor and gearbox constants, friction coefficients; document identification methodology.
3. Linearize about **unstable upright equilibrium** and obtain state-space model.
4. Build a simulation plant, **preferably** in MATLAB Simscape/SimMechanics in addition to the analytical state-space model, and validate their consistency. “Preferably” is not an absolute Simscape mandate.

### 1.2 Stability analysis (§4.2)
1. Analyze upright and hanging equilibrium local stability via eigenvalues of open-loop A matrix.
2. Check controllability and observability before designing controllers.
3. Discuss closed-loop stability of each controller (pole locations; gain/phase margins where applicable for PID).

### 1.3 Controllers (§4.3)
**Minimum 3 stabilization controllers plus 1 swing-up controller:**
1. PID to stabilize the pendulum at `theta1 = 0` (upright).
2. Full-state-feedback pole placement, with selected poles and calculated K.
3. LQR with justified Q/R and calculated K.
4. Energy-based **or equivalent** swing-up from hanging to stabilizer capture region.
5. For every controller show design method, final parameters/gains, and **simulation results before physical deployment**.

### 1.4 Hybrid Stateflow (§4.4)
1. MATLAB **Stateflow** supervisory chart with at least Swing-Up and Stabilization states.
2. State transitions based on pendulum angle `theta1` and angular velocity `dtheta1/dt` (capture region).
3. Appropriate continuous/discrete control law active in each state.
4. Safety transitions if the pendulum falls or leaves safety region (return to swing-up or safe stop).

### 1.5 Digital twin and dashboard (§4.5–4.6)
1. Animated **3D or 2D schematic** digital twin reflecting measured arm and pendulum angles; examples include Simscape Multibody/Simulink 3D Animation or equivalent.
2. During real-time hardware operation, twin synchronized with physical apparatus using the same sensor signals and/or control action.
3. MATLAB dashboard/GUI must display `theta1`, `theta0`, and actuator signal `u` live; additional relevant signals (angular velocities/current/etc.) as applicable.
4. Animated twin **embedded in the same dashboard** as the telemetry; active hybrid state visible.

### 1.6 Benchmarking (§4.7)
For PID vs pole placement vs LQR, present response plots and a quantitative comparison including:
- Settling time, rise time, percent overshoot, peak time for `theta1` after disturbance or capture.
- Steady-state errors for `theta1` and `theta0` as applicable.
- Error integrals: IAE, ISE, **and/or** ITAE on a standardized disturbance test.
- Repeatable manual/impulse disturbance, maximum angular deviation and recovery time.
- Peak/RMS actuator `u`, actuator saturation.
- Repeat important robustness tests **at least twice**, discuss parameter/model uncertainty.
- Compare simulated and experimental results.

### 1.7 Hardware and software rules (§5)
1. **One motor equipped with encoder** for horizontal arm actuation/measurement (`theta0`).
2. **Independent pendulum encoder at least 360 pulses/revolution** for `theta1`. A 12-bit absolute AS5600 gives 4096 angular positions/revolution, but its I²C digital output is not literally quadrature pulses: confirm instructor accepts it as satisfying the wording.
3. Neat wiring; rigid mechanical/electrical mounts.
4. Embedded system “may use any ... platform that can be configured as a MATLAB/Simulink hardware target,” with listed examples.
5. **Explicitly: “The model must be built and controlled from MATLAB/Simulink (Stateflow charts for the hybrid supervisory logic...)”.**
6. **Genuine real-time hardware-in-the-loop**: controller **including Stateflow chart** operates against real hardware, not just an offline simulation.
7. Accessible emergency stop/disable, mechanical end stops or soft limits, adequate clearance.

**Critical unresolved compliance question:** The user intends **NOT** to deploy ESP32 firmware via MATLAB/Simulink. Instead, custom ESP32 C++ firmware and a communication protocol over UART/USB serial or Wi-Fi with MATLAB scripts/Simulink. A potential compliant architecture is running **actual controllers and Stateflow on the PC**, communicating live with ESP32 as a low-level sensor/actuator interface. Whether this is acceptable as “HIL” in the instructor's interpretation (and if custom C++ Stateflow-equivalent on ESP32 is acceptable) **must be confirmed with instructor**. Do not conflate a MATLAB dashboard talking to standalone ESP32 control with a deployed/executing MATLAB Stateflow controller. Real-time transport latency/jitter is a significant engineering constraint.

### 1.8 Technical documentation/general guidelines (§6–7)
- Follow the standard control-design process shown in **Figure 2 on p. 5** for each controller and reference it in report; figure should be reviewed from original PDF.
- Use consistent symbols and sign conventions: `theta0`, `theta1`, `L0`, `l1`, `J0`, `J1`, `tau`.
- Maintain named/commented Simulink subsystems: plant, controllers, supervisory Stateflow, twin/dashboard, logging.
- Dated backups/version history; hardware configuration backup.
- Mandatory per-member contribution log.
- Cite external videos, code, datasets, tutorials and reference designs; original team design/identification required.
- Safe repeatable disturbance testing and demonstrable emergency stop.

### 1.9 Administrative and deadlines (§3, §8, §10)
- Team formation sheet: names, IDs, section, team lead; deadline set by instructor.
- **Phase 1, week 6:** MATLAB plant and open-loop validation (prefer Simscape).
- **Phase 2, week 9:** Stability analysis, PID + pole placement + LQR, simulation and closed-loop pole/eigenvalue justification.
- **Phase 3, week 13:** Working physical apparatus + real-time control + hybrid Stateflow + live dashboard + embedded twin + complete report.
- Grades: **hardware+GUI/robust disturbance demo 20%; synced digital twin 10%; technical report 20%; oral defense 15%; Simulink controls + Stateflow 20%; demo video 15% (≤5 min).** Successful energy-based swing-up in Stateflow, **on real hardware**, is separately emphasized in the grading table; **no separate weight assigned**.
- Report must include contributions, modelling/stability, design/tuning of three controllers, Stateflow hybrid logic, controller metrics/comparison, detailed sim-versus-experiment.
- Video shows hardware+dashboard+twin and brief control design/simulation discussion.
- Brief warns PID-only or missing pole placement/LQR/Stateflow yields deductions.

---

## 2. Suggested 3-person work breakdown (assistant's proposal, not from brief)

| Member | Primary ownership | Supporting/integration responsibilities |
|---|---|---|
| M1 — Modelling & Controls | Euler–Lagrange, identification procedure, linearization, Simscape/state-space consistency, eigenvalue/controllability/observability, design and tune PID, pole-placement, LQR | Controller output definition, response plots, quantitative benchmarks, report theory |
| M2 — Hardware & Embedded | CAD/fabrication, motor+driver sizing, encoder mechanics/wiring/calibration, MCU and power, emergency stop, real-time motor interface | Supply measured physical parameters to M1, implement controlled actuator plant identification, hardware safety tests |
| M3 — Hybrid, Dashboard & Integration | Swing-up law with M1, Stateflow hybrid capture/safety switching, serial protocol/software link, digital twin, GUI, telemetry, logs | Integration, experimental data processing, sim/real plotting, demo video |

Recommended handoffs:
- Before W6: M2 gives realistic geometry/masses/shaft/inertia to M1; M1 provides validated plant; M3 begins Stateflow/twin using same state/sign convention.
- Before W9: M1 gives simulated validated controllers and interface; M2 demonstrates reliable sensing/actuation with bounded latencies; M3 simulated hybrid logic.
- W10–11: PC ↔ ESP integration and all controllers on hardware.
- W12: disturbances, repeated benchmarks, safety, plots, report.
- W13: submission and oral defense.

Shared state vector (one suggested convention):
`x = [theta0, theta1, omega0, omega1]^T`, angles in radians, upright `theta1 = 0`; **align sign and angle zero across sensors, model, controllers and dashboard**.

---

## 3. User's initial hardware proposal

- **MCU:** ESP32-S3 (user already owns ESP32-S3 N16R8 in earlier context).
- **Actuator:** NEMA 17 **bipolar stepper** (exact motor spec unknown).
- **Driver:** DRV8825 STEP/DIR microstepping driver.
- **Angle sensing:** **2× AS5600** 12-bit absolute magnetic rotary encoders (one arm, one free pendulum).
- **Host:** MATLAB/Simulink for model/controllers/Stateflow and realtime dashboard/twin, communicating with ESP32 by **USB serial/UART or Wi-Fi**. **User specifically rejects deploying ESP32 code through MATLAB.**

### AS5600 notes
- 12-bit, **4096 positions/rev**, quantization ≈ **0.08789° = 0.001534 rad**/count; absolute *single-turn* reading, so arm continuous rotations require unwrapping.
- Both sensors typically use same fixed I²C 7-bit address `0x36`. With **custom ESP32 code**, ESP32-S3 has two I²C controllers, so **put each on its own I²C bus**; a TCA9548A is not inherently necessary. If constrained by certain standard Simulink I²C blocks, a multiplexer might be required, but **the user is writing custom ESP firmware**, so do not push the multiplexer.
- Encoder senses magnet angle using a diametrically magnetized, coaxial shaft magnet; check centering, gap, field strength, and nearby motor magnetic interference. AS5600 must be mounted on correct physical shafts.
- AS5600 directly gives angle; `omega0`, `omega1` need wrap-aware velocity estimation/filtering or a state observer. Naive 1 ms difference of one encoder count corresponds to ≈1.53 rad/s quantized derivative spike. Noise, filter latency, sensing jitter all affect LQR/Stateflow.
- Need absolute zero calibration and correct upright convention.
- Independent pendulum encoder ≥360 “pulses”/rev wording needs instructor confirmation re AS5600 absolute digital values.

### DRV8825 + stepper notes
- DRV8825 expects **STEP/DIR/ENABLE**, sets winding current limits (not a direct motor shaft torque setpoint); microstepping up to 1/32. A NEMA 17 frame specification gives *neither* reliable torque nor speed: need exact holding torque, rated current, phase resistance/inductance, rotor inertia, torque-speed curve.
- Stepper commanded microsteps specify *electrical field position*, **not guaranteed actual mechanical angle**, especially under dynamic load; may miss steps/resonate/stall.
- It can drive Furuta pendulum and perform energy-based swing-up if the controller outputs an appropriate motion/acceleration reference and motor actually follows within torque-speed limits; avoid claiming direct torque control.
- A **commanded arm acceleration input** can be convenient: integrate `alpha_cmd` to `omega_cmd`, generate step frequency and direction with hardware-timed or suitable high-speed pulse engine. This is a *modelled/assumed acceleration actuator*, not an inherent property of stepper hardware; validate step-tracking, saturation, delay and resonance.
- At 1/16 microstep with 200 steps/rev, 3200 pulse commands per rev; at 5 rev/s → 16,000 pulses/s; don't generate these with blocking delays inside a feedback loop.
- Current limit must be set to the **specific carrier sense-resistor** value. Many Pololu-style 0.1 Ω boards use `I_limit = 2 × Vref` (A if Vref V), but clones vary. Supply bypass capacitors close to VMOT and thermal management. Safe enable/stop design.
- **Second AS5600 on arm** is better than relying on inferred step counts: can detect actual motion/tracking discrepancies.

---

## 4. Reference stepper project raised by user

**GitHub:** https://github.com/ferrolho/rotary-inverted-pendulum

User pointed to this as proof that a stepper can balance a rotary inverted pendulum. In earlier chat the assistant stated:
- Lightweight 3D-printed system; NEMA 17 (claimed `17HS4023`, rated about 1 A in repo docs), Arduino Nano, pendulum AS5600, arm position from commanded steps.
- Originally **DRV8825**, subsequently **TMC2209**, both reported to achieve balancing; TMC2209 smoother in their documented testing.
- Firmware discussed **stepper acceleration commands** and hardware pulse generation rather than issuing repeated stop/start position targets; reinforcement-learning policy was also mentioned. The repo is not necessarily a direct PID/LQR or energy-based swing-up implementation; **do not confuse stabilization proof with evidence that all required course controllers are already implemented in this repo.**
- Earlier assistant quoted a comparison (approximately DRV8825 99.6% balanced vs TMC2209 100% for a test); treat as **repository-specific anecdotal measurements**, not universal driver performance guarantees. Verify against current repository before reusing exact specs or software details.
- Main transferable ideas: low-inertia design, stepper motion-control abstraction, actual latency measurement, soft angular limits, USB serial telemetry, step-timing safety. **Develop original control design and cite repo; copying is forbidden by course brief.**

---

## 5. Alternatives subsequently discussed

### 5.1 Speed-controlled servo
- Possible with a genuine fast bidirectional continuous-rotation **closed-loop velocity servo** (not simply an ordinary limited-angle hobby RC servo).
- A pendulum swing-up/controller can calculate desired acceleration `alpha_cmd`; integrate to an angular-velocity reference `omega_ref` fed to servo's velocity loop.
- But actual input is servo speed setpoint; need characterize internal velocity-loop bandwidth, delay, current/acceleration limits. Torque-input four-state Furuta model cannot be used **unchanged** unless inner servo dynamics are justified/augmented.
- User has not specified which kind/model of servo; keep ambiguity explicit.

### 5.2 Brushed DC motor with incremental quadrature encoder + H-bridge
**Parts needed:**
1. Brushed DC motor (direct drive or minimal-backlash gearbox) and **quadrature incremental encoder** on motor/arm to measure `theta0`.
2. Reversible H-bridge driver sized for motor supply and **peak/stall current**, not just nominal current; examples discussed TB6612FNG for small currents; DRV8871 for somewhat larger motors; VNH5019 for higher-current motor. These are examples, **not final selections nor verified local listings**.
3. **One** AS5600 and magnet for the free pendulum, provided course accepts its resolution/output type; motor encoder replaces arm AS5600 if mechanically appropriate.
4. ESP32-S3 with PCNT quadrature counting and MCPWM PWM generation; I²C for AS5600; USB UART to MATLAB; motor power, logic supply, caps, E-stop, shaft bearings/coupling/end stops.
5. Level translation if encoder A/B outputs are 5 V; ESP32 ordinary GPIO is not 5 V tolerant.
6. **Optional current sensor/current-control loop.** Motor can run with simple PWM/voltage control but then cannot equate PWM to motor torque.

Encoder decoding (if PPR means *channel cycles per revolution*): `4 × PPR` count edges/revolution in quadrature x4. Definitions of CPR/PPR differ between manufacturers. A geared motor's encoder may measure rotor, not exact output arm position due to backlash. Incremental encoder needs zero/homing on reboot, unlike absolute AS5600.

Suggested motor evaluation: voltage; torque-speed curve; rotor and load inertia; continuous/stall current; motor winding resistance/inductance; `Kt`, `Ke`; gearbox ratio/backlash/friction; resolution; acceleration/reversal limits; shaft support. No exact motor or geometry selected yet.

### 5.3 Current-controlled DC motor
- Brushed DC torque is `tau_m = Kt * i`; using suitable current feedback and a rapid inner current loop gives approximately direct torque command (subject to saturation).
- Most natural actuator for **mechanical torque-input** model `xdot = A_tau x + B_tau tau`; outer PID/pole placement/LQR/swing-up requests torque, inner loop converts to current setpoint. Extra parts/software required.
- A sensor alone (e.g. ACS712) does not magically create torque control; actual current feedback and PWM regulation is needed.
- Motor driver current limit/protection is **not necessarily a programmable closed-loop current-command interface**.

### 5.4 DC motor without current loop: crucial derivation and conclusion
The latest discussion focused on the false inference “no current control means cannot do energy swing-up / torque model”.

Brushed DC armature dynamics (idealized):

```math
V_a = R_a i + L_a \frac{di}{dt} + K_e \omega_m
```

```math
\tau_m=K_t i
```

```math
J_m\dot\omega_m=\tau_m-b_m\omega_m-\tau_{load}
```

If **electrical time constant** `L_a/R_a` is small compared with important mechanical bandwidth, approximate:

```math
i\simeq\frac{V_a-K_e\omega_m}{R_a}
\quad\Rightarrow\quad
\tau_m\simeq\frac{K_t}{R_a}V_a - \frac{K_t K_e}{R_a}\omega_m
```

Assuming a direct-drive torque-input linearized plant with four states `x=[theta0, theta1, omega0, omega1]^T`, where `omega_m=omega0`, substituting gives a **voltage-input** linearized model:

```math
\dot{x}=A_Vx+B_VV_a
```

```math
A_V=A_{\tau}-B_{\tau}\frac{K_t K_e}{R_a}
\begin{bmatrix}0&0&1&0\end{bmatrix},
\quad B_V=B_{\tau}\frac{K_t}{R_a}.
```

For geared motors, reflect the gearbox ratio, efficiencies, back-EMF, inertia and friction consistently; above direct-drive identity **does not hold without modification**. If inductance is not negligible, include winding current as an **additional state**. Real H-bridges have deadband, nonideal drops, direction-dependent coast/brake characteristics, bus dynamics and limits.

Now legitimate LQR/pole placement outputs **voltage** instead of torque:

```math
V_a=-K_V x
```

`duty ≈ sat(V_a/Vbus, −1, +1)` for an **idealized bipolar signed PWM-voltage interface**. Validate this approximation on actual motor/bridge. `u` in benchmarking is volts or duty, not Nm; report units and control saturation appropriately.

**Energy-based swing-up without current control:** derive/adapt an energy-pumping law using the chosen control input (motor voltage/PWM), or compute desired torque from swing-up then translate approximately to voltage with velocity compensation: `V_ref ≈ (R_a/Kt)*tau_ref + Ke*omega_m` under quasi-steady assumptions and within driver limits. The law's energy injection sign must follow the actual Furuta dynamics and chosen angle/axis convention; don't present arbitrary `sign()` law as universally correct. Implement tests in simulation first and ensure soft limits.

**Conclusion of conversation:** DC voltage-control is not inherently inferior to STEP/DIR stepper: both indirectly produce torque. DC voltage-input modelling is usually closer to standard motor and control-theory coursework; STEP/DIR is convenient for commanding trajectory/acceleration but harder to model as a *true torque actuator* and can miss steps. The better choice depends on size/inertia, torque-speed bandwidth, friction/backlash, instrumentation, MATLAB real-time control and cost.

---

## 6. Actuator-input modelling choices — do not mix them up

### Conventional torque-driven Furuta model

```math
M(q)\ddot q+C(q,\dot q)\dot q+G(q)+F(\dot q)=
\begin{bmatrix}\tau\\0\end{bmatrix},\qquad
q=\begin{bmatrix}\theta_0\\\theta_1\end{bmatrix}
```

Linearized around upright:

```math
\dot x=A_\tau x+B_\tau\tau,
\qquad \tau_{LQR}=-K_\tau x.
```

For a current-regulated motor this is a useful first-order approximation to the actuator. For **PWM-voltage-driven DC motor**, substitute the electrical relationship to make voltage the input instead. For an actuator model with a significant torque/velocity time constant, augment the state.

### Ideal arm acceleration input for stepper or speed servo
If a **fast actuator could accurately track commanded horizontal-arm acceleration**, take the second (unactuated pendulum) row of the Furuta dynamics:

```math
M_{10}(q)\ddot\theta_0+M_{11}(q)\ddot\theta_1+h_1(q,\dot q)=0,
\quad u:=\ddot\theta_{0,cmd}
```

then under **tracking assumption** `ddot(theta0) ≈ u`:

```math
\ddot\theta_1=\frac{-M_{10}(q)u-h_1(q,\dot q)}{M_{11}(q)}.
```

Linearization yields a valid hypothetical 4-state acceleration-input plant:

```math
\dot x=A_a x+B_a u,
\quad u_{PP}=-K_{PP}x,
\quad u_{LQR}=-K_{LQR}x.
```

For NEMA17+DRV8825, software integrates acceleration to step velocity, emits pulses. For speed-controlled servo, integrates acceleration into velocity reference. **Accurate acceleration tracking is not guaranteed**: characterize motor acceleration limits, resonance, lag and saturation, and ensure controller definition of `u` corresponds to actual plant input. Derive energy-based swing-up accordingly.

### Simple pendulum energy used as an explanation, not full Furuta energy
For `theta1=0` upright and hanging `theta1=pi`, illustrative pendulum-only energy relative to hanging is:

```math
E\approx\frac12 J_{pivot}\dot\theta_1^2+mg\ell(1+\cos\theta_1),
\quad E_{upright}=2mg\ell.
```

Here `ell` = COM pivot distance. **Full Furuta energy includes arm/pendulum coupling**, so develop correct mechanical energy based on derived dynamics and choose signs/energy pumping law accordingly. Swing-up handoff to PID/LQR/pole placement requires angular position *and angular velocity* capture guard and safety/fall-back transition in Stateflow.

---

## 7. PC ↔ ESP32 interface and timing

**User explicitly:** NO auto-deploy/code generation to ESP32 from MATLAB. Custom firmware; UART/USB serial or Wi-Fi communication with MATLAB scripts/Simulink.

Two distinct architectures have been considered:

**A — Host closes feedback loop (most directly consistent with course wording, pending instructor confirmation):** ESP32 reads encoder(s), timestamped telemetry, generates safe PWM or STEP/DIR and enforces watchdog/end limits. PC MATLAB/Simulink + real **Stateflow** computes 4 controller laws and hybrid switching, sends motor commands back. Dashboard/twin on PC. Main concern USB-serial/OS latency and determinism, especially for unstable upright stabilization. Measure round-trip latency, sensor age, jitter, missed cycles and effective sample rate; consider whether a real-time host/target arrangement is necessary.

**B — ESP32 closes feedback loop (more real-time robust but compliance concern):** ESP32 locally runs PID/LQR/swing-up+mode machine, while MATLAB only designs, tunes, visualizes and logs over serial/Wi-Fi. Such a firmware-written finite-state machine is **not by itself Stateflow implementation**. Must obtain instructor acceptance or use a qualifying Stateflow real-time architecture.

**UART recommended over Wi-Fi** for first prototype (typically more deterministic), but baud rate alone does not guarantee low/constant host scheduling latency. Use binary packets with framing, version, timestamp, mode, theta0/theta1, omega0/omega1, u, current if present, seq, CRC. Enforce stale-command timeout. Telemetry/display at 20–50 Hz can be slower than control/sensing. Previous indicative starting rates 500 Hz on MCU sensors/control and 20–50 Hz dashboard **were suggestions, not proven**; if PC closes loop choose rate based on measured end-to-end timing. Reference project reportedly used fast sampling / lower rate policy; verify before adopting numerical figures.

Safety: physical E-stop disconnects/disable motor drive; end stops or soft limits; watchdog to de-energize on lost comms, avoid unsafe uncontrolled startup, separate supply and grounds, tidy cabling, clearance. Device design must account for arm encoder wiring rotations.

---

## 8. Suggested next steps / questions Astra should ask

1. **Confirm official instructor acceptance** of 3-person team rather than required 6–8, and clarify custom ESP32 + serial MATLAB/Stateflow setup's compliance.
2. **Choose actuation strategy**, not just part family: (a) torque-controlled brushed DC with current loop, (b) voltage/PWM-controlled brushed DC model, (c) stepper with arm-acceleration input, or (d) closed-loop velocity servo. State control-input units explicitly.
3. Get **mechanical parameters** before selecting actuator: horizontal arm length/mass/COM/inertia; pendulum length/mass/COM/inertia; bearings, friction; expected arm rotation range; CAD or estimate; transmission ratio.
4. Obtain **exact motor datasheets** (NEMA 17 torque-speed, winding L/R/current; brushed DC voltage, resistance, inductance, Kt/Ke, current, encoder counts, rpm; servo bandwidth if relevant). No final exact motor model/part numbers known from this conversation.
5. Perform first-principles **torque, speed, acceleration, inertia and braking** estimate; realistically simulate current/voltage/stepper limits rather than treating an ideal motion source as real.
6. Clarify **if current sensing is an option**, budget, availability in **Egypt**. User tends to prefer local electronics suppliers (e.g., electronics-hub.best) and asks not to use Ubuy; this is background preference, not a confirmed constraint for this turn.
7. Set encoder interface (incremental A/B + hardware PCNT or two AS5600s separate I²C buses); calibrate frame angles, unwrapping, signed velocities.
8. Design plant/controller interface and structured **Stateflow capture + hysteresis + safety transitions**, simulation benchmarks, and viable experimental protocol.
9. Build phased demonstration sequence: open-loop motor/encoder tests → model identification → closed-loop upright only → energy swing-up → guarded transitions → disturbance benchmarking → digital twin/report.
10. If asked for implementation code, user generally prefers **complete runnable code**, not fragmentary fixes; ESP32-S3 C++/ESP-IDF/Arduino and MATLAB version/toolbox details must be clarified.

---

## 9. Important factual/epistemic boundaries

- The course brief is the authoritative source for requirements; the rest of this handoff is the assistant's proposed approaches and conversation reasoning.
- Course says **at least 360 pulses/rev on pendulum encoder**; don't automatically assert an AS5600 I²C sensor meets literal wording without instructor confirmation.
- Course may call PC-host plus remote hardware “HIL,” but strict classic HIL terminology is different; use course interpretation and instructor's guidance.
- **NEMA 17** is a motor frame size, **not a fixed torque spec**; **DRV8825** is a stepper current-regulating driver, **not independent commanded torque control**.
- **PWM without current sensing** can implement voltage-input control; do **not** claim torque is directly proportional to duty cycle regardless of speed/back EMF. The approximate inverse relation `V_ref ≈ R*tau_ref/Kt + Ke*omega` only holds under relevant electrical/drive assumptions and within limits.
- A stepper can swing up a Furuta pendulum, but a specific GitHub balancing demo is not proof that your selected motor + mechanics will meet the course's energy-law and three stabilization controller requirements.
- Servo speed control, stepper arm acceleration and DC voltage input produce **different state-space input matrices**. Do not use `K` derived for one plant/input directly with another.
- There is **no finalized choice** between stepper vs DC, no definite driver purchase, no current sensor chosen, no motor geometry or numeric dimensions, and no approved PC/MCU execution architecture.

---

## 10. Suggested opening prompt to Astra

> Continue helping me design my Control II Furuta pendulum project. Read this handoff as conversation context, not as a final design. I am a member of a 3-person team (the course brief says 6–8, so this needs approval). I have an ESP32-S3 and initially considered 2× AS5600 + NEMA17 + DRV8825, but now I'm comparing that with a brushed DC motor with quadrature encoder and an H-bridge, potentially **without a current-feedback loop**. I will NOT auto-deploy ESP32 firmware from MATLAB: I intend custom ESP32 firmware communicating with MATLAB/Simulink/Stateflow via UART or Wi-Fi. My last concern was how DC voltage-input control can implement energy-based swing-up and LQR/pole placement, versus the stepper acceleration-input approach. Please prioritize technical correctness, compliance with the attached Fall 2026 brief, quantitative actuator modelling, and a workable prototype. Begin by helping me select an actuator architecture, derive/compare the models and identify what motor and pendulum dimensions are required to size it.

