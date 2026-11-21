# Furuta Lab: user and developer manual

Furuta Lab is the Windows desktop application of this repository (PySide6/Qt, Python 3.11). It talks to the
FurutaWorkbench firmware (1.13) on the ESP32-S3 over a binary protocol on USB serial (460800 baud), shows the
sensors live, records runs, drives the motor under firmware safety limits, identifies the plant model, starts and
supervises the controllers that run on the ESP32, and shows the rig and its models as a 3D digital twin.

This manual describes what the code does. Labels in quotes are the exact texts of the widgets. Entry point:
`desktop/main.py`; package: `desktop/furuta_lab/`. The short operating guide `docs/FOCUSED_GUIDE.md` is still
shown by Setup & guide -> "Open the focused operating guide".

Contents: 1 Purpose and modes, 2 Install and launch, 3 The window, 4 Pages, 5 Safety, 6 Data, 7 Architecture,
8 Scripts without the GUI, 9 Troubleshooting.

---------------------------------------------------------------------------------------------------------------

## 1. What the app is for

The app has three jobs in the project workflow:

1. Characterize and identify: record still noise, the current-sensor zero, encoder scale, free swings and
   square-wave runs; fit the model (Parameter estimation) and export it for MATLAB.
2. Test controllers on the rig: start the controller that runs on the ESP32 (swing-up + LQR or PID balance),
   keep the heartbeat, show its state and record the run.
3. Compare model and rig: the Digital twin tab draws the rig from the live stream next to the identified model,
   the model observer and a closed-loop copy of the controller.

### Two operating modes

| Mode | Where the control loop runs | What the app does | Status |
| --- | --- | --- | --- |
| Streaming / HIL | Simulink or MATLAB on the laptop | would stream the rig state to the laptop and send the duty back | not built; the app does not do it |
| Onboard control | on the ESP32-S3, in a 1 kHz firmware loop; controller period 4, 2 or 1 ms (firmware 1.13: LQR or PID balance, swing-up, model observer) | starts the run, keeps the heartbeat, shows state, records | built and run on the rig |

What the app does in both cases is "streaming" in the sense that the firmware sends 50, 100 or 250 core records
per second to the laptop (default 100 Hz, 250 Hz during onboard runs and estimation); the laptop never closes a
control loop. Open-loop experiments (Motor tests, Parameter estimation Step 2) are uploaded to the ESP32 as a
finite sequence of segments and executed there; the laptop only keeps the heartbeat. If the laptop, the cable or
the app fails, the ESP32 stops the motor (section 5).

---------------------------------------------------------------------------------------------------------------

## 2. Install and launch

Requirements:

- the rig with the ESP32-S3 board, connected through its FTDI FT232R USB-serial bridge (the ESP32's own USB port does
  not carry the protocol); a COM port appears for it. The app lists COM11 first, the scripts default to COM9 or
  COM11 (section 8);
- a 12 V motor supply for any powered run (the app works without it; motor buttons stay disabled);
- Windows, Python 3.11 in `.venv` (the existing `.venv` is 3.11.0). `Setup-Furuta-Lab.ps1` only creates the
  `.venv` if it is missing and its error text asks for "Python 3.12+"; use the interpreter that works for the pinned
  wheels.

`desktop/requirements.txt` (pinned):

```
PySide6-Essentials==6.12.0
pyqtgraph==0.14.0
numpy==2.5.4
scipy==1.18.1
pyserial==3.5
PySide6-Addons==6.12.0
```

No `numpy<2.5` note exists in the code or the docs; numpy is pinned to 2.5.4. PySide6-Addons provides Qt Quick 3D,
which the Digital twin 3D view needs. Without it the app starts and everything except the 3D view works (the view
slot shows "3D view not available" and the reason). MATLAB is optional (section 4.4).

Install (once) and launch:

```
powershell -File Setup-Furuta-Lab.ps1        # creates .venv and runs pip install -r desktop/requirements.txt
.venv\Scripts\python.exe desktop\main.py     # or: Start-Furuta-Lab.ps1 / double-click Start-Furuta-Lab.cmd
```

`Start-Furuta-Lab.cmd` starts the app with `pythonw.exe` (no console). `desktop/main.py` sets
`QT_OPENGL=software`, redirects stdout/stderr to `data/logs/lab-YYYYmmdd-HHMMSS.log` (errors and tracebacks go
there, also for `pythonw`) and calls `furuta_lab.app.main()`. The plots use raster drawing (pyqtgraph with OpenGL
off); the 3D twin uses Qt's RHI (Direct3D 11), not OpenGL.

Launching the app sends no motor command. Close the app before any script opens the serial port (section 8): one
program can own the port.

---------------------------------------------------------------------------------------------------------------

## 3. The window

Title "Furuta Lab - sensors and characterization", 1320 x 880.

Top bar (always visible):

| Control | Meaning |
| --- | --- |
| port combo, "Refresh ports" | lists `serial.tools.list_ports` devices, COM11 first |
| "Connect USB" / "Disconnect" | opens the port, HELLO, session, reads the profile, starts the stream at 100 Hz; the driver stays disabled |
| "Open recording..." | opens a run folder (`manifest.json`) or a still capture folder (`capture.json`) |
| state pill | DISCONNECTED, CONNECTING, BOOT, DISARMED (green), ARMED (amber), RUNNING (red), "RUNNING - stopping", "FAULT - <fault name>", "FAULT - link lost" |
| "12 V connected - allow motion" (amber) | the permission box. Ticked it turns red: "MOTION ALLOWED - 12 V connected". Unticking sends STOP. Default: unticked, on every start |
| "STOP / DISABLE" (red) | sends STOP, cancels any run in preparation, finishes and loads the running recording |

Chip row: "Arm centered", "Pendulum down set", "Current zero set", "Scale verified" (green with a check when true), and
a sensor line (current, magnet status, sample count, link errors, dropped records, "capture recording/armed").

Progress row (hidden when idle): shown while a background task runs, with "Cancel" (sets the cancel flag, sends
STOP, kills a running MATLAB job). While a task is busy all action buttons are disabled; a second task is refused
("Wait for the current task or cancel it").

Tabs, in order: Live & recordings, Still noise & filters, Motor tests, Parameter estimation, Onboard control,
Digital twin, Setup & guide. Left: the page of the tab. Right (splitter): a stack of views chosen by the tab:

| Right view | Shown when |
| --- | --- |
| signal viewer (`SignalView`) | Live & recordings, Motor tests, Setup, and the default for Onboard control |
| noise / spectrum / MATLAB plot | after an analysis on the noise or Extras buttons |
| fit view (`FitView`, stacked x-linked panels, pole-zero plot) | Parameter estimation, after a fit or preview |
| 3D twin | Digital twin tab; Onboard control tab when the twin option "Show the twin instead of the plots..." is ticked |

Below the right view: "Messages" (the last report, Markdown) and "Raw data" (table of the newest 50 live samples, or
up to 300 samples of the visible range of an opened recording; "Pause"; "Export CSV...").

Every action runs through `LabApp.safe`: an exception does not crash the app; it is shown in Messages as "Action
needs attention" and in the status bar.

---------------------------------------------------------------------------------------------------------------

## 4. Pages

### 4.1 Live & recordings

Group "Signals":

- "Separate" (default, stacked x-linked plots) or "Combined (normalized +-1)".
- "Raw units" (off): counts, AS5600 raw 0-4095, mV, duty fraction instead of degrees, amperes and percent.
- "Window (s)" (default 10, range 1-60): live time window.
- "Software smoothing (Hz)" (default 20, 0.5-200): 2nd-order Butterworth applied to current and speeds; the label
  shows the delay. The live stream is 100 Hz, so cutoffs of 45 Hz or more are shown unsmoothed.
- One checkbox per signal with a readout (mean +- sigma over the last 0.5 s, or over the visible range of a
  recording). Signals: Arm angle, Pendulum angle, Arm speed, Pendulum speed, Current (raw), Current (smoothed),
  Current (MCU filter), Requested duty, Applied duty, Magnet status, Magnet AGC, Onboard controller state.
  Default ticked: Arm angle, Pendulum angle, Current (smoothed), Applied duty. "All" and "Default" buttons.
- "Back to live view" appears while a recording is shown.

Group "References - motor disabled, no motion" (three steps, each needs the device connected and DISARMED, and no
recording running):

1. "Arm is centered here": the arm is held at its mechanical centre; the firmware zeroes its encoder count and sets
   `centered`.
2. "Pendulum is hanging down here": stores `pendDownRaw` (AS5600 raw angle with the pendulum hanging still).
3. "Zero current": 1 s measurement of the current-sensor offset with the motor unpowered. Accepted only with a
   mean between 2200 and 2800 mV, no clipped samples and at least 100 samples; otherwise the report says
   "Current zero REJECTED".

Group "Live recording": "Start recording" / "Finish recording" writes a run folder (section 6) of the stream. It
needs DISARMED and starts no motor command.

References live in RAM on the ESP32 (arm centre) and in the profile (`pendDownRaw`, `currentZeroMv`). A reboot of the
board clears "centered" (section 9).

### 4.2 Still noise & filters

Group "1 - Record the stationary system": tick "Both joints are still; driver disabled" (required), "Record seconds"
(5, 0.2-30), "ADC capture rate (Hz)" (500, 1000 or 2000), "Record all three sensors". The firmware captures into its
own memory (4 MiB) and the app downloads it; ADC and filter settings are restored afterwards; the capture is saved
as a still capture (section 6). A capture that contains motor drive is rejected.

Group "2 - Inspect noise": "Sensor" (current sensor, motor encoder, pendulum angle), "Plot frequency up to (Hz)"
(100), "Plot noise spectrum + statistics" (Welch PSD, SD, peak-to-peak, drift, gaps), "Plot captured signal over
time".

Group "3 - Compare low-pass cutoffs": "Keep motion up to (Hz)" (0) and "Allowed filter delay (ms)" (20) -> "Compare
low-pass cutoffs" lists 2nd-order Butterworth candidates that lose at most 1 dB in the band and meet the delay;
"Preview selected filter", "Use selected cutoff for live smoothing", "Deploy selected CURRENT filter to MCU"
(DISARMED only, current sensor only; mechanical filters stay offline). The MCU overcurrent trip uses the MCU-filtered
current.

### 4.3 Motor tests (finite open-loop experiments)

Every button that moves the motor ends in `authorize_run` (section 5.1).

Group "1 - Mark safe travel while unpowered": "Mark left safe endpoint" / "Mark right safe endpoint" (move the arm by
hand; needs the 12 V box unticked and the arm centred; the centre must lie strictly between the endpoints). When
both are marked the app proposes a travel of 90 % of the closer side. Never seek a hard stop with the drive.

Group "2 - Commission a finite motion envelope" (defaults): "Maximum duty (%)" 0, "Travel each side (counts)" 0,
"Maximum run seconds" 10, "Current trip A (0 = off)" 4, "Speed trip counts/s (0 = off)" 0, "Slew rate (%/s)" 1000,
then "Apply these limits to RAM" (DISARMED; readback confirmed; starts nothing; the device profile also gets
`assumedBusV` from Setup).

Group "3 - Actuate and record": "Waveform" (hold, sine, chirp, prbs), "Signed amplitude (%)" 2, "Command seconds" 2,
"Frequency / chirp start (Hz)" 1, "Run & record - MOVES MOTOR". The run is 0.5 s hold, the waveform, 0.5 s hold.
Refused if the sequence is longer than "Maximum run seconds" or the amplitude exceeds "Maximum duty".

Group "4 - Measure breakaway dead zone": "Start duty (%)" 1, "Duty increment (%)" 1, "Maximum test duty (%)" 3,
"Movement threshold (counts)" 3, "Pulse length (s)" 0.25, "Rest between pulses (s)" 0.5, "Run alternating pulses -
MOVES MOTOR" (needs the slew rate to reach the top level within half a pulse), "Analyze recorded breakaway
pulses", "Save last dead-zone result to device RAM", label "Device dead zone: + x % - y %". After a breakaway run
the result is analysed and saved to RAM automatically. Dead zones are stored (`deadzonePos`, `deadzoneNeg`); the
firmware does not apply them to commands.

### 4.4 Parameter estimation (model identification)

Method: report sections 4 to 7. Code: `estimate_page.py` (GUI) and `estimation.py` (pure functions, no Qt).
Everything is done on the assembled rig. Fits run in the background worker; plots go to the fit view and tables to
Messages. A step whose inputs changed is marked stale and must be repeated.

Group "Session": "New session..." (name, default `session_YYYYmmdd_HHMM`), "Open session...". A session is the
folder `data/estimation/<name>/` with `session.json` (settings, recordings, results, model); `last.txt` remembers
the last one, which is opened at start.

Step 0 - Setup: "Motor supply Vs (V)" 12; "Current polarity" +1/-1; "Pendulum sign used for fitting" (from each
recording / +1 / -1); "Reverse the pendulum direction on the device (pendSign)"; "Pendulum timing correction
(ms)" 0 with "Estimate the pendulum timing from the fit runs" (scans 0-15 ms on the pendulum-equation R2); "Armature
inductance Lm (mH, optional, 0 = none)" 0; a checklist (current zero, arm centred, pendulum-down, scale, limits, stream
250 Hz, MCU current filter on); "Apply estimation recording settings" (250 Hz stream and 40 Hz Butterworth MCU
current filter, RAM only; previous values saved in the session) and "Restore normal settings". Advanced: "Signal
filter fd (Hz, 0 = auto: 40 at 250 Hz)", "Equation filter fp (Hz)" 10, "Moving threshold omega min (rad/s)" 0.3.

Step 1 - Free swing, arm held, motor off: "Free-swing record length (s)" 10, "Record free swing (passive)"
(never arms), list "Free-swing recordings", "Add the open recording", "Add folder...", "Remove selected", "Show
prepared signals", "Fit free swing" (Jp/G, b1/G, dry friction, natural frequency, with standard errors and checks).

Step 2 - Square-wave runs, pendulum hanging, MOVES MOTOR: "Seconds per frequency" 4; presets "Run A" (25 %,
1, 1.4, 3.5, 4.5 Hz), "Run B" (20 %, 1.2, 2.7, 3.5, 5 Hz), "Validation" (22 %, 1.1, 3, 4 Hz), each with amplitude,
frequency list, a prediction line (predicted arm swing against the commissioned travel, resonance warning within
25 % of the pendulum natural frequency, slew and limit requirements), "Preview" and "<run> - MOVES MOTOR". Runs go
through `start_sequence` and `authorize_run`. Amplitudes below twice the stored dead zone are raised to it. Lists
"Fit runs (A, B)" and "Validation runs".

Step 3 "Fit Rm and Kb" (electrical); Step 4 "Fit pendulum equation" (option "Use the free-swing Jp/G, damping and dry
friction; fit only the coupling h/G (recommended)", on); Step 5 "Fit arm equation" (option "One Coulomb friction for
both directions"); Step 6 "Refine by simulation" (tick h, b1, tau_p [default], Ja, Beq, tau_c+, tau_c-; "Window length
(s)" 4; "Show least-squares model on fit runs", "Refine", "Accept refined model", "Keep least-squares model"); Step 7
"Model against hardware" (any recording; "Re-sync to the measured state every (s, 0 = whole run)" 0; "Compare model
with hardware"); Step 8 "Model: state space and transfer functions" ("Discrete sample time Ts (ms)" 1, "Show model"
with poles and zeros, "Export model files..." -> parameters.json, `furuta_model.m`, `furuta_rhs.m`, TeX tables; in
MATLAB `run furuta_model.m`); Step 9 "Physical units (optional)" (nominal Ktau = Kb, known added mass, weighed
pendulum, assumed gearbox efficiency; "Apply scale").

Group "Extras" (below the estimation steps, part of this page): "MATLAB speed model" (PT1 or tfest), "Fit / validate
speed model in MATLAB (open recording)", "Export latest analysis JSON". MATLAB runs in a separate process
(`backend.MatlabBackend`, `matlab -batch ... furuta_backend`, timeout 180 s, jobs in `data/matlab-jobs/`); the
executable comes from `FURUTA_MATLAB`, then `PATH`, then `D:/MATLAB/R2023b/bin/matlab.exe`. Without MATLAB the
message is "Set FURUTA_MATLAB to matlab.exe; sensor/noise tools work without MATLAB".

### 4.5 Simulation bench (no hardware)

There is no tab for it. `simbench.py` is the simulated bench used by the tests and by the page: the Furuta model
with the inductance neglected, driven by the duty the firmware would apply (slew limit, through zero before
reversing, zero duty = open-circuit), with the sensors as the firmware streams them (encoder quantization, 12-bit
AS5600, noisy current). Functions: `firmware_segments` (the Step 2 "Preview" curves: requested and applied duty),
`sim_furuta`, `sim_swing`, `to_rows`, `write_run` and `synthetic_session` (a complete estimation session in a folder,
for tests). Its parameter set `TRUE` is plausible, not measured.

### 4.6 Onboard control

The ESP32 runs the controller. The page starts it, keeps the heartbeat, shows the controller state from the `ctrl` byte
of every core record and records the run. STOP / DISABLE, unticking the 12 V box or closing the app stops the motor
(the heartbeat stops; the firmware lease expires within 300 ms).

Group "1 - Readiness": a checklist with a check (ok), a cross (blocking) or "!" (advisory) for: firmware with onboard
control, device DISARMED, 12 V box ticked, arm centred, pendulum-down reference, encoder scale (the model used
1496 counts/rev), profile signs (armSign +1, pendSign -1 expected), maximum duty, travel (counts and degrees), slew
(advisory below 50 /s), current trip or "IGNORED", capture armed/recording, FAULT, run length and heartbeat,
controller period. Fields (defaults): "Run length - maxDuration (s)" 20 (0.5-3600), "No run-length limit (runs until
STOP, a trip or a fault; firmware 1.8)", "Maximum duty (%)" 50, "Slew rate (%/s)" 10000, "Current trip A (0 =
off)" 4, "Ignore the current sensor during runs (...; firmware 1.10)" (off; removes the ADC_INVALID_OR_CLIPPED and
overcurrent trips; the duty limit is then the only bound on the current), "Apply these onboard-run limits to RAM".
"Set profile signs to the model's (armSign +1, pendSign -1)" appears only when the device signs differ and asks for
confirmation. Travel stays as commissioned on Motor tests.

Group "2 - Run - MOVES MOTOR":

| Controller choice | start arguments | Use |
| --- | --- | --- |
| Swing-up + LQR balance | swingUp true, stab 3 | start with the pendulum hanging at rest, arm centred |
| LQR balance only | swingUp false, stab 3 | hold the pendulum lightly near upright (catch light green), press Start, let go; leaving the catch region ends the run (ONBOARD_PENDULUM_LOST) |
| Swing-up + PID balance | swingUp true, stab 1 | needs firmware 1.13 (HELLO `onboardStabilizers` contains `pid`) |
| PID balance only | swingUp false, stab 1 | as LQR only; the arm integral is reset one period after the catch |

- "Controller period": 4 ms (default, as designed and validated in Simulink), 2 ms, 1 ms (did not swing up on the rig).
- "Record the run to data/runs (stream raised to 250 Hz for the run)" (on).
- "High-rate capture on the MCU (500 Hz records, current at 2 kHz), downloaded after the run" (on): armed before the
  start, downloaded after the run into `<run>/burst/`.
- "Start onboard controller - MOVES MOTOR" (enabled only with onboard firmware, 12 V box ticked and state DISARMED),
  "Clear fault" (enabled in FAULT).
- Live line: state pill (NOT RUNNING, SAFE STOP, SWING-UP, STABILIZE), catch-region light, theta0, theta1, duty,
  current, state; "Last run ended: <reason>" (STOP, run length, FAULT name, ONBOARD_SAFE_STOP, ONBOARD_PENDULUM_LOST,
  PEND_INVALID).

Group "3 - Controller knobs (device RAM; a restart restores the design values)": a table (knob, value, unit, meaning)
read with "Read from device", changed with "Apply edited values" (DISARMED only; only changed values are sent),
"Restore design values", and "Measure controller CPU time on the MCU" (microseconds per step and share of 4/2/1 ms;
an overrun trips CONTROL_OVERRUN). Knobs include LQR gains K0-K3, `velFc`, `fricGain`, catch/exit/safe angles
(`catchTh1`, `catchW1`, `exitTh1`, `safeTh0`), the swing-up knobs (`E0`, `kE`, `aPush`, `wc`, `zc`, `aKick`, ...), and
the PID gains `Kp1`, `Kd1`, `Kp0`, `Ki0`, `Kd0`. `Ts`, `rearm`, `modelArmSign`, `modelPendSign` are read-only; `swingUp`
and `stab` are set by the controller choice. The values and the design come from section 7.5.

What the start does (`OnboardPage.start`): checks controller needs, `authorize_run`, no live recording, no pending
download; cancels a left-over capture; raises the stream to 250 Hz; sets `Ts` if changed; arms the capture; writes
the manifest; sends ARM and START_ONBOARD. After the run it summarizes the run (duration, first STABILIZE time, arm
range, peak current and duty, FAULT) and downloads the capture.

### 4.7 Digital twin

The rig in 3D, posed on every display frame (8 ms timer, only while a view is visible) from the newest stream
record, with no network hop. Mouse: drag to orbit, wheel to zoom. Solid = rig; translucent cyan = the identified
model. Needs PySide6-Addons.

Group "1 - Source": "Live: the connected rig" (default) or "Playback: the open recording" with "Play / pause",
"Restart", speed (0.25x, 0.5x, 1x, 2x), a seek slider; "Live stream at 250 Hz (smoother twin)" (raises the stream, RAM,
DISARMED only).

Group "2 - Model ghost": mode "Identified model (driven by the applied duty)" (default), "UDP packets (MATLAB
twin_send / twin_play)" or "Off"; "Re-sync the model to the rig every (s, 0 = never)" 2; "UDP port to listen on" 7401
(7400 belongs to the Unreal twin). The cyan model (`ModelGhost`) integrates the identified equations with breakaway
friction from the duty the firmware applied, starting from the measured state.

Box "Extra ghosts (side by side, each on its own)" (all off by default):

| Colour | Class | Shows |
| --- | --- | --- |
| orange | `FastResyncGhost` | the model re-synced every 0.2 s: short-horizon model error |
| green | `ObserverGhost` | the controller's model observer (firmware 1.12/1.13, `est_observer_ref.m`) run on the stream: what the controller acts on |
| violet | `ClosedLoopGhost` | the model with its own copy of the onboard controller (supervisor, swing-up, stabilizer, friction, clip and slew); choose "Closed loop stabilizer" LQR (stab 3) or PID (stab 1) and "Closed loop re-sync every (s, 0 = never)" 2 |

Extra ghosts are drawn only, not forwarded over UDP. A ghost that raises an exception is switched off with a message.

Group "3 - Forward to the Unreal twin (optional)": "Send both poses as RIP1 UDP packets" (off), host 127.0.0.1,
port 7400 (the Unreal twin listens on 7400). Packet RIP1: 16 bytes, `'RIP1'`, source byte (0 rig, 1 model), q_arm,
q_pend as float32; angles follow `twin/README.md`: q_arm = theta0, q_pend = theta1 - pi (0 hanging). Use a port other
than the one listened on. Only the rig and the cyan model are forwarded.

Group "4 - View": "Reset camera", "Open in its own window", "Show the twin instead of the plots on the Onboard
control tab". The HUD lists the angles of each pose, the controller mode, the pose age and the frame rate. The pose age
is the delay above the fastest packet of the last 2 s (clocks differ). Latency tip: set the FTDI "Latency Timer" to
1 ms in Device Manager (default 16 ms).

Drawing signs `DISPLAY_SIGN = (-1, -1)` flip both drawn angles (a mirror symmetry; packets keep the convention). With
the arm not centred the arm is drawn at 0 ("arm not centered: arm drawn at 0").

### 4.8 Setup & guide

Group "Encoder scale": "Reduction (provisional)" 34, "Motor encoder cycles / revolution" 11, "Quadrature multiplier" 4,
"Scale verified against a measured output angle", "Apply proposed encoder scale to RAM" (34 x 11 x 4 = 1496 counts per
output revolution).

Group "Known-displacement scale check": "Measured count change", "Measured output angle (deg)", "Use measured
scale" (also sets the arm sign).

Group "Device profile": "Assumed motor supply (V)" 12, "Clear a resolved fault" (CLEAR_FAULT), "Cancel device capture
(stuck "Capture/calibration in progress")" (BURST_CANCEL, firmware 1.10), "Save current profile to device NVS"
(SAVE_CONFIG: keeps the profile after a restart), "Show device / link details (raw units)".

Group "Help": "Open the focused operating guide".

Settings changed by Apply buttons live in device RAM until "Save current profile to device NVS".

### 4.9 Saving, loading and fault recovery

- Every recording (Live recording, Motor tests, estimation runs, onboard runs) is saved as it is taken (section 6) and
  loaded back into the viewer when the run ends ("Run saved", "Run loaded").
- "Open recording..." loads a run folder or a still capture folder into the same viewer; the Raw data tab and
  "Export CSV..." follow it.
- A run that ends in a FAULT is still saved; `final_status` in the manifest has the fault. The pill shows "FAULT -
  <name>"; use "Clear fault" (Onboard control) or "Clear a resolved fault" (Setup) when the cause is removed.
- If the link is lost the pill shows "FAULT - link lost" and the app sends STOP; press "Disconnect" and "Connect USB".

---------------------------------------------------------------------------------------------------------------

## 5. Safety

### 5.1 Interlocks in the app

A powered action starts only when `authorize_run` passes: device connected and DISARMED; "12 V connected - allow
motion" ticked; arm centred; maximum duty above 0; travel above 0; the sequence no longer than "Maximum run seconds"
and its peak not above "Maximum duty". Before and after: `Device.inhibit` blocks ARM, START and START_ONBOARD (it is
True after connect, after STOP and after a failed start); STOP bumps a generation counter that cancels a run that is
still being prepared ("Run canceled by STOP"). Run buttons are disabled unless connected, 12 V ticked and DISARMED.
Only one background task at a time. Settings are refused unless DISARMED and not recording. Closing the app stops the
motor.

### 5.2 Guards in the firmware (`firmware/include/engine.h`, `main.cpp`)

| Guard | Behaviour |
| --- | --- |
| arming | ARM needs DISARMED, no latched fault, arm centred, `maxDuty > 0`, `travelCounts > 0`, a valid profile, a current zero when the current trip is on, acquisition ready, no capture/calibration |
| START_ONBOARD | needs ARMED, `countsPerRev`, `pendDownRaw >= 0`, a fresh pendulum reading (< 20 ms, magnet detected), `armSign`/`pendSign` equal to the model's, valid knobs |
| heartbeat | the app sends HEARTBEAT every 80 ms while armed or running; no heartbeat within `heartbeatMs` (300 ms) -> fault HEARTBEAT_TIMEOUT, motor off |
| travel limit | `|count - centre| > travelCounts` -> ARM_TRAVEL |
| speed limit | optional `velocityCountsPerSecond` -> ARM_VELOCITY |
| current trip | filtered current above `currentTripA` for `currentTripMs` (10 ms) -> OVERCURRENT; invalid or clipped ADC (outside 100-3100 mV or older than 20 ms) -> ADC_INVALID_OR_CLIPPED. Both are skipped when "Ignore the current sensor" is on |
| encoder | ENCODER_INVALID |
| clip and slew | duty clipped to `maxDuty`, slew limited, goes through zero before reversing (1 ms blanking) |
| run length | experiments end with their sequence; onboard runs at `maxDuration` (0 = none: heartbeat, STOP and trips remain) |
| pendulum data | onboard run: PEND_INVALID if the reading is older than 20 ms |
| SafeStop | onboard supervisor: `|theta0| > safeTh0` stops the motor (notice ONBOARD_SAFE_STOP); in balance-only mode leaving the catch region stops it (ONBOARD_PENDULUM_LOST) |
| timing | CONTROL_OVERRUN if the control loop overruns; BURST_OVERFLOW / BURST_CORE_OVERFLOW if a capture overflows |
| STOP / DISARM / ABORT | accepted without a session, disable the output, do not clear a latched fault; CLEAR_FAULT does |

A fault latches until CLEAR_FAULT. After a reboot the arm is no longer centred and nothing resumes.

### 5.3 Before a powered run (checklist)

1. The centre rod is out of the rig.
2. 12 V is on and the supply is as in the profile ("Assumed motor supply").
3. The area around the rig is clear; nothing in the arm's travel; a hand near STOP / DISABLE.
4. References set: arm centred (arm in the middle), pendulum hanging still and "Pendulum down set", "Current zero set".
5. Limits commissioned: maximum duty, travel, run length, current trip (4 A default).
6. For onboard runs: signs armSign +1 / pendSign -1, controller period 4 ms, correct controller for the start
   position (hanging for swing-up, near upright for balance only).
7. Tick the 12 V box last; start; STOP / DISABLE ends any run.

The scripts of section 8 add a command-line confirmation flag with the same meaning: `--confirmed-rod-out-12v-on-area-clear`.

---------------------------------------------------------------------------------------------------------------

## 6. Data

Everything is under `data/` (git-ignored except what was committed as results).

| Folder | Content |
| --- | --- |
| `data/runs/<YYYYmmdd_HHMMSS_micro>/` | one recorded run |
| `data/captures/<...>/` | one still capture |
| `data/estimation/<session>/` | one estimation session (`session.json`, exports); `data/estimation/last.txt` |
| `data/matlab-jobs/<...>/` | MATLAB job input, request, result, log |
| `data/logs/lab-*.log` | application log |

### Run folder

- `manifest.json`: `schema` 2, `source` ("hardware"), `created` (Unix time), `config` (the device profile at the start:
  limits, trips, `currentZeroMv`, `sensitivityMvA`, `countsPerRev`, `armSign`, `pendSign`, `pendDownRaw`, dead zones,
  `streamHz`, `adcHz`, filter), `status` (device status at the start: state, `centered`, `centerCount`, `bootId`,
  counters), `metadata`, `complete` (false while recording, true when finished), and after the end `samples`,
  `final_status` and `link_errors`.
- `metadata`: `gear`, `motor_cpr`, `scale_verified`, `12v_declared`, `left_count`, `right_count`, `purpose` (`waveform`,
  `breakaway`, `estimation`, `free_swing`, `free_swing_added_mass`, `onboard`, `onboard_burst`), `segments` and
  `dead_windows` for open-loop runs, estimation fields (`estimation_step`, `preset`, `amplitude_pct`, `freqs_hz`), and for
  onboard runs `controller`, `controller_label`, `start`, `controller_period_s`, `onboard_params` (all knobs),
  `burst_seconds`. Scripts add `suite`, `run`, `note`.
- `samples.bin`: the stream as received: for every packet a 4-byte little-endian length and the payload (22-byte batch
  header and n x 88-byte core records). `load_run` reads it back.
- `burst/` (onboard runs with the high-rate capture): a still-capture folder (below).

### Still capture folder

`capture.npz` (arrays `adc`, `core`), `capture.json` (config, status, info, metadata) and `capture.mat` (the same for
MATLAB). An onboard run keeps its capture in `<run>/burst/`.

### Reload

"Open recording..." (select the run or capture folder). In Python:
`from furuta_lab.backend import load_run, load_capture; rows, manifest = load_run(path)`; `rows` is a NumPy array of
`wire.CORE` (fields in section 7.4). Estimation sessions hold paths relative to the repository and are reopened with
"Open session..." or automatically from `last.txt`.

---------------------------------------------------------------------------------------------------------------

## 7. Architecture (for developers)

### 7.1 Threads and processes

| Thread / process | Job |
| --- | --- |
| Qt GUI thread | widgets, 200 ms refresh timer (`LabApp.refresh`), twin frame timer (8 ms) |
| "Furuta UART" (`Device._loop`) | the only owner of the serial port: reads, parses frames, appends records to a 60000-row buffer and the recording file, sends HEARTBEAT (80 ms, only while armed/running and not inhibited), STATUS (500 ms), replies to RPCs |
| "Lab analysis" (one-worker `ThreadPoolExecutor`) | every job through `LabApp.submit`: connect, apply, capture, fits; results return by Qt signal (`done`, `failed`, `progress`) |
| MATLAB process | started on demand by `MatlabBackend`, never touches serial or the GUI |

Acquisition, heartbeat and STOP do not depend on the GUI or on MATLAB. If the loop raises (for example the MCU
restarted: "MCU restarted; reconnect and re-center"), it sends STOP and sets the state `CONNECTION_ERROR`.

### 7.2 Data flow

firmware -> `Device` (UART thread: `wire.Parser`, `core_batch`) -> `blocks` buffer and `record` file -> pages
(`LabApp.refresh` -> `signals.compute` -> `SignalView`; `OnboardPage.update_status`; `TwinPage.tick` -> `twin_core`
ghosts -> `TwinView`, and optional `UdpLink`). Commands go GUI/worker -> `Device.rpc` (JSON, waits for the ACK, one retry
after 0.6 s) -> firmware. `EstimatePage` and `OnboardPage` get recordings through `LabApp.start_sequence` /
`load_recorded` and never write to the port themselves.

### 7.3 Modules

| File | Role |
| --- | --- |
| `desktop/main.py` | entry, log file, `QT_OPENGL=software` |
| `furuta_lab/app.py` | `LabApp` main window: top bar, Live, noise, motor, Extras, Setup pages, task plumbing, `authorize_run`, recording load |
| `device.py` | serial link, session, RPC, heartbeat, capture/burst download, recording |
| `wire.py` | protocol v1: COBS, CRC-32, record dtypes, parser |
| `backend.py` | `ROOT`, MATLAB jobs, `save_capture`, `load_capture`, `load_run`, `jsonable` |
| `signals.py` | signal specs, unit conversion, smoothing, statistics, CSV export |
| `analysis.py` | noise PSD, filter candidates, segments, dead-zone analysis |
| `viewer.py`, `fitview.py` | signal plots (separate / combined), stacked fit panels and pole-zero plot |
| `estimate_page.py`, `estimation.py` | Parameter estimation page and its pure fitting code |
| `simbench.py` | simulated bench for tests and synthetic sessions |
| `onboard_page.py` | Onboard control page, knob table, `angles`, `summary` |
| `twin_page.py`, `twin_core.py`, `twin_view.py`, `twin_assets/` | Digital twin page, kinematics, ghosts, UDP link; Qt Quick 3D view; `*.ripmesh`, `twin_geometry.json`, `model_params.json`, `twin.qml` |

### 7.4 Wire protocol summary (`protocol/SPEC.md`, `wire.py`)

Frame: COBS-encoded, 0x00 delimiter. Header `<BBHQIH` (22 bytes with CRC): version 1, type, flags 0, session nonce, packet
sequence, payload length; CRC-32 (zlib) of header and payload at the end. Types: 1 command, 2 ACK/NACK, 3 event, 4
samples, 5 burst chunk. Commands, ACKs and events are bounded JSON; a command carries `id` (increasing) and `op`.
The app connects with HELLO (needs `protocol` 1 and `coreRecordBytes` 88), SESSION_OPEN (boot id and challenge),
GET_CONFIG, START_STREAM.

Operations used: HELLO, SESSION_OPEN, STATUS, HEARTBEAT, GET_CONFIG, SET_CONFIG, SAVE_CONFIG, CALIBRATE (arm, down,
current), START_STREAM, STOP_STREAM, UPLOAD_BEGIN, SET_SEGMENT, VALIDATE, ARM, START, START_ONBOARD (`swingUp`, `stab`), STOP,
CLEAR_FAULT, CAPTURE_BURST, BURST_INFO, BURST_CANCEL, READ_CHUNK, ONBOARD_GET, ONBOARD_CONFIG, ONBOARD_DEFAULTS,
ONBOARD_BENCH. Firmware also knows DISARM, ABORT, TABLE_CLEAR, TABLE_CHUNK, UPLOAD_EXPERIMENT.

Core record (type 4, 88 bytes, little endian, `wire.CORE`): `device_us`, `seq` (500 Hz index), `count` (arm encoder),
`pend_raw` (AS5600), `adc_raw`, `current_mv`, `filtered_mv`, `requested`, `applied` (duty), `pend_age_us`, `adc_age_us`,
`mag_status`, `agc`, `magnitude`, `validity` (bit 1 pendulum fresh), `limits` (amplitude, slew, reversal), `apply_us`,
`adc_us`, `pend_us`, `state` (0 BOOT, 1 DISARMED, 2 ARMED_IDLE, 3 RUNNING, 4 STOPPING, 5 FAULT), `ctrl` (0 not running,
1 SafeStop, 2 SwingUp, 3 Stabilize). Batch header: boot id, config id, run id, dropped, count. The stream is 50, 100
or 250 Hz of the 500 Hz acquisition; burst captures keep every record (and ADC records of 22 bytes at 500-2000 Hz).
The parser drops undecodable frames until the first valid one after opening the port (stale bytes of the FT232R).
Events (type 3) include CURRENT_ZERO_UPDATED / REJECTED, ONBOARD_SAFE_STOP, ONBOARD_PENDULUM_LOST, PEND_INVALID,
BURST_COMPLETE, BURST_OVERFLOW. The port is left streaming on disconnect (FT232R behaviour).

### 7.5 Where the numbers come from

- Twin models: `twin_assets/model_params.json` is the identified model of 2026-11-20 (the same as
  `matlab/model/furuta_params_20261120.m`, written by `desktop/scripts/export_model_20261120.py`); `twin_core.load_model`
  reads it through `estimation.full`. Geometry: `twin_assets/*.ripmesh` and `twin_geometry.json` are exported from the CAD by
  `twin/matlab/export_twin_meshes.m` (Unreal twin, `twin/README.md`).
- Onboard design numbers: `firmware/include/onboard_design.h` is generated by `controllers/hybrid/export_onboard.m` from
  `ctrl_hybrid_design.m` in `matlab/simulink_r2026a`. It holds Ts, the LQR gain K, the PID gains, friction, catch and
  swing-up constants and the plant numbers. `twin_core.load_design` parses this header, so the violet closed-loop ghost
  uses the same numbers as the firmware controller (`firmware/include/onboard.h`). Do not edit the header; change the
  design file and run `export_onboard`.

### 7.6 Add a page

1. Write a class with `build(self, layout)` that adds group boxes with `app.group`, `app.number`, `app.button`,
   `app.text` (these helpers give the standard look; `button(..., action=True)` disables the button while a task
   runs).
2. In `LabApp.build` add the tab title to `self.nav` and the builder to the list that creates the pages; the order must
   match. Call `update_status()` of the page from `LabApp.refresh`, and `recording_finished()` from `load_recorded` /
   `run_loaded` if it reacts to recordings.
3. Do work through `app.submit(fn, done, label)` and powered runs through `app.authorize_run()` + `app.start_sequence`
   or `Device.run_onboard`. Never touch the port from the GUI thread.
4. If the page needs a right-hand view, add it to `right_stack` and map the tab in `LabApp.page_changed`.
5. To add an onboard controller, add an entry to `CONTROLLERS` in `onboard_page.py` (`start` arguments of START_ONBOARD,
   optional `needs` stabilizer) and knob descriptions to `KNOBS`.

### 7.7 Tests

```
cd D:\Furuta-Pendulum
.venv\Scripts\python.exe desktop\test_lab.py            # 41 tests (24 in test_lab itself + the 17 of test_estimation)
.venv\Scripts\python.exe desktop\test_estimation.py     # 17 tests: the estimation chain on synthetic data
.venv\Scripts\python.exe desktop\test_twin_ghosts.py    # 5 tests: ghosts on recorded rig runs
```

`unittest`, offscreen Qt, no hardware. `test_lab.py` covers the wire format against the firmware's golden vectors, the
parser, noise and filters, the dead-zone analysis, the setup gating, STOP cancelling a late start, the onboard page
(readiness, knobs, start flow, burst download, PID choices), capture-busy handling and the twin (meshes, kinematics,
ghosts, UDP, playback). The ghost tests on real runs need the run folders in `data/runs`.

---------------------------------------------------------------------------------------------------------------

## 8. Scripts without the GUI (`desktop/scripts/`)

All scripts use `furuta_lab.device.Device` directly. Close the app first: only one program can own the serial port.
Passive steps never arm the driver (`Device.inhibit` stays True). Powered steps need
`--confirmed-rod-out-12v-on-area-clear` (rod out, 12 V on, area clear). The port comes from `--port` (`rig.py`, default COM11) or
the environment variable `FURUTA_PORT` (the rig suite scripts, default COM9).

| Script | Purpose |
| --- | --- |
| `rig.py` | status, setup of limits, passive free swing, powered square-wave runs and restore of the stream/filter; adds runs to an estimation session |
| `rig_suite.py` | one-step-per-call acquisition: `info`, `ref arm|down|current`, `watch`, `record`, `swing` (passive) |
| `rig_suite_batch1.py` | open-loop powered experiments, pendulum hanging: `list`, `validate`, `run`, `chain` |
| `rig_suite_batch2.py` | closed-loop baselines of the onboard controller: `list`, `dry`, `run` |
| `rig_suite_dryrun.py` | onboard controller armed with 12 V off and the rod in; checks `controlOverruns` of the 1 ms loop (needs `--confirmed-12v-off-rod-in`) |
| `rig_suite_analysis.py` | analysis of a free-swing run (period, damping, model check) |
| `rig_suite_backlash.py` | gear play between the motor encoder and the arm from recordings |
| `rig_suite_lag.py` | pendulum-reading lag relative to the arm encoder |
| `rig_metrics.py` | performance metrics of onboard runs (catch transient, steady state, push recovery) |
| `lqi_design.py` | LQI design (LQR with an arm integral, the same law as the firmware PID) |
| `observer_study.py` | estimator x LQR study on the re-identified model |
| `reidentify.py`, `reidentify_variants.py` | headless re-identification with the estimation chain; variants judged on validation runs only |
| `coupling_model.py`, `coupling_fit.py` | two-mass model with shaft-coupling play and its fit |
| `export_model_20261120.py` | writes the 2026-11-20 model as MATLAB parameter set and for the twin |

Offline scripts (analysis, design, fits) do not open the port. The docstrings of some scripts name `desktop/<script>.py`;
the files are in `desktop/scripts/`.

---------------------------------------------------------------------------------------------------------------

## 9. Troubleshooting

| Message or symptom | Cause and action |
| --- | --- |
| Port does not open / "could not open port" / access denied | another program (a script, a second app, MATLAB) has the port. Close it, press "Refresh ports", connect. The ESP32's own USB port does not carry the protocol: use the FTDI port |
| "Choose an available USB port" | no port selected; plug in the cable and press "Refresh ports" |
| "Incompatible firmware" | HELLO did not report protocol 1 and 88-byte records; flash `firmware/` |
| "MCU restarted; reconnect and re-center" | the board rebooted during the session. The arm centre is lost (the Arm centered chip is off, status `centered` false); reconnect, set the arm reference again, check the pendulum-down reference and the current zero (store the profile with "Save current profile to device NVS" if they should survive) |
| "Connection error: ..." / pill "FAULT - link lost" | cable or bridge problem; the app sent STOP. Disconnect, reconnect |
| "Motor test blocked: the arm is not centered ..." | do Live -> References -> "Arm is centered here"; set maximum duty and travel on Motor tests and "Apply these limits to RAM" |
| "12 V is declared disconnected. Powered tests are disabled." | tick "12 V connected - allow motion" only with the supply on |
| "Connect and disarm first" | the state is not DISARMED; press STOP / DISABLE, clear a fault |
| "Run canceled / motion disabled", "Run canceled by STOP" | a STOP or an unticked 12 V box cancelled the start; tick again and restart |
| "Sequence lasts x s but the device maximum run is y s", "Sequence needs x % duty but the device maximum duty is y %" | raise "Maximum run seconds" or "Maximum duty (%)" and apply, or shorten the sequence |
| "Slew rate ... cannot reach ... within half of a pulse" | raise "Slew rate (%/s)" or lengthen the breakaway pulse |
| "Capture/calibration in progress" | a capture is armed or recording, or a current-zero calibration runs. Wait, or Setup -> "Cancel device capture" (firmware 1.10) |
| "Disarm before changing settings" | settings only in DISARMED and not while recording |
| Current zero REJECTED | mean must be 2200-2800 mV, no clipping, 100+ samples; motor unpowered; retry |
| FAULT HEARTBEAT_TIMEOUT | the app stopped sending heartbeats (busy machine, cable); clear the fault, reconnect |
| FAULT ARM_TRAVEL | the arm left the commissioned travel; clear, recentre, raise travel only if the mechanics allow it |
| FAULT OVERCURRENT / ADC_INVALID_OR_CLIPPED | current above the trip for 10 ms, or ADC beyond 100-3100 mV (about +3.2 A); check the 12 V, the sensor zero, the load; "Ignore the current sensor" is the documented but unsafe override |
| FAULT PEND_INVALID, "Pendulum reading invalid" | magnet status or I2C reading bad or older than 20 ms; check the AS5600 magnet distance and the cable |
| FAULT CONTROL_OVERRUN | the 1 ms loop overran; use a slower controller period; see "Measure controller CPU time on the MCU" |
| "<controller> needs firmware 1.13 or newer" | flash the current firmware |
| "This firmware supports only ... ms; flash 1.7 for faster periods" | choose a supported controller period |
| Onboard start: signs differ from the model | "Set profile signs to the model's (armSign +1, pendSign -1)", then do the passive sign check on the Live tab |
| Onboard run ends at once with ONBOARD_SAFE_STOP / ONBOARD_PENDULUM_LOST | arm beyond `safeTh0`, or balance-only started outside the catch region (light not green) |
| "The capture never started (no run reached RUNNING)" | the run did not start; nothing to download |
| Twin does not move | not connected, or the source is "Playback" without an open recording, or the twin view is hidden (the frame timer runs only while a twin view is visible); the arm is drawn at 0 when not centred; with the model ghost mode UDP no packets arrive |
| Twin: "3D view not available" | PySide6-Addons missing: `pip install -r desktop/requirements.txt` |
| "Cannot listen on UDP port N" | the Unreal twin or another program uses it; choose another "UDP port to listen on" |
| Twin lags the rig | raise the stream with "Live stream at 250 Hz"; set the FTDI latency timer to 1 ms |
| "Set FURUTA_MATLAB to matlab.exe" | MATLAB is only needed for the speed-model fit; set the variable or ignore |
| "Wait for the current task or cancel it" | one background task at a time; wait or press "Cancel" |
| Something crashed silently | read `data/logs/lab-*.log` |
