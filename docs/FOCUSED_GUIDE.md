See [FURUTA_LAB.md](FURUTA_LAB.md) for the complete manual of the desktop application.

# Furuta Lab: start with the measurements

## Launch

Double-click `Start-Furuta-Lab.cmd`, or run `Start-Furuta-Lab.ps1` in the
repository. The local Python environment is
already installed on this PC. On a new PC, run `Setup-Furuta-Lab.ps1` first
with Python 3.12. Select COM11 and **Connect USB**. Keep **12 V connected**
unchecked while the DC supply is unplugged. The motor-run buttons stay disabled.

This frontend uses Qt widgets and raster plots, with OpenGL disabled. Its
serial/heartbeat thread is separate from the GUI and the analysis worker.
MATLAB does not start for live sensors, noise spectra or filter comparisons.
It starts in a separate process only when you request MATLAB model validation.

## First session: USB only

1. Connect USB and watch Live sensors. Choose current millivolts, encoder
   counts, or raw pendulum degrees. Only one plot is continuously redrawn.
2. Put the arm at center and click **Arm is centered here**. It records the
   current count; it does not move. This reference is lost on MCU reset.
3. Let the pendulum hang still and click **Pendulum is hanging down here**.
4. Click **Zero current** with the motor disabled. Keep it still for one
   second. Current calibration affects amperes; raw millivolts remain unchanged.
5. In Still noise & filters, confirm that both joints are still. Record 5 s.
   Wait for recording, then download, to finish. No motor command is sent.
6. Choose each sensor and click **Plot noise spectrum + statistics**. Use
   **Plot captured signal over time** to inspect jumps, drift or clipping.

Still captures save automatically under `data/captures`. Each folder has
raw arrays, configuration/metadata JSON and a MATLAB-readable `capture.mat`.
Keep the folder together. Open saved still capture reloads it without hardware.

Current ADC capture offers 500, 1000 or 2000 samples/s; mechanical capture
targets 500/s. Start at 500 for low-frequency filter work: it gives the MCU
more scheduling margin and often longer uninterrupted records. Use 2000
when you need bandwidth above 250 Hz. The chosen ADC rate/filter bypass is
temporary during passive capture; the prior ADC rate and filter are restored
afterward, including after a canceled download once finite capture finishes.
The live display stream is deliberately 100/s. Real timestamps determine the
actual rates. A weak magnet warning is retained; it qualifies angle accuracy
even when the sensor changes smoothly. Fixing it is a mechanical task.

## Read the spectrum without guessing a filter

SD measures stationary variation in the chosen signal's units. Peak-to-peak
is sensitive to isolated spikes. Drift compares the beginning and end.
PSD is noise power per Hz; narrow peaks may be periodic interference.
The report includes sample rate, invalid/held samples, gaps, within-block
jitter and PSD resolution. The estimator uses valid contiguous blocks and
interpolates within a block only; it does not bridge missing samples. Short
or invalid blocks are excluded. Raw recordings are never rewritten.

A constant motor encoder may have zero stationary noise but still have
quantization during motion. Still PSD alone does not tell us how much useful
motion bandwidth must be kept. It also cannot validate a feedback filter.
A 2 kHz capture cannot resolve a 20 kHz PWM carrier; high-frequency signals
may alias. Use an oscilloscope for carrier-frequency investigation.

### Compare cutoff frequencies

Enter **Keep motion up to (Hz)** and **Allowed filter delay (ms)**. These are
requirements, not quantities inferred from still noise. For example, if you
want to inspect motion up to 5 Hz, enter 5 as a trial requirement, not as a
claim that 5 Hz is the correct control bandwidth.

Compare low-pass cutoffs evaluates causal second-order Butterworth filters.
A candidate passes if attenuation at your required bandwidth is at most
1 dB and the maximum delay from DC through that band fits your delay budget.
Acceptable candidates are ordered by predicted remaining stationary noise.
If none passes, revise the delay/noise/motion tradeoff or test more frequencies.
If PSD bins are too coarse for your required band, the app refuses a cutoff
recommendation. Record longer or use a lower ADC rate to reduce short-block
fragmentation. Zero padding cannot repair missing low-frequency resolution.
Plot frequency up to changes the visible frequency range; recompute/preview
after editing it. It does not discard higher-frequency data from the report.

Preview shows raw and predicted filtered PSD. **Use selected cutoff in live
preview** adds an orange filtered trace without changing raw data. The cutoff
must fit the slower live stream's Nyquist frequency to be shown there.
Mechanical filters are host previews. **Deploy selected CURRENT filter**
changes the MCU's current filter at its nominal ADC rate; raw current remains
available and MCU filtered current is a separate live signal. Deployment is
disarmed and temporary until explicitly saved to NVS.

Check a real moving response before accepting a filter for control. Good
noise suppression can still introduce unacceptable delay or erase dynamics.

## Encoder scale: 34:1 is provisional

Setup begins with reduction 34, 11 motor encoder cycles/revolution and x4
quadrature. If those are correct, the output scale is 1496 counts/revolution.
Click Apply proposed encoder scale only when you want that provisional scale
in RAM. It is not marked verified automatically.

For a real check, record the count change over an independently measured
output rotation. Enter both signed values in Known-displacement scale check.
Use measured scale computes counts/output revolution and direction. Repeat
both ways to assess backlash. Full-assembly physical estimation requires a
recording marked with verified scale. Motor-only shaft speed uses motor
counts/revolution directly; it does not need the uncertain gearbox ratio.

## Determine safe mechanical travel without motor power

With 12 V physically disconnected, center the arm. If it turns freely, move
it by hand to a safe left endpoint within cable slack and mark it, then do
the same on the right. Do not force the gearbox or use powered hard-stop seeking.

The app proposes symmetric travel inside the closer endpoint, with a 10%
margin. Apply these limits to RAM commissions that conservative count limit.
These are operator-marked safe travel boundaries, not measured collision
stops or rated speed/torque/thermal limits. Preserve raw endpoint counts in
recording metadata. Re-center after every MCU reset.

## Powered actuation and dead zone — a later session

Reconnect 12 V only when the setup is ready. Check the 12 V declaration in
the app, select the actual physical assembly, set finite maximum duty,
duration and travel, and apply them. Zero duty/travel defaults prevent arming.
Current trip (A) and speed trip (counts/s) are also explicit settings; 0 means
OFF. Current protection requires a trustworthy zero/gain and sensible margin
above measured noise. The firmware's configured current-trip persistence is
retained. The app does not certify these thresholds from an idle recording.
The checkbox is an operator declaration; there is no supply-voltage detector.

Run & record starts a bounded hold, sine, chirp or seeded PRBS waveform.
Duty fields are percent: 3 means 3%, not 3 V. All waveforms include a half-
second baseline and tail, which count toward maximum duration. Requested duty
can differ from applied duty due to slew/protection. STOP disables output;
the mechanism may coast. Firmware independently enforces travel, finite run
duration, heartbeat and configured current/velocity protections.

Breakaway uses short alternating positive/negative pulses with rest between
pulses. Start with a small commissioned range. The firmware permits at most
16 segments; this workflow supports at most three tested duty levels per run.
For a wider search, collect separate runs with suitable increments rather
than one unbounded sweep. After the run, Analyze recorded breakaway pulses.

A pulse is usable only if the baseline was stationary, encoder data is valid,
and no limit flag intervened. Movement requires your explicit count threshold.
Results use recorded applied duty. A missing positive/negative threshold means
**not found in the tested range**, not zero dead zone. This is breakaway under
the recorded load and supply, not sustaining friction or a universal motor
constant. Repeat with the arm/pendulum removed and attached as needed.

## Parameter estimation (assembled rig)

Everything is done with the arm and pendulum attached. Only the motor supply
`Vs` (12 V) and the encoder scale (gear × 11 × 4) are entered; no masses,
lengths or gearbox efficiency. The result is a model for controller design,
stability analysis, simulation and a digital twin, checked by driving it with
a recorded duty and overlaying it on the hardware signals.

The model has ten numbers: `Rm`, `Kb` and eight values divided by `Kτ`
(`Ja`, `Jp`, `h`, `G = m·g·Lp`, `Beq`, `b1`, `τc+`, `τc−`), i.e. torques are
expressed in amperes of motor current. The linear models, poles, transfer
functions and the simulation depend only on these. `Larm` comes out of the
data. SI values (kg·m², N·m) need one optional reference (Step 9); `m` and `Lp`
separately cannot be obtained with these sensors and are not needed.

Work through the numbered steps of the Parameter estimation tab. Every fit step
shows the signals it used (grey bands = samples left out), the fitted equation
against the measurement, a table with approximate uncertainties and checks that
name the remedy. Results are kept in a session (`data/estimation/<name>/
session.json`); refitting a step marks the later ones "stale".

0. **Setup.** Check `Vs`. Press *Apply estimation recording settings*
   (250 Hz stream, 40 Hz Butterworth filter on the MCU current; the overcurrent
   trip uses the filtered current). Set the references (arm center, pendulum
   down, current zero) and the encoder scale. *Restore normal settings* puts
   the previous stream rate and filter back.
1. **Free swing, motor off.** Hold the arm, release the pendulum 30–40° from hanging (not from upright),
   *Record 20 s*, *Fit free swing*: natural frequency, damping, `Jp/G`, `b1/G`.
2. **Square-wave runs, pendulum hanging (moves the motor).** Run A and Run B
   (fit), then Validation. The app predicts the arm swing once `Kb` is known
   and warns if it exceeds the commissioned travel or if "Maximum run seconds"
   / "Maximum duty" are too small; it does not block. Finished runs are added
   to the lists automatically.
3. **Electrical fit** `i = V/Rm − (Kb/Rm)·θ̇0`, driven samples only (at zero
   duty the driver is in standby and the motor is open-circuit).
4. **Pendulum-equation fit** → `h/G`, `Jp/G`, `b1/G` and `Larm = g·h/G`;
   compared with the free swing.
5. **Arm-equation fit** → `Ja`, `G`, `Beq`, `τc+`, `τc−` per `Kτ`, moving
   samples only. The least-squares model is now complete.
6. **Refine by simulation.** Adjusts `Ja`, `Beq`, `τc±` so that 4 s simulated
   windows match the recordings (10–30 s). Accept or keep the least-squares model.
7. **Model against hardware.** Choose a recording (the validation run, or any
   later run, including closed-loop runs) and compare. Fit % =
   100·(1 − ‖y − ŷ‖/‖y − ȳ‖). In open loop the arm angle drifts slowly; a
   re-sync interval shows the short-term agreement.
8. **Model.** Upright and hanging state-space models (volts and duty input),
   poles, transfer functions θ0/V and θ1/V, discrete model at `Ts`, the
   nonlinear equations with numbers, and a pole-zero plot. *Export model files*
   writes `parameters.json`, `furuta_model.m` (`ss`, `tf`, `c2d`),
   `furuta_rhs.m` (nonlinear model for `ode45`/Simulink) and `parameters.tex`.
9. **Physical units (optional).** Nominal (`Kτ = Kb`), a known added mass
   (second free swing), a weighed pendulum, or an assumed `ηg`.

Recordings made earlier can be added with *Add folder…*. A recording without an
encoder scale in its profile uses the Setup tab's gear × cycles × quadrature.

### MATLAB backend (Extras)

Fit / validate speed model in MATLAB starts a headless worker using PT1 or
tfest on the open recording, with the arm (output-shaft) speed and the applied
duty. It saves the MATLAB model to `data/matlab-jobs/.../model.mat`. MATLAB
startup may take tens of seconds; live acquisition and STOP stay independent.
Set FURUTA_MATLAB to matlab.exe if MATLAB is installed elsewhere.

## Digital twin tab

The rig in 3D (solid), posed from the newest stream record or from the open recording (section 1 · Source), with the
identified model as a translucent cyan ghost (section 2, re-synced to the rig every 2 s). Section 3 forwards the rig
and the cyan ghost to the Unreal twin (UDP 7400). In section 2, **Extra ghosts** adds up to three more, side by side,
each switchable on its own (off by default; a HUD line each; drawn only, not forwarded):

| Color | Ghost | Shows |
|---|---|---|
| orange | model re-synced every 0.2 s (`twin_core.FastResyncGhost`) | the model's short-horizon error, also while balancing (the cyan ghost falls off the unstable upright between its 2 s re-syncs) |
| green | the controller's observer (`twin_core.ObserverGhost`) | the estimate the onboard controller acts on (12 ms lag compensated) |
| violet | closed loop (`twin_core.ClosedLoopGhost`) | the model with its own copy of the onboard controller (LQR or PID, chosen in the group; re-sync every N s) |

## Saving and fault recovery

Run recordings are under `data/runs`, with a manifest and typed sample batches.
Still captures have NPZ, JSON and MAT copies. Export latest analysis JSON
preserves numerical results, assumptions and residuals. Save current profile
to NVS is a separate explicit operation; a center reference is never restored
as an absolute position at reboot.

If the port is busy, close MATLAB/PlatformIO serial monitors before connecting.
If a test faults, inspect device/link details, resolve the cause and only then
Clear a resolved fault. STOP stays available during background jobs. Closing
the app stops the drive and releases the port. No powered test is validated
merely because USB-only acquisition succeeds.
