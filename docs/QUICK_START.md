# Furuta Workbench: practical operating guide

## 1. Open and connect

In MATLAB R2023b, change the Current Folder to this repository, then run:

```matlab
addpath('matlab');
app = FurutaWorkbench();
```

Use a fresh app after updating these files. Close the previous app first; it
owns the serial port. Only one app or serial monitor can use COM11 at a time.
Select Hardware, choose COM11 (check Refresh ports if Windows changes it),
leave baud at 460800, then Connect. Wait for HARDWARE / DISARMED.
Connecting never commands motion. STOP / DISABLE disables the driver.

The first window creation can take several seconds in R2023b. Once connected,
the plots refresh at up to 5 Hz; sensor acquisition and file recording run
independently. Pause view freezes the display, not acquisition or the motor.
Keep MATLAB available during a powered experiment: the device stops if an
armed session loses its heartbeat. Closing the app sends STOP and releases
the port. A native MATLAB crash leaves the MCU heartbeat timeout responsible
for disabling an armed drive.

To learn without hardware, choose Simulator -> Connect -> Simulator setup.
Its example scale and limits are synthetic. Hardware never uses those values.

## 2. What the top controls mean

| Control | Meaning |
| --- | --- |
| Hardware / Simulator / Replay | Real COM port, synthetic system, or saved data. Changing mode disconnects the old session. |
| Connect / Disconnect | Open or release the selected connection. |
| Arm | Enable permission for a validated finite command. It does not start the sequence. |
| Upload sequence | Send the current experiment and validate it on the MCU. |
| Run sequence | Start the uploaded experiment after arming. |
| STOP / DISABLE | Cancel commands, disable drive. A latched fault still needs Clear fault. |
| Live buffer / Loaded run / ADC burst / Mechanical burst | Dataset used by the analysis tabs and selected-data export. |
| Channel | Signal to plot/analyze. CurrentMv is a good first choice before calibration. |
| From / to (s) | Analysis interval measured from the beginning of the selected dataset. Use 0 and 1e9 for all data. |
| Pause view | Freeze the live display; recording and acquisition continue. |
| Record / Finish recording | Start/finalize a passive recording without starting the motor. |

Live & Manual always shows current live telemetry, even when the analysis
source is a saved run or burst. The source selector controls the analysis
tabs. Every spectrum and fit therefore needs the correct source and interval.

## 3. Establish references before interpreting angles or amperes

Do these operations with the driver DISARMED. Clicking a zero button records
a reference; it does not reposition anything.

1. Put the horizontal arm at its physical center. Click Center arm here
   (or Set arm center here in Calibration). Center is lost after MCU reset.
2. Let the pendulum hang still. Click Pendulum down here. The calibrated
   upright angle PendRad is approximately -pi/pi when down and 0 when upright.
   DownRad is 0 when hanging down. PendRaw remains the original 0..4095 value.
3. With no motor drive/current, click Current zero (1 s). Wait for the
   CURRENT_ZERO_UPDATED event. CurrentA then uses
   (CurrentMv - currentZeroMv) / sensitivityMvA. Default sensitivity is
   185 mV/A for the stated ACS712 variant; actual accuracy needs gain checking.
4. Fit the motor encoder scale to enable a physical arm angle. In Calibration,
   enter paired delta counts and independently measured delta arm angles in
   degrees, then Fit counts per output revolution. Use displacements from one
   reference, including both directions if possible. Example only: measured
   +125 counts over +30 degrees and -124 counts over -30 degrees. Do not copy
   this example as your hardware scale. Unused rows may remain 0,0.

countsPerRev is quadrature counts per complete OUTPUT arm revolution,
including gearbox ratio. The motor's quoted PPR alone is insufficient.
armSign comes from the fitted direction; pendSign is the pendulum convention.

### Why the animation may hold the arm still

The arm needs BOTH a center reference and a finite positive countsPerRev.
Until then, its drawn angle stays at center and the label explains why. Raw
Motor count still shows encoder motion. No output angle is guessed.

The pendulum can animate independently. Before a down reference is set, the
app labels it a relative preview: the first valid reading is drawn down.
After calibration it uses the actual hanging-down reference. An invalid/stale
sensor hides the pendulum rod. A weak/strong magnetic warning is shown even
when angles remain readable. The drawing uses unit lengths; it is a sensor
schematic, not a fitted geometric model.

Save profile to device NVS only after checking references/scale/filter values.
Apply profile to RAM is temporary; Save to device NVS is explicit persistence.
Saving never restores the incremental arm center after reboot.

## 4. Inspect current noise first, with no commanded motion

1. On Live & Manual select CurrentMv. Look for clipping, drift or jumps.
2. Click Record, wait 10-30 seconds, then Finish recording. The bottom bar
   shows the recording directory under data/runs. Retain raw millivolts even
   before current calibration.
3. Open Runs & Settings. Set Raw current capture (s), initially 1-2 seconds.
   Click Capture passive burst. Wait for Active: 0 and nonzero sample counts.
4. Click Download completed burst. This fetches raw ADC samples and the
   simultaneous mechanical samples. The analysis source becomes ADC burst.
   Use Save burst MAT to preserve it.
5. Open Time Analysis and compute statistics/plot. Then Spectrum & Noise:
   choose Welch PSD, keep Remove mean checked, select a segment length that
   fits the record (e.g. 512 or 1024), and Analyze.
6. For a band such as 0-50 Hz, enter the band edges and click Band RMS after
   computing the PSD. Compare like-for-like captures and supply conditions.

The live stream is 50, 100 or 250 Hz, selected by streamHz in the profile.
The ADC burst targets 2000 samples/s; the mechanical burst targets 500/s.
Actual timestamps determine sample rate and gaps/jitter. A 250 Hz live trace
cannot resolve a 20 kHz PWM carrier, nor can a 2 kHz ADC burst. Peaks may be
aliases; neither dataset substitutes for an oscilloscope at PWM frequencies.

FFT amplitude is a frequency amplitude view. Welch PSD is power per Hz;
ASD is amplitude per square-root Hz. A spectrogram shows change over time.
Remove mean removes the DC level for noise inspection. Increasing Nfft with
zero padding gives a denser plotted frequency grid, not new measured detail.
More samples and longer Welch segments improve frequency resolution.

If the app refuses a spectrum/filter because sampling is irregular, inspect
gaps and timing first. Resample small jitter is an explicit analysis option,
not permission to hide large gaps. Retain the raw data and timing warning.
Inspect timing displays the selected interval's sample rate, gap count and
jitter. Use contiguous span sets From/to to the longest valid measured span
inside your current interval. Then Analyze or Design + compare again. It
does not fill gaps or change the raw file. Set From/to back to 0/1e9 to inspect
the entire dataset again. A shorter span has less frequency resolution.
FRF + coherence needs an exciting Applied command and measured output;
it is not meaningful for an idle, constant-zero input recording.

## 5. Compare and deploy a filter

First select the source, channel and interval. In Filters select:

| Type | Parameters / use |
| --- | --- |
| bypass | Unchanged signal; reference case. |
| ema | Low-pass cutoff in Hz. Simple smoothing with lag. |
| moving | Window length in samples. Larger means more smoothing and delay. |
| butter | Second-order low-pass, cutoff in Hz. |
| notch | Center frequency in Hz and Q in the window/Q field. |

Click Design + compare. Inspect both the raw/filtered traces and gain/delay or
gain/phase. Raw / filtered PSD compares noise attenuation. Smoothing a trace
does not prove the filter is suitable for feedback; delay matters.

Offline zero phase uses future samples and cannot run on the MCU. Display
filter in Live is a preview and only applies when the design sample rate
matches that live stream. A filter designed on a 2 kHz ADC burst cannot be
displayed unchanged on a 250 Hz live stream.

Deploy causal filter to MCU requires DISARMED, a causal design, and a design
sample rate matching the MCU ADC rate (2000 Hz). Deployment changes the
FilteredMv channel; raw ADC/current telemetry remains available. It is not
saved across resets unless you explicitly Save to device NVS.

For this deployment workflow, select ADC burst, exclude gaps with Use
contiguous span, then in Filters set Design rate Hz to 2000 and check Resample
small jitter before Design + compare. Actual ADC timestamps may imply e.g.
2004 Hz; this explicit option resamples the comparison to the nominal 2000 Hz
used by the device coefficients. It accepts only a nearby rate (within 5%)
and never bridges missing samples. Leave Design rate at 0 for a comparison
using the measured data rate. The report retains the original timing quality.

## 6. Commission bounded motion and use manual commands

Read docs/HARDWARE.md for the actual current-sensor input and driver limits.
The default profile intentionally has maxDuty=0 and travelCounts=0. You must
choose limits for your mechanism before ARM can succeed. No maximum duty,
travel or current trip is automatically certified by the app.

In Runs & Settings, edit the profile table, then Apply profile to RAM. Read
profile to confirm the device values. The most important fields are:

| Field | Units / interpretation |
| --- | --- |
| maxDuty | Fraction 0..1: 0.05 means 5%. A maximum, not a command. |
| slewPerSecond | Maximum change in normalized duty per second. |
| maxDuration | Maximum finite experiment duration in seconds. |
| travelCounts | Allowed absolute encoder displacement from the arm center. |
| velocityCountsPerSecond | Speed trip in raw counts/s; 0 disables this trip. |
| currentTripA | Estimated current trip; 0 disables this trip. Requires a credible current zero/gain. |
| currentTripMs | Current-trip persistence in milliseconds. |
| assumedBusV | Assumed motor voltage used for interpretation/models, not a voltage measurement. |
| heartbeatMs | Device connection timeout, default 300 ms. |
| streamHz | Live display/record stream rate: 50, 100 or 250 Hz. Raw burst rates are separate. |
| adcHz / pwmHz | Acquisition/carrier configuration; use the supported hardware settings. |

Start with a deliberately small finite command within commissioned limits.
On Live & Manual, Signed duty (%) is percent: +3 means +3%, -3 reverses
direction, 0 requests disabled output. Duration (s) bounds the command.
Apply timed command uploads, arms and starts that finite command in one
operator action. This button CAN MOVE THE ARM. It differs from the separate
Arm button, which does not start a sequence. STOP cancels the command.

If no motion occurs, inspect the event log, firmware state, maxDuty, travel
reference, fault and requested/applied plots. Slew limiting can make a very
short command reach much less than its requested amplitude. Do not increase
limits blindly to overcome a reported fault.

## 7. Build repeatable experiments

In Experiments select a waveform, amplitude/offset (%), duration and frequency.
Replace sequence starts a new sequence; Append segment adds another part.
Preview shows requested and limited duty before upload. Pre/post baseline,
repeats and rest apply to the assembled experiment. The segment table/JSON
shows the exact requested sequence. Advanced JSON is data, never executable
MATLAB code.

| Waveform | Typical purpose |
| --- | --- |
| hold / step | Timed constant input and step response. |
| pulse / doublet | Short positive/negative response checks. |
| ramp / triangle | Slow deadband/friction/hysteresis inspection. |
| sine / square | Response at one frequency; square includes harmonics. |
| chirp | Sweep through frequencies over a finite duration. |
| multisine / PRBS | Broadband excitation for identification. |
| table | Imported finite command samples with specified interpolation. |

Use Upload sequence -> Arm -> Run sequence, or Record + run to combine those
actions and save data. All can be rejected by the MCU if the sequence exceeds
the profile limits. Firmware schedules the waveform; MATLAB does not stream
individual PWM updates. Requested and Applied can differ because of duty,
slew, reversal or protection limits. Use Applied for identification.

Prepare burst with next Run captures at the next experiment start. Choose a
capture duration long enough for excitation and response tail. After the run
disarms, Download completed burst, then save it before another capture.

## 8. Estimate parameters and identify a model

Always retain one experiment for fitting and a different portion/experiment
for validation. Choose a source, channel and useful time interval, excluding
unwanted handling transients. Analysis runs while DISARMED.

- Motor response: excite within limits, use CountVelocity first if arm scale
  is unknown; use ArmVelocity after scale calibration. Fit a low-order ARX or
  motor speed model, then inspect measured/predicted output and residuals.
- Pendulum decay: keep the ARM PHYSICALLY FIXED, displace the pendulum slightly
  and release without driving. Use calibrated DownRad and the decay method
  for natural frequency and damping. An unpowered gearbox is not proof the
  arm is fixed. Use small oscillations and avoid handling samples.
- FRF/coherence: use sufficiently broad excitation and a synchronized Applied
  input/output dataset. Poor coherence can mean noise, poor excitation,
  nonlinear behavior or timing problems. It does not identify the cause alone.
- ARX needs orders and input delay in samples. Training fraction separates
  chronological training and validation blocks. Fit percentage alone is not
  a physical-validity test; check stable poles, delay, residuals and new data.
- tfest/n4sid/ssest use System Identification Toolbox when available. View
  model displays available model responses and residual diagnostics.
- Physical nonlinear fitting needs measured geometry, inertia/electrical
  assumptions, initial state, a small selected free-parameter set and bounds.
  Use SI units. Several parameters may be inseparable from the available data.
  It does not automatically discover every physical constant from one run.

Export model keeps fit details/metadata; parameter registry exports record
sources and quality. No fit is deployed as a balancing controller.

## 9. Save, replay and troubleshoot

Recording directories contain configuration/experiment/manifest JSON,
events.jsonl, original raw_capture.bin and typed samples.bin batches.
Finish recording finalizes the manifest. Keep the whole directory together.
Open saved run selects offline data; Replay saved run feeds its timeline
through the live viewer without a hardware output path. Export data/Save
selected CSV use the selected source/interval; MAT preserves MATLAB types.

| Symptom | Check/action |
| --- | --- |
| Port absent/busy | Close other apps/PlatformIO serial monitor; Refresh ports; check COM USB cable. |
| Connected but no angles/current A | Raw channels work before calibration. Check center, output counts/rev, pendulum down and current zero. |
| Animation arm fixed | Read the message below it. Center alone does not determine encoder scale. |
| Weak magnet warning | Data may still change, but angle accuracy remains qualified; inspect centering/gap mechanically when accessible. |
| FAULT | Inspect Runs & Settings log. STOP leaves faults latched. Resolve the cause before Clear fault and rearming. |
| Window lags | Pause view or lower streamHz to 100/50 while disarmed; finish recording before heavy analysis; use the current app code. |
| Connection error / backlog | Drive is disabled; close/reconnect and inspect the saved events. Re-center after any MCU reset. |
| Spectrum/filter rejects data | Check selected source, interval, sensor-valid flags, missing samples and timestamp jitter. |
| Current looks noisy | Examine raw millivolts and a passive ADC burst; record SD/drift/band RMS. Do not hide it with filtering before measuring. |

For implementation details see USER_GUIDE.md and ../protocol/SPEC.md.
Guide / help in Runs & Settings opens this document inside MATLAB.
