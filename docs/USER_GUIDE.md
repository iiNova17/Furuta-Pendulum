# Operating the workbench

Launch with addpath('matlab'); app = FurutaWorkbench(). It starts disconnected.
The mode selector makes Hardware, Simulator and Replay visibly different.
Switching mode requires disconnecting the current session. Port refresh only
enumerates ports; Connect performs the firmware handshake.

For a step-by-step introduction and a control-by-control reference, start with
[QUICK_START.md](QUICK_START.md). Guide / help in Runs & Settings opens it.

## Settings and zeros

The hardware profile begins with maxDuty=0 and travelCounts=0, preventing
arming until explicitly commissioned. In Runs & Settings, enter finite limits
and click Apply profile to RAM. maxDuty is a normalized fraction, not percent.
Manual/experiment amplitude fields display percent. Current trip 0 means OFF.
The current circuit's electrical restrictions are in HARDWARE.md.

Set arm center at the physical center. This only records current count; it
does not move the arm. A MCU reset loses this reference. Persistent counts per
revolution and signs do not restore absolute position.

Set Hanging Down or Upright while holding the pendulum still. The GUI uses a
circular mean of recent valid readings, including across 0/4096. Hanging-down
reference makes model angle pi (equivalent to -pi); model zero is upright.
DownRad is a separate local angle around the hanging position. Weak-field
calibration stays provisional; no AS5600 OTP is programmed.

Current zero collects one second while the driver is disabled. The MCU rejects
clipping or implausible mean values. It does not use the old USB diagnostic's
30 mV SD threshold as an automatic failure criterion. The event log records
sample mean/SD/count. Inspect noise and drift in a raw ADC burst before
deciding whether zero/gain/current-trip accuracy is adequate.

Encoder scale fitting needs actual count changes and independently known
output angles in degrees, preferably both directions. Known-current gain
fitting needs externally known signed current points and mean mV. The app
cannot invent those references from motor duty or stall current.

All profile/zero/filter changes are disarmed operations. Readback confirms
the applied profile. Save to device NVS is a separate explicit action.
Import profile loads the editor only. It does not arm, move or restore position.

## Experiments

Choose a waveform and edit amplitude/offset, duration and frequency. Replace
or append segments. Use advanced segment JSON for phase, pulse duty, ramp
endpoint, logarithmic chirp, PRBS bit period/seed, multisine vectors or table
interpolation. The JSON is data; arbitrary code is never evaluated.

Pre/post baseline and repeats/rest apply to the complete execution sequence.
Preview shows requested and amplitude/slew/reversal-limited commands.
The firmware independently validates total duration, segment count and limits.

Upload and validate -> Arm -> Run is the explicit sequence. Record + run
combines those actions after the operator clicks it, and finalizes recording
after completion/post-roll. Manual finite commands use the same upload/arm/run
path and limit enforcement. Editing a field, moving a slider or changing tabs
does not move the motor.

Stop disables output and cancels queued starts. A latched fault requires
Clear fault before rearming. Stop does not promise zero mechanical velocity.
A missed heartbeat stops an armed MCU. Heavy offline analysis is restricted
to disarmed state so GUI computation cannot silently own an unattended run.

## Recording, raw bursts and replay

Record captures full received core samples independently of plot refresh.
Live buffer holds the latest 60000 samples; saved recordings retain the whole
received session. The live stream is selected from the MCU's core acquisition;
download a mechanical burst for its complete higher-rate capture.

Capture passive burst records with the motor disabled. Prepare burst with
next Run starts capture with the next experiment. Its duration is independent
of waveform length, so include the desired response tail in capture duration.
Firmware preallocates finite ADC/mechanical buffers and rejects oversized runs.

After capture completes/disarms, Download completed burst fetches both channels.
ADC burst selects individual raw current samples. Mechanical burst selects
the accompanying raw counts, pendulum readings and applied command.
Save burst MAT preserves data and its capture profile.

Runs use manifest/configuration/experiment JSON, timestamped events, original
wire bytes and length-prefixed sample batches. Samples remain typed, including
int64 count and uint64 timestamps. Open saved run gives offline analysis;
Replay feeds the same viewer without a hardware output path.
Export selected data supports MAT v7.3 or CSV plus a metadata sidecar.

Configuration changes are events, not edits to raw data. For comparisons,
prefer one stable configuration per recording. Profile IDs remain in every
sample; do not mix filter/zero/sign changes into one identification interval.

## Analysis

The top source, channel and From/to controls select the analysis dataset.
Time Analysis provides statistics, autocorrelation, overlays and response
metrics with explicit baseline/target. Derivative channels have an undefined
first sample; start the selected interval after it.

Spectrum & Noise provides FFT, PSD, ASD, spectrogram and input/output FRF with
coherence. Segment length is in samples; FFT length 0 selects the natural
length. Band fields are Hz. Mean removal and Hann/rectangular windows are
explicit. Resample small jitter is opt-in; gaps and invalid/held sensor
samples must be handled by selecting valid contiguous data.

FFT amplitude and PSD density have different normalization/units. Zero-padding
does not increase physical frequency resolution. Coherence needs at least two
averaged segments. Spectra use device timestamps rather than GUI arrival time.
At 2 ksample/s, Nyquist is 1 kHz; PWM carrier/ripple near 20 kHz needs another
validated sampling path or suitable scope.

Filters offers causal bypass, EMA, moving average, Butterworth and notch.
Cutoff is Hz; the other numeric field is moving window length or notch Q.
Select gain/delay, gain/phase or raw/filtered PSD. Offline zero phase requires
the signal toolbox and cannot be deployed. Deployment requires design at the
current ADC rate, a disarmed device, and supported causal coefficients.
Live display filtering only applies when its design rate matches the display
data rate. Raw acquisition and fast protection remain independent.

## Identification

Applied duty is the measured command input. Choose output and interval,
training fraction, ARX orders/delay or toolbox model order. Training and
validation are chronological blocks. A motor-speed fit requires ArmVelocity
or CountVelocity; pendulum decay requires DownRad and a physically fixed arm.
It does not assume that an unpowered geared motor fixes the arm.

Inspect measured/model traces, held-out error, residual correlations,
conditioning, poles and stability. View model offers linear Bode/step and
residual plots. Model export retains the estimator object and metadata.

Physical nonlinear fitting requires measured/fixed parameters in its JSON,
free parameter names and explicit bounds. All values use SI units.
Outputs must include calibrated arm angle, upright pendulum angle and current.
The model neglects pendulum axial inertia and uses tangential geometry.
It fits selected identifiable parameters, not every constant automatically.
Geometry, current gain and estimated duty-to-voltage conversion affect the
result. Uncertainty/identifiability warnings remain part of the result.

The parameter registry labels source/quality. Export fitted models to MAT and
registry values to JSON. No estimated parameter is silently deployed as a
motor controller.
