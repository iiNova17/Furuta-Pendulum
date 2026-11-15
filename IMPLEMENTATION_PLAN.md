# Furuta experiment workbench: implementation plan and model handoff

Prepared 21 October 2026 for implementation by another model.
Target repository: D:/Study/Senior/Control 2/Furuta Pendulum Project/Repo/Furuta Pendulum

## 1. Outcome and scope

Build a complete, runnable desktop experiment workbench consisting of:

1. A PlatformIO firmware project for the ESP32-S3.
2. A MATLAB app with a single public .m entry point and modular supporting .m files.
3. A documented, versioned USB serial protocol connecting them.
4. Recording, replay, calibration, waveform experiments, signal/noise analysis,
   filter comparison/deployment, system identification and parameter-estimation workflows.
5. A simulator and automated tests that exercise the same app without hardware.

The operator must be able to connect, inspect sensors, establish references,
configure a finite experiment, preview its command, run it, stop it, save it,
inspect time/frequency responses, compare filters, fit models and validate
those models against separate data, entirely from the app.

Deliver working features rather than decorative panels or TODO buttons.
One launcher file does not mean one monolithic file: use a programmatic
uifigure app with a readable class and separate services. No .mlapp binary is
required. Target MATLAB R2023b on Windows.

This phase is a characterization/identification workbench. It provides direct
motor excitation, not an unvalidated arm-position servo, torque source,
automatic swing-up or upright balancing controller. Keep interfaces suitable
for later PID/pole-placement/LQR and real Stateflow integration. Do not claim
this app alone fulfills the course's Stateflow/HIL requirements.

## 2. Current context and evidence boundaries

The latest user report supersedes earlier incomplete USB-test status:

- The user says the hardware tests were completed, the motor moves, its encoder
  counts, and the current sensor reads.
- The pendulum magnetic encoder responds to movement but still reports weak
  magnetic field. The user considers its behavior good.
- Current-sensor waveform inspection, noise analysis, filtering, parameter
  estimation and system identification have not yet been done.
- The user states that the system is presently connected with 12 V ON.
- Electronics are enclosed; test-point access is limited.

Treat successful operation as user-reported. Do not claim calibrated angle or
current accuracy, quantified noise, identified parameters, electrical protection
or validated control gains from these reports. Preserve weak-field diagnostics.
Do not re-import old test failures as the current result; take fresh captures.

At planning time this repository contains README.md, LICENSE and .gitattributes
and is otherwise essentially empty. Reinspect it before implementation because
the user or another model may add files. Preserve existing work and license.

MATLAB R2023b was found at D:/MATLAB/R2023b. Directories for Signal Processing,
System Identification, Control System, Optimization, DSP, Simulink and Stateflow
were present; runtime availability/licensing was not checked. Detect it at app
startup rather than assuming directory presence guarantees a usable toolbox.

This handoff is a software implementation request. Build and test against
simulation first. Do not auto-connect to hardware on launch, auto-upload,
reset the attached device, auto-arm, or execute a motion demonstration.
The connected board may be running different firmware by implementation time.
Coordinate flashing with the operator: first disconnect motor power, upload,
verify boot-disabled behavior, then restore power for operator-started tests.

## 3. Fixed hardware interface

| Item | Current configuration |
| --- | --- |
| MCU | ESP32-S3 N16R8; 16 MB flash, 8 MB octal PSRAM |
| Serial connector | COM USB-to-UART, UART0; do not consume GPIO43/44 |
| Native USB | Reserve GPIO19/20 |
| Motor driver | TB6612FNG channel A |
| Current sensor | ACS712 5 A, 5 V supply, nominal 185 mV/A |
| Motor encoder | A/B quadrature, reported 11 cycles/channel/motor rev; x4 interpretation needs confirmation |
| Pendulum encoder | AS5600, I2C address 0x36, DIR grounded |
| Motor supply | User's 12 V adapter; no bus-voltage measurement channel is installed |
| Unknowns | Gear ratio, actual encoder CPR, motor constants, geometry/inertias, friction, backlash |

| GPIO | Function |
| --- | --- |
| 4 | Current-sensor analog output, ADC1 |
| 5 / 6 | Motor encoder A / B |
| 8 / 9 | Pendulum SDA / SCL |
| 15 | Driver PWMA |
| 16 / 17 | Driver AIN1 / AIN2 |
| 18 | Driver STBY; external 10 kohm pull-down |

Driver VCC and encoder logic are 3.3 V. Current-sensor OUT presently connects
directly to GPIO4 without a divider. The current path is driver AO1 -> sensor
IP+ -> IP- -> motor pin 6; driver AO2 -> motor pin 1. Grounds are common.
Neither motor terminal is a ground connection.

Known constraints must be visible in hardware documentation:

- The 5 V current-sensor output is not intrinsically safe for an ESP32 input
  under every current/fault condition. Software attenuation is not protection.
- The reported roughly 2.2 A motor stall current exceeds the driver's stated
  1.2 A average rating. A functioning motor test is not a thermal/current rating.
- The GUI cannot measure VM, supply rails or actual STBY voltage with this wiring.
- A GUI Stop is not a physical emergency disconnect. STBY disables drive; it
  does not guarantee that mechanical motion stops instantly.

Do not silently assume a different driver, protected ADC input, measured bus
voltage or automatic hardware current limiting. No locked-rotor experiment is
part of the automatic experiment library for this hardware.

## 4. Architecture and repository layout

    README.md
    IMPLEMENTATION_PLAN.md
    firmware/
        platformio.ini
        include/              pins, configuration, wire-format definitions
        src/
            main.cpp
            acquisition/      ADC, quadrature, pendulum, timestamping
            actuator/         signed duty and bridge operating modes
            experiments/      waveform generation and finite sequencer
            processing/       calibration and causal filters
            safety/           state machine, deadlines, limits, faults
            transport/        framed protocol, queues, command dispatch
            storage/          versioned NVS and acquisition buffers
        test/                 host-testable protocol/math/state tests
    matlab/
        FurutaWorkbench.m     public launcher: app = FurutaWorkbench()
        +furuta/
            WorkbenchApp.m
            SerialTransport.m
            Protocol.m
            DeviceSession.m
            ExperimentBuilder.m
            RunRecorder.m
            CalibrationService.m
            AnalysisService.m
            FilterService.m
            IdentificationService.m
            SimulatorTransport.m
            ReplayTransport.m
        tests/
        examples/             clearly labelled synthetic data/configs
    protocol/
        SPEC.md
        golden_vectors.json
    config/
        default_profile.json
        schema.json
    docs/
        HARDWARE.md
        USER_GUIDE.md
        EXPERIMENTS.md
        VALIDATION.md
    data/                     ignored real recordings, except small fixtures

Keep the existing LICENSE. Add .gitignore entries for PlatformIO output,
MATLAB caches/autosaves, local paths and large recordings. No machine-specific
COM number or absolute development-tool path in normal configuration.

The firmware owns acquisition timing, waveform timing, output limiting and
communication-loss handling. MATLAB owns user interaction, experiment design,
plots, recording/replay and computationally expensive analysis. Drawing and
USB arrival times must never become the experiment's time base.

Use the previous USB bench project's pin definitions and known-compatible
PlatformIO settings as references, not its printf telemetry or diagnostic
polling loop as the architecture. In particular, do not copy unbounded UART
logging or a blocking I2C timeout into a time-critical actuator task.

## 5. Firmware behavior

### 5.1 Build and scheduling

Start with pinned espressif32 6.12.0 / Arduino 2.0.17, esp32-s3-devkitc-1,
16 MB partitions, qio_opi, PSRAM, and UART0 serial transport. Document any
deliberate platform upgrade and retest its PCNT/ADC/PWM APIs.

Separate bounded tasks for acquisition, actuator/safety, protocol service,
and buffering/transmission. Timer/ISR code only schedules work or performs
bounded peripheral operations; no I2C, heap allocation, Serial.print or
filesystem/NVS writes in an ISR. Use preallocated buffers and explicit
ownership. Log task overruns, sample age, overflow and maximum execution time.

Starting engineering targets, to be measured rather than promised:

| Function | Initial target |
| --- | --- |
| Actuator sequencer / limit checks | 1 kHz |
| Motor count / pendulum acquisition | 500 Hz, individually timestamped |
| Current acquisition | 2 ksample/s continuous candidate; configurable |
| GUI redraw | 10-20 Hz |
| Health/status | 5-10 Hz |
| Heartbeat | 10 Hz, 300 ms initial device timeout |
| PWM carrier | Configurable, initial 20 kHz candidate; record actual frequency/resolution |

Use a monotonic 64-bit microsecond clock. Record actual capture/command-apply
times, sample numbers and missed deadlines. Read pendulum samples in a bounded
worker; if I2C stalls, the safety task must still disable output promptly.
Discover practical bus rate (100/400 kHz) and transaction latency rather than
declaring a new nominal rate to be achieved.

Use robust PCNT overflow extension appropriate for powered motion. Do not use
the old manual-test assumption of fewer than 15000 edges between polls.
Test positive/negative overflow, concurrent reads and reversal. Keep the
monotonic raw count separate from user zero offsets.

### 5.2 Acquisition modes and data quality

Provide two mandatory modes:

1. Live overview: continuous timestamped sensor/actuator frames, chosen to fit
   the serial budget; plotting is decimated independently of recording.
2. High-rate current burst: acquire unaveraged ADC conversions into bounded
   MCU memory, with start/trigger/sample-clock metadata, then download the
   complete burst. Support motor-disabled noise recording first; later support
   a finite waveform plus simultaneous current and mechanical recording.

ADC conversion and millivolt calibration must refer to the same sample.
Use the supported calibrated conversion/DMA path for the chosen framework.
Do not pair two independent analogRead and analogReadMilliVolts conversions
and present them as one measurement. Record attenuation, resolution, ADC
calibration source and sampling mode. DMA buffers must use suitable internal
memory; PSRAM can hold copied recordings, not an assumed DMA-compatible pool.

Suggested initial capture budget: at most 4 MiB from verified available PSRAM,
with an internal-RAM fallback at smaller duration/rate. The app calculates
duration/capacity before arming. Reject oversized runs, stop acquisition or
abort cleanly on overflow according to the selected mode, and report the
event. Never wrap silently over the start of an identification experiment.

Expose per-sensor validity, fresh/held status, timestamp/age and raw data.
AS5600 status, AGC and magnitude accompany angle. Slow health fields need not
be read every angle cycle, but their age must be known.

Weak-field status is a persistent warning, not a reason to hide a responding
sensor. Allow raw logging and exploratory open-loop characterization with
that warning recorded in the run. Missing magnet, failed reads or implausible
jumps are distinct conditions. Do not advertise a weak-field dataset as
calibrated angle truth. Sensor-loss behavior must depend explicitly on which
sensors the running mode needs; powered modes always need a valid arm-limit
channel. No silent continuation using stale safety data.

### 5.3 Actuator semantics

Primary command unit is signed normalized duty in [-1, 1]. Display percent in
the UI, retain normalized duty in data. It is neither torque nor measured
voltage. Optional approximate voltage command is duty = Vrequested / Vbus_assumed;
label Vbus as user-entered and Vmotor as estimated. Disable this conversion if
Vbus is unset or implausible. Actual H-bridge voltage drops and supply sag are
not measured.

Verify and document the TB6612 truth table. Define drive, short-brake, and
standby/coast separately. Do not assume PWM-low is electrically coast.
Use one documented PWM modulation mode initially. Zero command, run completion
and fault default to PWM=0 and STBY LOW; active braking is a separate bounded,
explicitly configured operation if supported.

Pass all manual/waveform commands through the same amplitude, slew, runtime,
arm-travel and sensor-fault logic. Apply direction changes through a
documented zero-output transition; record any blanking interval and slew
limiting. Preserve both requested and actually applied duty, plus reasons for
clipping. Never bypass limits because an experiment requests a square wave.

### 5.4 State machine and failure handling

States: BOOT, DISARMED, ARMED_IDLE, RUNNING, STOPPING, FAULT.
Acquisition/recording state is separate: passive acquisition works DISARMED.

- BOOT, reset, reconnect and firmware errors command STBY LOW.
- Connection performs identification and enters DISARMED, with no implicit
  movement. Restoring a profile or reconnecting never resumes a prior run.
- ARM requires a valid session, valid finite limit configuration, current
  position reference and applicable sensor health. ARM alone does not move.
- START accepts only a fully validated uploaded experiment and transitions at
  a reported device time; stale/duplicate starts must not restart a waveform.
- STOP/DISARM cancels pending starts and all queued output, disables drive,
  invalidates the arm lease and acknowledges completion. Stop does not mean
  the mechanism's velocity is known to be zero.
- Normal completion also disables drive. FAULT latches its reason, and clear
  fault returns only to DISARMED.
- Lost host heartbeat, device watchdog, invalid safety sensor, exceeded travel,
  current/ADC limits or repeated timing overruns disable output on the MCU.

Default output limit is zero until a profile is deliberately commissioned.
Provide editable finite duty, slew, duration, raw-count travel, velocity and
current thresholds with units and readback. Do not invent a universally safe
current/duty limit from the motor's stall figure. A raw-count travel envelope
allows bounded duty tests before gear ratio is known; degree-based limits
become available after counts-per-output-revolution calibration.

Incremental arm position is relative. After power/reset, require re-centering
or a verified homing procedure before powered runs. Persistent configuration
does not restore physical absolute position. Sensor offsets, signs, gearing,
limits and filter changes occur only when disarmed and create new config IDs.
Never allow re-zeroing to move the active safety envelope during a run.

Current trips use an independently bounded, low-latency path; a display filter
cannot suppress a peak trip. Show when current calibration/noise makes an
ampere threshold unreliable. ADC clipping or invalid calibration is not zero
current. Software limits supplement the physical power switch and do not
repair the known driver/ADC hardware limitations.

## 6. USB protocol contract

Write SPEC.md and golden byte vectors before implementing both endpoints.
Use UART0, binary framing and typed messages. Default link candidate is
460800 baud, with a tested 921600 option and a lower-rate fallback. Both sides
must agree explicitly on the baud/rate; do not silently change it mid-run.

Recommended framing: COBS-encoded packet terminated by 0x00. Decoded packet:

| Field | Type |
| --- | --- |
| Protocol version | uint8 |
| Message type | uint8 |
| Flags | uint16 |
| Session ID | uint64 |
| Packet sequence | uint32 |
| Payload byte length | uint16 |
| Payload | Exactly length bytes |
| CRC | CRC-32/ISO-HDLC over header and payload |

All multibyte values little-endian, IEEE-754 floats where specified. State CRC
polynomial/reflection/init/final XOR and include a test vector. Decoded packet
maximum initially 2048 bytes; reject oversize/inconsistent frames before
allocation. Serialize fields explicitly, never send a packed C struct whose
layout MATLAB must guess. Bound queues and parser work under garbage input.
Keep debug strings off the binary channel except typed log messages.

Use bounded UTF-8 JSON payloads for low-rate configuration/commands and binary
arrays for samples/bursts. Reject non-finite values, unknown required fields,
wrong types, excessive nesting and unsupported schema versions.

Mandatory command/message groups:

- HELLO / CAPABILITIES / STATUS: board identity, firmware/build, protocol
  version, boot ID, available rates/buffer capacity and calibration versions.
- SESSION_OPEN / HEARTBEAT: new host-session nonce, lease and negotiated modes.
- GET_CONFIG / SET_CONFIG / CALIBRATE / SAVE_CONFIG: transactional validation,
  device readback and explicit NVS save only while disarmed.
- ARM / DISARM / STOP / CLEAR_FAULT.
- UPLOAD_EXPERIMENT / VALIDATE / START / ABORT / EXPERIMENT_STATUS.
- START_STREAM / STOP_STREAM; acquisition must remain independent of motor arm.
- CAPTURE_BURST / BURST_INFO / READ_CHUNK / chunk verification and retry.
- ACK / NACK / EVENT / TELEMETRY / SAMPLES.

Every command has a command ID; acknowledgements include that ID, result,
reason and device time. A session-local deduplication cache makes retries
idempotent, especially START, ARM and relative operations. Use absolute
parameters instead of ambiguous toggle commands. Boot ID changes invalidate
all pending requests, zeros tied to incremental counts and run state.

STOP is highest priority and idempotent; it bypasses ordinary run/config queues.
A valid STOP can be accepted without the current motion session so recovery
after a host restart remains possible. Replaying ARM/START from an older
session must never create motion.

The host sends a heartbeat every 100 ms while owning an armed session.
Firmware timeout initially 300 ms, measured on-device. A busy MATLAB analysis
callback must not secretly keep an unobserved run alive; process heavy offline
analysis when disarmed, or use a verified background worker while maintaining
responsive transport. Timeout should stop safely even when MATLAB freezes.

Core sample schema must include:

- uint64 device capture time, uint32 sample sequence, boot/run/config IDs
  either in each record or its unambiguous batch header;
- int64 raw motor count, raw pendulum count, pendulum sample age/status;
- raw ADC conversion and matching calibrated mV, current calibration ID;
- requested and applied duty, output mode and actual apply-time relationship;
- validity/freshness, saturation/limit flags and dropped-sample counters.

Derived angles, currents and velocities must be reproducible from the raw
record and configuration. Optional transmitted filtered values must identify
the exact filter/config version. A core sample is a time-aligned record of
asynchronous sensors, not a claim of simultaneous physical sampling.

For ADC bursts, send explicit start time, actual sample period, sample count,
raw values, sequence/overrun status and synchronized actuator-event records.
Preserve ADC/DMA timing resolution and quantify uncertainty when only block
timestamps are available.

Budget bandwidth before choosing rates: 8N1 gives roughly baud/10 bytes per
second, before protocol overhead. Aim below 70% sustained utilization. For
example, a 56-byte core record at 500 Hz already needs 28 kB/s before framing;
460800 baud does not also have unlimited room for an unfiltered ADC stream.
Batch records, reject unsupported simultaneous modes, use local burst storage,
or negotiate a proven higher baud. Never reduce recording rate silently.

## 7. MATLAB app layout and user interaction

Use uifigure, uigridlayout, uitabgroup, uiaxes, tables and persistent line
objects. A clean technical desktop interface should work at 1366x768 and scale
up. Units in labels, readable fonts, keyboard focus, and status text alongside
color. No web frontend, external server or generated visual assets are needed.

Persistent top strip:

- mode selector: Hardware / Simulator / Replay, visually unmistakable;
- port selection/refresh, baud, Connect/Disconnect;
- device/firmware identity, state, sensor health, packet loss and sample age;
- separate Arm and Run, and a prominent Stop/Disable reachable from every tab.

Persistent bottom strip: active run ID, elapsed/remaining time, recording path,
buffer usage, error/event summary and whether outputs are disabled.
Never show green Connected as if it meant safe, calibrated or armed.

| Tab | Required functional content |
| --- | --- |
| Live & Manual | Raw/engineering sensor values; signed duty setpoint, finite timed pulse/jog; configurable plots and a simple measured-angle schematic |
| Experiments | Waveform/sequence editor, limits, preview, pre/post capture, upload/validate/run/abort, progress |
| Calibration | Motor center, encoder scale/sign, pendulum down/upright references, current zero/gain, configuration readback/save |
| Time Analysis | Select runs/channels/intervals, overlay, cursors, statistics, transient response and actuator clipping |
| Spectrum & Noise | FFT, PSD/ASD, spectrogram, coherence/FRF, noise-band statistics and acquisition-quality report |
| Filters | Raw/filtered comparison, filter design, response/phase/group delay, offline apply and supported MCU deployment |
| Identification | Dataset preparation, black-box and physical parameter workflows, estimates/uncertainty and validation |
| Runs & Settings | Session browser, replay, export, hardware/profile/sample/limit settings, event log, toolbox availability |

Manual controls must use an explicit Apply/Run action. Dragging a slider,
editing a numeric field, changing tabs, loading a run, or reconnecting must
not send motion. A timed jog ends automatically on-device. Optional hold-to-jog
requires a device-side renewal lease; mouse-release alone is not a reliable
stop mechanism.

The launcher's normal mode is disconnected. Simulator/replay never transmit
to hardware. Switching mode disconnects/disarms the hardware transport first.
Closing the app attempts STOP/DISARM, stops callbacks/timers, flushes the log
and closes serial resources; MCU timeout covers host failure.

Separate byte reception/parsing, raw recording, analysis and UI refresh.
Serial callbacks only perform bounded ingestion and queue work; do not fit
models, redraw all plots, or block awaiting an ACK inside those callbacks.
Render a limited time window with min/max-preserving display decimation.
Full acquisition data stays in the recording even if UI frames are skipped.
Prevent callback re-entry and unbounded array/table growth.

Use MATLAB serialport and byte callbacks; app-owned request IDs implement
asynchronous responses. Handle partial/multiple frames per callback, garbage
before handshake, disconnects, board resets and callbacks after app close.
R2023b support must be checked against that installation: do not use newer
serial properties merely because they appear in current online docs.

## 8. Calibration and reference conventions

Use radians/rad/s internally; allow degrees/degrees/s on screen. SI units and
conversion metadata travel with every dataset. Maintain both raw and derived
channels. Calibration never overwrites raw samples.

### Arm

Store motor-encoder count offset and sign. Define G as motor rev/output rev.
If 11 PPR means cycles/channel, nominal output counts/rev = 4*11*G; verify,
do not silently assume. Prefer a GUI calibration where the operator marks
known output angular displacement and the app estimates counts/output rev
from several measurements in both directions.

Arm angle = sign * 2*pi*(count - centerCount)/countsPerOutputRev.
Before scale is known, display counts and counts/s; degree-based control and
limits remain unavailable. Preserve an unwrapped arm coordinate for cable
travel limits. Re-centering means recording current position, not moving there.
Saved scale/sign is reusable; a saved incremental count offset is not proof
of the next boot's physical position.

### Pendulum

Keep separate quantities:

- raw absolute sensor angle;
- user-selected bench-relative angle;
- physical model angle theta1, zero upright, wrapped to [-pi, pi);
- unwrap segment and validity, resetting segment continuity after an outage.

Provide explicit Set Hanging Down and Set Upright actions. With rawDown:

    theta1 = wrapToPi(sign * (raw - rawDown) * 2*pi/4096 + pi)
    phiDown = wrapToPi(theta1 - pi)

Implement the wrap locally if the installed toolboxes do not provide it.
Use circular statistics for calibration windows crossing 0/4096, require
stationarity, and show scatter and magnet status. Weak-field calibration may
be saved as provisional with the warning retained; do not burn AS5600 OTP.
Capture sign using a known manual direction. Do not average wrapped angles
linearly or differentiate them before handling wrap.

### Current

Acquire a user-selected stationary, drive-disabled zero window. Report mean,
standard deviation, drift, outliers, clipping and effective sample count.
Offer a documented mean/robust estimator and uncertainty estimate accounting
for correlation. Reject wrong-scale/disconnected/clipped data; distinguish
noisy-but-plausible zero from an impossible offset.

Do not copy the old arbitrary 30 mV diagnostic threshold as a universal sensor
specification, and do not relax thresholds silently to produce a pass.
Record whatever quality policy is used. Noisy zero should remain visibly
qualified; current-trip thresholds must account for the resulting uncertainty.

Current estimate = (voltage - zeroVoltage)/sensitivity. Default sensitivity is
nominal 0.185 V/A, labelled nominal. Provide a multi-point gain/offset wizard
using externally known signed currents when such references are available;
do not infer gain from the motor's nominal/stall current. No current-feedback
controller is implied by displaying this estimate.

All calibration edits are disarmed operations. Apply to RAM with device
readback; Save to Device is a separate explicit NVS write. Store schema,
firmware compatibility, timestamp, sensor identity if available, calibration
quality and hash. Export/import JSON profiles. Invalidate dependent derived
channels/filters appropriately when signs/scales change.

## 9. Waveforms, timed commands and experiment sequences

Implement a finite sequence engine executing on the device clock. Upload and
validate the complete bounded specification before allowing Run. MATLAB
timers are for UI/communication, never for scheduling individual duty edges.

Mandatory waveform families:

| Family | Main parameters |
| --- | --- |
| Constant/step | Signed level, delay, duration, baseline |
| Pulse/doublet/pulse train | Amplitude(s), widths, spacing, count |
| Ramp/triangle | Start/end levels, slopes or periods, repeat count |
| Sine | Amplitude, DC offset, frequency, phase, duration |
| Square | Levels/amplitude, frequency, duty fraction, duration |
| Chirp | Linear/log sweep, positive valid frequency bounds, amplitude, duration |
| Multisine | Frequency list, phases/seed, amplitudes, normalization, duration |
| PRBS | Documented polynomial/order, seed, bit period, levels and finite length |
| Arbitrary samples | Imported finite time/value table, interpolation mode, explicit units |

Specify units, initial value, endpoint behavior and phase continuity at segment
boundaries. Preview requested waveform and the envelope after amplitude/slew
constraints, including transition gaps. Do not describe a slew-limited square
wave as the requested ideal square. Record the applied signal from firmware.

Support segment tables, pre-trigger/baseline and post-run recording, repeats,
rest intervals, abort conditions, notes, and named templates. Repetition is
finite; every run has a maximum device-enforced duration. STOP cancels all
remaining segments. No automatic recentering between runs.

Validate on both host and MCU:

- finite numbers, nonnegative times and positive durations where required;
- strictly ordered sample times and bounded sample/segment counts;
- amplitude including DC offset within configured limits;
- frequency below the supported waveform update/measurement bandwidth;
- log-chirp frequencies strictly positive;
- total run duration, capture capacity, expected serial utilization;
- current configuration ID and required references/sensors available.

Use the actual applied input for identification. Log limit activation,
direction-change gaps and clipping; reject or explicitly segment such regions
for linear-model fits. A symmetric input is not a guarantee of zero final arm
position. Travel limits remain active throughout chirps/PRBS.

Arbitrary waveforms are data, not executable code. Do not evaluate arbitrary
MATLAB expressions or execute instructions embedded in imported files.
Cross-language tests must compare preview samples and MCU waveform samples
at the same device-relative times.

## 10. Recording, replay and time-domain analysis

Give every run an immutable ID. Use a directory containing:

    manifest.json             schema, device, firmware, conditions, units
    configuration.json        hardware, zeros, gains, limits, sample setup
    experiment.json           requested sequence, seed and target timing
    events.jsonl              ACKs, actual starts/stops, faults, edits, warnings
    raw_capture.bin           optionally original wire frames for diagnostics
    samples.mat               chunked v7.3 or documented HDF5 schema
    adc_burst.mat             when a raw high-rate capture is used
    analysis/                 derived results, filter versions, fitted models

Record host UTC timestamps, display local Cairo time, and retain MCU
monotonic time as the signal-analysis time base. Include source/firmware hashes,
toolbox versions, bus voltage assumption, magnet flags, overflow/packet loss,
timing jitter, calibration quality and processing history. Protect against
disk-full/write failure: mark incomplete data and notify the firmware/app to
stop a recording-dependent experiment. Bounded buffers must not block the
MCU's safety task.

Export selected channels to CSV with units/time/reference metadata in a
sidecar. Export figures and analysis tables, parameter JSON and MATLAB models.
No raw sample is replaced when a filter, detrend or calibration is applied.
Replay uses the same viewer/analysis services, with obvious REPLAY status and
no hardware writes. Compare multiple runs using separate legends/config IDs.

Time analysis must provide:

- selectable channels, absolute/device-relative/run-relative time;
- synchronized cursors, interval selection, zoom/pan, channel overlay;
- mean, median, SD, RMS, min/max, peak-to-peak, histogram and outlier count;
- detrend/remove mean as reversible operations;
- missing samples, clipping and held/invalid sample overlays;
- derived velocity with explicit estimator, time basis and filter delay;
- step-response metrics where applicable, with baseline/target selection and
  a reason when rise/settling time is not meaningful;
- actual command onset, delay estimate, input/output cross-correlation;
- response repeatability across experiments.

Pausing the plot must not pause acquisition or conceal that output is active.
Pausing an experiment is a different operation; simplest initial behavior is
Abort and rerun rather than an ambiguous resumable motor state.

## 11. Frequency and noise analysis

Analyze raw timestamped samples, not GUI points or half-second printed means.
Before every spectrum/FRF, report actual sample interval statistics, valid
sample count, gaps, clipping, sensor freshness and whether the record is
uniform enough for the selected method.

Do not replace gaps with zeros or repeat held sensor values without disclosure.
Default to contiguous valid intervals. If resampling is selected, show the
interpolation/anti-alias method and gap threshold, preserve the original data,
and record that this is a derived dataset. Refuse ordinary uniform-grid FFT
claims for materially nonuniform/gapped records until explicitly prepared.

Implement:

1. Properly normalized one-sided amplitude FFT, configurable rectangular/Hann
   window, mean removal, coherent-gain correction and DC/Nyquist treatment.
2. Welch PSD and amplitude spectral density, with window length, overlap, FFT
   length, segment count and units (e.g. A^2/Hz, A/sqrt(Hz), rad^2/Hz).
3. Spectrogram for nonstationary noise and time-varying experiments.
4. Selectable band power/RMS, dominant peaks, noise floor and autocorrelation.
5. Input/output frequency response magnitude/phase and coherence. Use applied
   input and aligned output, expose low-excitation bands, and require enough
   averaged segments. One-segment coherence is not evidence of a good model.
6. Raw-versus-filtered spectra using identical record/window settings.

Show Nyquist and true frequency resolution from observation/segment duration.
Zero padding creates denser plotted bins, not more resolving power. A detected
50 Hz peak is a candidate interference component, not proof of its cause.

At 2 ksample/s, the observable band ends at 1 kHz. This cannot resolve a
20 kHz PWM carrier or all current ripple. Offer a higher-rate ADC burst only
if its actual rate, bandwidth, timing and storage path have been validated;
otherwise state that a suitable scope/DAQ is needed for that band. A digital
filter applied after sampling cannot undo analog aliasing. Document the
analog bandwidth/filtering that is actually present instead of assuming it.

Provide guided recordings:

- drive disabled, mechanism stationary: ADC/sensor baseline noise;
- drive disabled, manual pendulum motion: angle noise and wrap behavior;
- bounded constant-duty plateaus: operational noise versus baseline;
- sine/chirp/PRBS: input/output frequency response;
- repeat after one deliberately documented hardware/filter change.

Do not call operational current ripple random sensor noise. Separate trends,
mechanical response, switching-related components and stochastic residuals
where the acquisition bandwidth allows it.

## 12. Filter workbench

Mandatory initial filter choices: bypass, moving average, exponential
low-pass, low-order Butterworth low-pass, and notch. Optional median and
Savitzky-Golay smoothing can be added if their implementation/toolbox
dependencies are satisfied.

Each design includes sample rate, type/order/cutoff or center/Q, coefficients,
initial state/reset behavior, units, causal/noncausal label and config hash.
Validate normalized frequencies and stability. Use second-order sections for
IIR implementations rather than fragile high-order direct-form polynomials.

Show time traces, RMS/SD, spectra, amplitude/phase response and group delay.
Quantify the noise reduction versus lag/distortion tradeoff; a smoother plot
is not automatically a better control signal.

Three explicitly separate destinations:

1. Display-only live filtering in MATLAB, retaining raw acquisition.
2. Offline analysis filtering, including optional zero-phase forward/backward
   filtering clearly labelled noncausal and unavailable to a real controller.
3. Deployable causal MCU filter for supported types, while disarmed, followed
   by readback/version confirmation and known-vector verification.

Start with deployable bypass/EMA/moving-average/biquad sections; reject
unsupported offline algorithms rather than approximating them silently.
Reset state at run/config/reference changes and across discontinuous segments.
Always preserve an independent raw/fast protection path.

Filter angles only after suitable unwrap/circular handling. Estimate velocity
using a documented finite-difference/regression/observer path with its timing
and delay. Do not convert a discontinuity at +/-pi into a velocity spike.
For identification, preserve common input/output timing; applying different
filters to them can create a false plant phase lag.

## 13. System identification and physical parameter estimation

These are two related but distinct workflows. A black-box transfer function
does not uniquely reveal every physical motor/pendulum parameter.

### 13.1 Dataset preparation

Select input and outputs, units, operating point, run/intervals and sample
time. Align by device acquisition/apply timestamps, handle known delays,
choose detrending/filtering, and show saturation, weak-field periods and gaps.
Report a quality summary before fitting.

Use separate whole runs or contiguous chronological blocks for estimation
and validation. Do not randomly shuffle time-series samples. A held-out
experiment with different excitation is preferable. Apply identical declared
processing and delay conventions to estimation/validation data.

Start around the stable hanging configuration with small, bounded excitation.
The Furuta system is nonlinear and coupled; a local linear fit has an operating
range. Do not run open-loop upright identification without a separately
validated stabilizer, and do not reuse a downward fit as the upright plant.

### 13.2 Black-box models

Provide functional GUI paths for:

- nonparametric frequency response with coherence;
- first/second-order transfer-function candidates with optional input delay;
- ARX order/delay selection;
- state-space/subspace candidates (e.g. n4sid/ssest) when available.

Support one motor input and selectable outputs: arm count/angle, arm speed,
pendulum angle, current. Always name model input units. A model fitted from
duty has duty-input gain; it cannot be relabelled voltage- or torque-input
without an explicit, justified conversion.

Arm position includes integration of speed; do not force its response into a
stable first-order speed-model template. Gearing, encoder resolution and
current gain must be known or qualified before reporting physical units.

For each model show:

- model structure/order, continuous/discrete sample time, delay and units;
- measured versus predicted and freely simulated response on held-out data;
- RMSE and defined fit percentage, with sensible handling of constant signals;
- residual autocorrelation and input/residual cross-correlation;
- poles/zeros, stability and Bode/step response where meaningful;
- uncertainty/confidence information when supported, not fabricated;
- warnings for insufficient excitation, poor coherence, clipping, ill
  conditioning, confounded parameters or operating-point mismatch.

Use System Identification Toolbox APIs where available, with R2023b syntax
checked locally. Provide basic regression/fitting fallbacks only where they
are mathematically justified; never label a fallback as equivalent to a
toolbox estimator it does not implement.

### 13.3 Physical parameter workflows

Create a parameter registry with value, units, source (measured/nominal/fitted),
uncertainty, reference frame, experiment/config IDs and validation status.
Unknowns remain unknown; do not populate plausible-looking numbers.

| Workflow | Required information | Valid result / limit |
| --- | --- | --- |
| Encoder scale | Known arm displacement and signed counts, multiple directions | Counts/output rev; backlash/repeatability observations |
| Geometry | Operator measurements of lengths, masses, COM, shaft/transmission details | Measured geometry; documented inertia assumptions |
| Pendulum free decay | Arm held mechanically fixed, small motion about down, angle time history | Natural frequency/damping; inertia or friction only with sufficient known geometry/mass |
| Motor speed response | Bounded duty steps/chirp, calibrated arm/motor speed, fixed load/configuration | Local gain/time constant/delay, direction asymmetry and operating range |
| Dead zone/friction/backlash | Small bidirectional ramps/reversals, angle/count response, optional calibrated current | Effective thresholds/hysteresis; not exact gearbox output position from rotor counts alone |
| Electrical model | Known terminal voltage and calibrated current/speed at sufficient bandwidth | R/L/Ke only when observable; unsupported parameters remain unavailable |
| Mechanical grey-box | Derived equations, measured geometry, selected free parameters, persistently exciting valid data | Fitted parameter combinations with identifiability and held-out validation |

For free decay, do not assume a disabled geared motor makes the arm fixed.
Record how the arm was constrained and verify negligible arm motion. Use
phiDown for the small-angle model. For a rigid pendulum, natural frequency
depends on m*g*l_COM/J_pivot; frequency alone cannot separately identify m,
l_COM and J_pivot. Friction estimates depend on the assumed friction model.

Use motor relations consistently:

    Va = R*i + L*di/dt + Ke*omega_motor
    tau_motor = Kt*i
    omega_motor = G*omega_arm

Here Va is not directly measured by the existing wiring. Duty*assumed_Vbus is
an approximation dependent on bridge mode/drops and supply sag. Do not claim
precise R, L or Ke from that approximation without uncertainty/validation.
Ordinary low-rate telemetry cannot establish a fast electrical time constant.
Kt, inertia, efficiency and friction can be mutually confounded; fix independently
measured parameters or fit identifiable combinations. Avoid noisy numerical
differentiation as an unquestioned physical measurement.

Use linear grey-box (idgrey/greyest) or nonlinear grey-box (idnlgrey/nlgreyest)
only with the corresponding model and supported toolboxes. A base-MATLAB
bounded/parameterized least-squares fallback may fit a small explicit model.
Respect physical bounds, expose initial guesses/free versus fixed parameters,
check sensitivity/conditioning, and validate on a separate experiment.
Do not promise automatic identification of every constant from one waveform.

No automated locked-rotor/stall test, auto-tuning sweep to maximum duty, or
unbounded random input. Electrical experiments that require added voltage
instrumentation must say so in the UI.

## 14. Toolbox and environment policy

At startup check MATLAB release, function availability and relevant license
features. Return a readable capability report. Prefer checking availability
without consuming licenses until a feature is used; handle checkout failures.

| Capability | Baseline / dependency |
| --- | --- |
| GUI, serial, recording, FFT, statistics, simple filters | Base MATLAB R2023b-compatible implementation |
| Welch PSD/FRF/coherence, advanced filter design | Signal Processing Toolbox where present; tested basic Welch fallback is acceptable |
| Advanced identification | System Identification Toolbox |
| Model response/control analysis | Control System Toolbox where needed |
| Constrained physical fitting | Optimization Toolbox or explicitly limited base-MATLAB alternative |
| Future hybrid controllers | Simulink/Stateflow, outside this phase's completion claim |

Hide no requested capability behind a silent failure. Missing optional
functions should show the exact dependency and keep the remaining app usable.
Compute arrays and draw them on uiaxes explicitly rather than depending on
newer plotting-function Parent options. Verify all APIs against R2023b.
Do not require an external Python service for the normal MATLAB workflow.

## 15. Implementation sequence and reviewable milestones

### M1 - Contracts and device-independent core

Create repository structure, profile/schema, protocol specification, golden
vectors, shared unit/sign conventions and mock transport. Implement CRC/COBS,
bounded parsing, command IDs, waveform math and calibration/filter primitives.
Deliver a script/test that exchanges packets between the two implementations.

### M2 - Firmware acquisition and disabled-device session

Implement BOOT/DISARMED, sensor drivers, timestamps, health, PCNT overflow,
ADC sampling, binary streams, calibration/config readback and passive burst
capture. Keep motor enable unavailable until M3 safety tests pass.
Demonstrate rates/overruns on a mock first, then operator-coordinated hardware.

### M3 - Bounded actuation and experiment engine

Implement the common output path, finite sequencer, ARM/STOP/session lease,
travel/current limits and faults. Test all invalid/duplicate/timeout paths.
Validate output semantics and timing on disabled/isolated hardware before
operator-started low-amplitude powered experiments.

### M4 - Usable MATLAB acquisition app

Deliver the launcher, connection workflow, Live/Manual, Calibration,
Experiments, recording and replay. Simulator mode must support the complete
workflow, including faults and packet loss. Every actuator request shows
device acknowledgement and actual state; no optimistic Armed indicators.

### M5 - Analysis and filter workbench

Deliver time/spectrum/noise tools, dataset-quality checks, high-rate burst
download, filter comparison and supported MCU filter deployment. Verify
numerical scaling/delay with synthetic known signals and cross-language tests.

### M6 - Identification and parameter registry

Deliver dataset preparation, model estimation/comparison/export and the
physical workflows above. Use known simulated plants to establish parameter
recovery and held-out validation before using real plant data.

### M7 - Integration, documentation and operator acceptance

Run tests, build firmware, run MATLAB backend and GUI smoke tests, exercise
mock/replay, and record exactly which hardware checks were actually performed.
Provide a concise commissioning guide, sample saved experiment, and known
limitations. Do not stop after M4 and call placeholder identification complete.

## 16. Verification and acceptance criteria

### Automated software checks

- C++ and MATLAB encode/decode the same golden frames, including negative
  counts, uint64 timestamps, floating values and known CRCs.
- Partial, concatenated, corrupt, oversized, duplicated, stale and out-of-order
  messages do not cause unintended commands or unbounded parser memory.
- Boot/reconnect/close/error paths cannot auto-arm or resume.
- STOP cancels a queued future start; an old heartbeat cannot renew a new
  session; MCU reset invalidates calibration references tied to count origin.
- PCNT extension handles both rollovers and direction changes.
- Angle zero/circular mean/unwrap work across 0/4096 and reset after data loss.
- Waveform previews and MCU samples match within quantization/tick tolerance,
  including segment edges, phase, slew constraints and seeded PRBS.
- Same raw ADC sample produces the paired millivolt value; invalid/clipped
  samples are flagged and not silently converted to plausible current.
- Known sine amplitude is recovered with correct FFT normalization; PSD
  integrates to known variance within stated statistical tolerance.
- Sampling jitter/gaps/held data trigger the appropriate analysis warnings.
- Known filter poles/gain/delay match between MATLAB and MCU precision;
  zero-phase offline processing cannot be deployed as a causal controller.
- Known synthetic plants are identified and validated on a separate run;
  unidentifiable parameter sets are detected or explicitly reported.
- Long synthetic recordings do not create unbounded GUI memory growth.
- Replay/export/import preserve raw samples, units and configuration metadata.

### Operator-coordinated hardware acceptance

Perform only when the operator is ready for the relevant physical test.
Do not automate live movement just to make the implementation report green.

1. Verify disarmed boot and external disable behavior with motor power removed.
2. Check fresh sensors, zero references, direction and raw-count travel limits.
3. Measure achieved acquisition intervals, jitter, loss and bandwidth for each
   advertised sampling mode. Publish the tested capabilities.
4. Inspect current raw waveforms and noise before choosing filters or current
   limits. Keep the pendulum weak-field status visible until actually resolved.
5. Start a short bounded motor command; verify physical direction/output mode
   and the recorded requested/applied signal.
6. Exercise normal completion, GUI Stop, application close, USB/heartbeat loss,
   stale commands and applicable limits under controlled conditions.
7. Initial firmware targets: STOP output disable within two actuator ticks
   after a valid STOP is parsed; heartbeat loss disable within configured
   timeout plus two ticks. Measure physical delay separately; UART/UI delay
   and mechanical stopping time are not included in those software targets.
8. Record one noise capture, one finite response experiment and one held-out
   validation run through the GUI; save/reopen and reproduce analysis.

Inaccessible test points remain unverified. A serial ACK proves firmware
processing, not physical PWM, motor-terminal voltage or electrical protection.

### Definition of done

From a clean checkout, documented steps build firmware and launch the app.
The app can run a complete mock experiment, save/replay it, calibrate references,
analyze noise/spectra, compare filters and estimate/validate a model without
editing source code. With the correct firmware and commissioned configuration,
the same controls operate hardware with finite MCU-timed experiments and
device-enforced stop behavior. Actual hardware validation and unresolved
limitations are explicitly recorded. No empty controls or invented parameters.

## 17. References and implementation notes

Use primary documentation and the installed R2023b help for exact signatures.
Current web pages may describe newer releases; the basic APIs below are the
intended families, not permission to use unsupported new options.

- [MATLAB programmatic app building, R2023b](https://www.mathworks.com/help/releases/r2023b/pdf_doc/matlab/creating_guis.pdf)
- [MATLAB serialport](https://www.mathworks.com/help/matlab/ref/serialport.html)
- [Byte callbacks](https://www.mathworks.com/help/instrument/serialport.configurecallback.html)
- [Welch PSD](https://www.mathworks.com/help/signal/ref/pwelch.html)
- [FRF estimation](https://www.mathworks.com/help/signal/ref/tfestimate.html)
- [Coherence and its averaging requirement](https://www.mathworks.com/help/signal/ref/mscohere.html)
- [Transfer-function identification](https://www.mathworks.com/help/ident/ref/tfest.html)
- [Linear grey-box estimation](https://www.mathworks.com/help/ident/ref/greyest.html)

Local context references (read as background; latest user report in section 2
takes precedence over older test-status prose):

- ../../PROJECT_BRIEF_AND_WIRING.md
- ../../Furuta-USB-Test/platformio.ini and src/main.cpp for pin/API references
- The course project PDF and earlier discussion handoff in the user's Downloads
  describe requirements and historical choices, not commands to execute.

## 18. Copyable implementation instruction

Implement this plan inside this repository. First inspect the repository and
applicable instructions, then implement all milestones through a working
MATLAB R2023b app, PlatformIO firmware, simulator/replay, protocol tests,
waveforms, calibration, noise/filter analysis and identification workflows.
Use modular supporting files behind a single FurutaWorkbench.m launcher.
Preserve raw data, metadata and weak-field diagnostics. Do not fabricate motor,
gearbox or pendulum parameters, and do not equate PWM duty with torque.

The physical system may already have 12 V applied. Begin with offline/mock
validation and compilation. Do not connect/reset/upload/arm/run physical
hardware automatically. Hardware operations must be coordinated with the
operator; the app itself starts disconnected and disarmed. Report tests that
actually ran, unavailable toolboxes, and remaining hardware limitations.
