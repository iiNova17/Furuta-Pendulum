# Hardware interface and commissioning

The user reports successful motor, encoder and current-sensor operation.
Firmware 1.1 was built and uploaded during the focused-app follow-up, then
tested over USB with the 12 V line disconnected. Powered motion, mechanical
hard stops, current limits and physical parameter values remain unvalidated
by these passive tests. See VALIDATION.md for the measured scope.

| GPIO | Connection |
| --- | --- |
| 4 | ACS712-5A OUT, direct ADC input |
| 5 / 6 | Motor encoder A / B |
| 8 / 9 | AS5600 SDA / SCL, 3.3 V I2C |
| 15 | TB6612 PWMA |
| 16 / 17 | TB6612 AIN1 / AIN2 |
| 18 | TB6612 STBY, plus external 10 kohm to GND |

Keep UART0 GPIO43/44 and native USB GPIO19/20 free. Use COM USB, not native
USB, for this firmware's UART0 transport.

Driver VCC is 3.3 V; VM is the switched 12 V motor supply. Grounds are common.
The current sensor needs 5 V. The magnetic encoder/logic need compatible 3.3 V
power and pull-ups. Motor-current return must not flow through MCU ground
traces. Retain the specified local VM decoupling.

Current path: AO1 -> sensor IP+ -> IP- -> motor pin 6; AO2 -> motor pin 1.
Neither motor terminal is ground. Channel B control inputs are grounded and
its outputs unconnected.

## Output semantics

Positive normalized duty: AIN1 HIGH, AIN2 LOW, PWM on PWMA.
Negative duty reverses AIN1/AIN2. The carrier defaults to 20 kHz, 10-bit PWM.
TB6612 PWM-low during a driven direction is short-brake behavior, not coast.
Zero output, Stop, completion and faults set PWM zero, directions LOW and
STBY LOW (standby/high impedance). There is no separate active-braking command
in this release. Motor inertia can still carry motion after disable.

Direction reversal slews to zero and inserts at least one actuator tick of
standby before driving the opposite direction. Requested/applied duty and
limiting flags are recorded.

## Known physical limits

- The sensor's 5 V output can exceed ESP32 input limits at higher current or
  during faults. Direct connection has no analog protection. Software ADC
  attenuation/trips cannot protect an overdriven pin.
- Reported motor stall current is about 2.2 A versus the TB6612's stated
  1.2 A average capability. Successful motion is not thermal/current sizing.
  Do not perform a locked-rotor experiment with this workbench.
- No motor-terminal voltage, VM, supply rail or actual STBY voltage is sensed.
  Estimated volts use the entered bus assumption and omit unmeasured drops/sag.
- Encoder weak-field status remains a warning. Responding angles do not
  establish calibrated angular accuracy.
- The motor encoder senses the geared motor, not an independent arm encoder;
  gearbox backlash cannot be reconstructed exactly from it.

## First commissioning

With motor power physically removed, flash this project's binary firmware.
Verify disabled boot and the wiring/physical power-disable arrangement.
Connect the app; observe actual acquisition and health before commissioning
nonzero output. Set finite raw-count travel and duty/duration/slew limits,
then explicitly mark the arm's physical center. Calibrate scale before using
radians or degree limits. Re-center after every MCU restart.

Use the physical supply switch/disconnect for emergency isolation. The GUI
Stop is a command plus a lease timeout, not a replacement for that hardware.
Current limits require valid zero/sensitivity and a margin compatible with
measured noise; zero ampere threshold means software current trip is OFF.
Do not infer a protective threshold from a plausible-looking current trace.

Check actual sample rates, PWM timing, Stop delay and fault behavior in a
controlled hardware session before using identification experiments.
Inaccessible voltage test points remain unverified.
