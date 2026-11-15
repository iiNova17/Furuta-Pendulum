# Suggested characterization sequence

Begin with Simulator, then repeat selected measurements in a coordinated
hardware session after commissioning finite limits.

| Experiment | Setup | Analyze | Do not infer |
| --- | --- | --- | --- |
| Current noise | Motor disabled, mechanism still; raw ADC burst | Mean, drift, SD, histogram, PSD/ASD, autocorrelation, band RMS | Gain accuracy or under-load current accuracy from zero alone |
| Pendulum reference/motion | Arm centered; manually move pendulum both ways | Raw wrap, direction, relative/model angle, weak-field status | Accurate magnetic calibration merely from motion response |
| Encoder scale | Known physical arm displacement, multiple directions | Counts/output rev, residual/backlash behavior | Gearbox output position exactly from rotor counts |
| Small motor pulse/step | Finite commissioned duty/travel/current limits | Applied versus requested input, response/delay, count speed | Torque proportional to duty or calibrated terminal voltage |
| Sine/chirp/multisine | Stable hanging operating region, low amplitude, finite duration | FRF/coherence, repeatability, excitation bandwidth | A local hanging model as an upright model |
| PRBS | Known seed/bit period, limits active, room for travel | ARX/state-space model and held-out run | Symmetric command guarantees centered final arm |
| Free decay | Arm physically fixed, pendulum small-angle displacement | Natural frequency/damping; m*g*l/J | Independent mass, COM and inertia from frequency alone |
| Filter comparison | Same raw record and interval | Noise reduction, gain/phase/group delay, residual response | Smooth appearance means better feedback |

Save at least a second run for validation and repeatability. Keep input units
explicit: normalized duty, approximate volts with bus assumption, or an actual
measured quantity. Do not relabel a duty-input fitted gain as torque-input.
No automatic locked-rotor, unlimited jog or maximum-duty sweep is provided.
