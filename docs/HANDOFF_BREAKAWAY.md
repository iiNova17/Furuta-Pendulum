# Handoff: arm breakaway friction in the harness and the Simulink swing-up (needs R2026a)

Written 2026-11-19. For whoever owns `matlab/harness` (Simscape plant) and the swing-up block. Read all of it,
then do the "To do" steps in order. The rest is already done and verified (see the table at the end).

## Why

The first powered swing-up runs (firmware 1.7, `data/runs/20261119_*` on `onboard-control`, not committed here)
never left the hanging rest on their own. The swing-up's kick asked for **11.1 % duty**; the arm needs **13 % /
14 %** to break free (breakaway dead zone measured with Furuta Lab Motor tests, 2026-11-07, `deadzonePos/Neg`).
No copy of the model had this: `furuta_rhs.m`, the Simscape plant, `estimation.py` and the controller's motor
inverse all used only the identified sliding Coulomb friction (7.4 % / 7.3 %), smoothed with tanh(w0/0.1).
Replaying the recorded 11.1 % duty: the model without breakaway moves the arm 285° in 6 s, with it 0° (rig:
stuck 9–27 s until nudged).

## The model (done, report equation `eq:stick`, Table 6)

- `tsp = 0.13 * Vs / Rm = 0.4554 A`, `tsn = 0.14 * Vs / Rm = 0.4905 A` (at rest there is no back-EMF).
- Stick-slip (Karnopp), in `matlab/model/furuta_rhs.m` and `desktop/furuta_lab/estimation.py` (same physics):

  ```matlab
  fr = tc*tanh(w0/p.eps);                                   % sliding (identified) friction
  f0 = i - p.Beq*w0 - 2*p.Jp*s*c*w0*w1 + p.h*s*w1^2;        % arm generalized force without the dry friction
  f1 = p.Jp*s*c*w0^2 - p.b1*w1 - p.tp*tanh(w1/p.eps1) + p.G*s;
  M  = [p.Ja + p.Jp*s^2, p.h*c; p.h*c, p.Jp];
  if p.tsp > 0 && abs(w0) < p.stick_band                    % stick_band 0.05 rad/s
      need = f0 - M(1,2)*f1/p.Jp + (M(1,1)*p.Jp - M(1,2)^2)*w0/(p.Jp*p.stick_T);   % stick_T 2 ms
      if need <= p.tsp && need >= -p.tsn, fr = need; end      % stuck: friction holds the arm
  end
  f0 = f0 - fr;
  ```
- Validation fits of the identified model unchanged within 2 points (the square waves are above the breakaway).

## Temporary state of the harness (done, so the tests stay valid)

`harness_config.m` sets `cfg.p.tsp = 0; cfg.p.tsn = 0;` right after loading `furuta_params.m`: the Simscape
plant has no breakaway, so the harness keeps using the model without it and `tests/test_harness.m` (Simscape vs
`furuta_rhs`) and `validate_open_loop.m` still compare like with like. Checked in R2023b: `harness_plant_rhs`
with `cfg.p` equals the old model to 5e-9 on 300 states. **Not run:** `test_harness.m` itself (R2026a models).

## To do (R2026a)

1. **Simscape plant friction.** In `build_harness.m`, replace the arm Coulomb blocks ("Friction (eq. 78 ...)",
   `tanh`, `tau_c+`, `arm Coulomb`) by one MATLAB Function block that returns the arm friction torque exactly as
   `furuta_rhs` computes `fr` above, from the signals the plant already has (`w0`, `theta1`, `w1`, `i`) and `hp`
   (the plant parameters; add `tsp`, `tsn`, `stick_band`, `stick_T`). The pendulum dry friction stays as it is.
   Then delete the two `cfg.p.tsp/tsn = 0` lines in `harness_config.m`, rebuild (`build_harness`), and run
   `tests/test_harness.m`: `test_simscape_plant_is_the_report_model` must still agree to round-off. Add a test:
   from the hanging rest, 11 % duty for 2 s keeps the arm at rest; 20 % moves it.
2. **Swing-up motor inverse** (`swingup_energy.slx`, energy law). Firmware 1.8 (`firmware/include/onboard.h` on
   `onboard-control`) already does this; make the Simulink block identical:

   ```matlab
   sf   = tanh(w0/w_fric);
   coul = tcp*max(sf,0) + tcn*min(sf,0);                     % as before
   sa   = tanh(a0/a_stick);
   stat = tsp*max(sa,0) + tsn*min(sa,0);                     % breakaway in the direction of the wanted acceleration
   g    = exp(-(w0/w_stick)^2);
   i    = Ja*a0 + Beq*w0 + coul + stick_gain*g*(stat - coul);  % then the existing i_max / u_max limits
   ```
   Knobs in `ctrl_hybrid_design.m`: `D.swing.stick_gain = 1; D.swing.w_stick = 0.2; D.swing.a_stick = 5;` and
   `P.plant.tsp/tsn` from `cfg.p` (only correct after step 1). Expected (Python closed loop on the model with
   breakaway, 4 ms): the old law never moves the arm; the new one catches after 1.44 s, arm within 47°.
3. **Onboard export.** Make `export_onboard.m` write `pl_tsp`, `pl_tsn`, `sw_stick_gain`, `sw_w_stick`,
   `sw_a_stick` into `onboard_design.h` and regenerate the replay files with the new law. Then, on
   `onboard-control`: point the `OnboardParams` defaults at `design::` instead of the interim
   `include/onboard_stiction.h` (written by `firmware/tools/onboard_rates.py`), delete that header, and remove
   `c.p.stickGain=0` from the replay in `test/native_onboard.cpp`.
4. **Report section 10** (swing-up design): describe the breakaway feedforward and rerun the worst-case tests.
5. **Balance friction compensation** (firmware 1.9 on `onboard-control`; `ctrl_hybrid.slx` friction compensation
   block). On the rig with 1.8 the LQR's corrections (1-9 % duty) stuck below the 13 % breakaway; the arm broke
   free with a lurch and the LQR saturated (run `20261119_205453_867500`). Make the block identical to 1.9:

   ```matlab
   s    = tanh(u0/fric.band);
   coul = fric.duty(1)*max(s,0) + fric.duty(2)*min(s,0);              % identified Coulomb duty, as before
   stat = (p.Rm*p.tsp*max(s,0) + p.Rm*p.tsn*min(s,0))/p.Vs;           % breakaway duty (13 % / 14 %)
   g    = exp(-(w0/w_stick)^2);
   u    = u0 + fric.gain*(coul + fric.stick*g*(stat - coul));
   ```
   Knobs: `D.fric.gain = 1` (was 0), `D.fric.stick = 1`. Python closed loop on the model with breakaway: arm stuck
   41-49 % of the balance time without, 74-80 % with Coulomb only, 4 % with this; rerun `lqr_corner_test`.

## Results

| Item | Result | Verified |
|---|---|---|
| Breakaway values from the dead zone | 0.4554 / 0.4905 A | yes (arithmetic) |
| Model without / with breakaway at 11.1 % from rest | 285° / 0° in 6 s (rig: stuck) | yes, Python model vs recorded runs |
| Validation fits with breakaway | within 2 points of Table 4 | yes, Python |
| `furuta_rhs.m` = Python model (with breakaway) | 1.3e-8 on 400 states (133 stuck) | yes, R2023b |
| Harness uses the model without breakaway (`cfg.p`) | equals the old model to 5e-9 | yes, R2023b (`.m` only) |
| `tests/test_harness.m` after this change | — | **unverified** (R2026a) |
| Firmware 1.8 feedforward = simulated law | 2e-11 over 1500 steps; kick 24.1 % duty | yes (native C++) |
| Swing-up from rest on the model with breakaway | old law stuck, new law caught at 1.44 s | yes, Python closed loop |
| Swing-up from rest on the rig with firmware 1.8 | — | **unverified** (not run with power yet) |
| Firmware 1.9 balance compensation = simulated law | 1.5e-13; arm stuck 4 % vs 41-49 % (simulation) | yes (native C++, Python) |
| Simscape plant / Simulink swing-up and balance with breakaway | — | **to do** (steps 1–5) |
