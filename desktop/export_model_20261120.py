"""Write the 2026-11-20 model (variant E of reidentify_variants.py) as a MATLAB parameter set.

    python desktop/export_model_20261120.py

-> data/estimation/suite_20261120/export/   (estimation.export: parameters.json, furuta_model.m, furuta_rhs.m, .tex)
-> matlab/model/furuta_params_20261120.m    (same struct, fields and linear models as furuta_params.m)
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from furuta_lab import estimation as E  # noqa: E402

SUITE = ROOT / 'data/estimation/suite_20261120'
GRAVITY = 9.80665


def mat(M):
    M = np.atleast_2d(np.real(np.asarray(M, float)))
    return '[' + '; '.join(' '.join(f'{v:.10g}' for v in row) for row in M) + ']'


def main():
    var = json.loads((SUITE / 'variants.json').read_text())
    P = E.full(var['best_model'])
    se = json.loads((SUITE / 'result.json').read_text())['se']       # least-squares standard errors (same fit runs)
    larm = GRAVITY * P['h'] / P['G']
    lin = E.linear_models(P, 12.0, 1e-3)
    info = {'identified': '2026-11-20', 'method': 'desktop/reidentify_variants.py variant E: claude_loop chain, '
            'refinement of h, b1, tp, Ja, Beq, tcp, tcn by simulation, fit runs b1_fitA-C, pendulum timing 10 ms',
            'breakaway': '6 % / 6 % duty, first moving pulse, median of three arm positions (b1_breakaway_low)',
            'note': 'se are those of the least-squares fit (before refinement)', 'pend_delay_ms': var['pend_delay_ms'],
            'validation_fit_pct': var['scores'][var['best']]}
    E.export(SUITE / 'export', P, se, lin, None, larm, info)
    up, dn = lin['upright'], lin['hanging']
    g = lambda k: f'{P[k]:.10g}'  # noqa: E731
    sc = var['scores'][var['best']]
    L = [
        '% FURUTA_PARAMS_20261120  Furuta pendulum model re-identified on 2026-11-20, after the rig was moved.',
        '% Same struct, fields and linear models as furuta_params.m (report Table 6, 2026-11-15), which stays the',
        '% report\'s model and the harness default. Select this one with harness_config(..., \'Model\', \'20261120\').',
        '%',
        '% Torque-like parameters are divided by K_tau: torque is expressed in amperes of motor current.',
        '% State x = [theta0; theta1; dtheta0; dtheta1] in rad and rad/s; theta1 = 0 upright, pi hanging.',
        '% Input: duty d in [-1, 1] (armature voltage V = d*Vs). Outputs: theta0, theta1, current i (A).',
        '% Source: data/estimation/suite_20261120 (desktop/reidentify_variants.py, variant E), the claude_loop chain',
        '% with the arm terms (Ja, Beq, tc+, tc-) also refined by simulation and a 10 ms pendulum timing correction.',
        f'% Validation (3 runs never fitted): arm angle {sc["arm angle"]:.1f} %, pendulum {sc["pendulum angle"]:.1f} %,',
        f'% arm speed {sc["arm speed"]:.1f} %, current {sc["current"]:.1f} % (Table 6 on the same runs: 54.7 / 45.3 / 76.0 / 71.4 %).',
        '% Not modelled: the shaft coupling (about 1-3 deg of play plus flex between motor encoder and arm; the',
        '% pendulum reading lags the encoder by 9-15 ms). Treat it as a delay in the controller design.',
        'p = struct();',
        f'p.Rm = {g("Rm")};     % ohm, armature resistance incl. driver',
        f'p.Kb = {g("Kb")};    % V*s/rad, back-EMF constant at the arm shaft',
        f'p.Ja = {g("Ja")};  % A*s^2/rad, arm-side inertia',
        f'p.Jp = {g("Jp")}; % A*s^2/rad, pendulum inertia about its pivot',
        f'p.h = {g("h")};  % A*s^2/rad, inertial coupling',
        f'p.G = {g("G")};    % A, gravity torque coefficient',
        f'p.Beq = {g("Beq")};  % A*s/rad, arm viscous friction',
        f'p.b1 = {g("b1")}; % A*s/rad, pendulum viscous friction',
        f'p.tcp = {g("tcp")};   % A, arm Coulomb friction, dtheta0 > 0',
        f'p.tcn = {g("tcn")};   % A, arm Coulomb friction, dtheta0 < 0',
        f'p.tp = {g("tp")}; % A, pendulum joint dry friction',
        'p.Vs = 12;              % V, supply voltage',
        f'p.eps = {E.SIM_EPS:g};   % rad/s, arm friction smoothing tanh(w/eps)',
        f'p.eps1 = {E.SIM_EPS1:g}; % rad/s, pendulum friction smoothing',
        '% Arm breakaway (static) friction: 6 % duty in both directions, the first pulse that moved the arm (median',
        '% over arm positions +2 / -24 / +31 deg, runs b1_breakaway_low of 2026-11-20); 13 % / 14 % on 2026-11-07.',
        f'p.tsp = {g("tsp")};  % A, positive direction',
        f'p.tsn = {g("tsn")};  % A, negative direction',
        f'p.stick_band = {E.STICK_BAND:g};     % rad/s, the arm can stick inside |dtheta0| < stick_band',
        f'p.stick_T = {E.STICK_T:g};       % s, time constant that brings a stuck arm to rest',
        f'p.Larm = {larm:.10g};  % m, arm length from h/G',
        'p.pend_delay = 0.010;   % s, pendulum timing correction used in the identification (15 Nov: 7 ms)',
        'x0_hanging = [0; pi; 0; 0];',
        'x0_upright = [0; 0; 0; 0];',
        '',
        '% Linear models (input V in volts; *_duty versions take the duty cycle)',
    ]
    for tag, s in (('up', up), ('down', dn)):
        L += [f'A_{tag} = {mat(s["A"])};', f'B_{tag} = {mat(s["B"])};  % per volt',
              f'Bd_{tag} = {mat(s["Bd"])};  % per unit duty', f'C_{tag} = {mat(s["C"])};', f'D_{tag} = {mat(s["D"])};',
              f"sys_{tag} = ss(A_{tag}, B_{tag}, C_{tag}, D_{tag}, 'InputName', 'V', 'OutputName', {{'theta0','theta1','i'}});",
              f"sys_{tag}_duty = ss(A_{tag}, Bd_{tag}, C_{tag}, D_{tag}*p.Vs, 'InputName', 'duty', 'OutputName', {{'theta0','theta1','i'}});",
              f'G_{tag} = minreal(tf(sys_{tag}));']
    L += ['% Nonlinear model: furuta_rhs.m (the same function works with both parameter sets), e.g.',
          '%   ode45(@(t,x) furuta_rhs(x, duty(t), p), [0 5], x0_hanging)']
    out = ROOT / 'matlab/model/furuta_params_20261120.m'
    out.write_text('\n'.join(L) + '\n', encoding='utf-8')
    poles = np.linalg.eigvals(up['A'])
    print('wrote', out.relative_to(ROOT), '| upright poles', np.round(np.sort(poles.real), 2),
          '| Larm %.2f cm' % (larm * 100))
    old = np.linalg.eigvals(np.array(json.loads((ROOT / 'data/estimation/claude_loop/export/parameters.json').read_text())['upright']['A']))
    print('Table 6 upright poles', np.round(np.sort(old.real), 2))


if __name__ == '__main__':
    main()
