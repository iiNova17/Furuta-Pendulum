"""Figures for the PID design section of furuta_model.tex.

Reads pid_data.mat, written by matlab/simulink_r2026a/controllers/hybrid/pid_report_data.m from the results of
pid_tune.m, and writes fig_pid_tuning.pdf, fig_pid_sens.pdf, fig_pid_push.pdf and fig_pid_swingup.pdf here.
Run from the repository root:  python report/figures/make_pid_figures.py
"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

HERE = Path(__file__).resolve().parent
NAVY, STEEL, LIGHT, TERRA, GREY, GREEN = '#143C6E', '#43638B', '#4C87AD', '#CC7451', '#888888', '#2d6a3e'
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['CMU Serif', 'Latin Modern Roman', 'Computer Modern Roman', 'DejaVu Serif'],
                     'mathtext.fontset': 'cm', 'font.size': 9, 'axes.labelsize': 9, 'legend.fontsize': 8, 'xtick.labelsize': 8,
                     'ytick.labelsize': 8, 'axes.grid': True, 'grid.color': '#dddddd', 'grid.linewidth': .6,
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.prop_cycle': matplotlib.cycler(color=[NAVY, TERRA, STEEL, LIGHT]),
                     'savefig.bbox': 'tight', 'savefig.pad_inches': .02, 'lines.linewidth': 1.1})
TW = 6.3
D = loadmat(HERE / 'pid_data.mat', squeeze_me=True, struct_as_record=False)
NAMES = [str(n) for n in D['names']]
LAB = {'Kp1': r'$K_{p1}$', 'Kd1': r'$K_{d1}$', 'Kp0': r'$K_{p0}$', 'Ki0': r'$K_{i0}$', 'Kd0': r'$K_{d0}$'}
MARK = ['o', 's', '^', 'D', 'v']
COL = [NAVY, TERRA, STEEL, GREEN, GREY]
deg = np.degrees

# ---- 1. search history: cost of every evaluated candidate and the accepted path ----------------------
H = D['hist']; J = np.asarray(H.J, float); acc = np.asarray(H.accepted, bool); g = np.atleast_2d(H.g)
k = np.arange(len(J))
fig, (a1, a2) = plt.subplots(1, 2, figsize=(TW, 2.5), gridspec_kw={'width_ratios': [1.25, 1]})
fin = np.isfinite(J)
a1.plot(k[fin & ~acc], J[fin & ~acc], 'x', color=GREY, ms=4, mew=.8, label='rejected candidate')
a1.plot(k[acc], J[acc], 'o-', color=NAVY, ms=4, mfc='w', label='accepted step')
if 'nomargin' in D:
    a1.axhline(float(D['nomargin'].J), ls='--', color=TERRA, lw=.9, label='best without the margin limit')
a1.set_yscale('log'); a1.set_xlabel('Candidate number'); a1.set_ylabel(r'Cost $J$')
a1.legend(frameon=False, loc='upper right')
ga = g[acc] / g[0]
for i, n in enumerate(NAMES):
    a2.plot(np.arange(len(ga)), ga[:, i], marker=MARK[i], color=COL[i], ms=3.5, mfc='w', label=LAB[n])
a2.axhline(1, color='#bbbbbb', lw=.8, zorder=0)
a2.set_xlabel('Accepted step'); a2.set_ylabel('Gain / starting gain')
a2.legend(frameon=False, ncol=5, fontsize=7.5, loc='lower center', bbox_to_anchor=(0.5, 1.0), handlelength=1.2, columnspacing=.8)
fig.tight_layout(); fig.savefig(HERE / 'fig_pid_tuning.pdf'); plt.close(fig)

# ---- 2. one-at-a-time sensitivity around the tuned gains --------------------------------------------
S = D['sens']; m = np.asarray(S.mult, float); it = np.atleast_2d(S.itae); ok = np.atleast_2d(S.pass_ if hasattr(S, 'pass_') else getattr(S, 'pass')).astype(bool)
fig, ax = plt.subplots(figsize=(TW * .7, 2.7))
for i, n in enumerate(NAMES):
    ax.plot(m, it[i], '-', color=COL[i], lw=1)
    ax.plot(m[ok[i]], it[i][ok[i]], MARK[i], color=COL[i], mfc='w', ms=4.5, label=LAB[n])
    ax.plot(m[~ok[i]], it[i][~ok[i]], 'x', color=COL[i], ms=5, mew=1.1)
ax.set_xscale('log'); ax.set_xticks(m); ax.set_xticklabels([f'{v:g}' for v in m]); ax.minorticks_off()
ax.set_yscale('log')
ax.set_xlabel('Gain / tuned gain'); ax.set_ylabel(r'ITAE after the push [rad$\,$s$^2$]')
ax.legend(frameon=False, ncol=3, fontsize=7.5, loc='upper center')
ax.text(.02, .02, 'crosses: push test failed', transform=ax.transAxes, fontsize=7, color=GREY)
fig.tight_layout(); fig.savefig(HERE / 'fig_pid_sens.pdf'); plt.close(fig)

# ---- 3. push test: starting gains, tuned gains and LQR, nominal lag and 15 ms -----------------------
R = D['runs']
fig, axs = plt.subplots(3, 2, figsize=(TW, 4.6), sharex=True)
for j, lag in enumerate([10, 15]):
    for name, col, ls, lab in [('start', GREY, '--', 'PID, starting gains'), ('lqr', TERRA, ':', 'LQR'), ('tuned', NAVY, '-', 'PID, tuned gains')]:
        r = getattr(getattr(R, name), f'push_{lag}')
        t = np.asarray(r.t, float)
        axs[0, j].plot(t, deg(np.asarray(r.th1, float)), ls, color=col, label=lab)
        axs[1, j].plot(t, deg(np.asarray(r.th0, float)), ls, color=col)
        axs[2, j].plot(t, np.asarray(r.u, float), ls, color=col, lw=.8)
    axs[0, j].axhspan(-3, 3, xmin=4 / 6, color='#e8eef6', zorder=0)
    axs[0, j].set_title(f'Pendulum sensor lag {lag} ms', fontsize=8.5)
    axs[2, j].set_xlabel('Time [s]')
axs[0, 0].set_ylabel(r'$\theta_1$ [deg]'); axs[1, 0].set_ylabel(r'$\theta_0$ [deg]'); axs[2, 0].set_ylabel('Duty cycle')
for a in axs[0]:
    a.set_ylim(-6, 6)
h, l = axs[0, 0].get_legend_handles_labels()
fig.tight_layout(rect=(0, 0, 1, .95))
fig.legend(h, l, frameon=False, ncol=3, fontsize=8, loc='upper center', bbox_to_anchor=(0.5, 1.0))
fig.savefig(HERE / 'fig_pid_push.pdf'); plt.close(fig)

# ---- 4. swing-up and catch with the tuned PID (nominal lag) ----------------------------------------
r = R.tuned.swingup_10; t = np.asarray(r.t, float); mode = np.asarray(r.mode, float)
fig, axs = plt.subplots(3, 1, figsize=(TW, 4.0), sharex=True)
th1 = deg(np.asarray(r.th1, float)); th1w = (th1 + 180) % 360 - 180
th1w[1:][np.abs(np.diff(th1w)) > 180] = np.nan
axs[0].plot(t, th1w, color=NAVY); axs[0].set_ylabel(r'$\theta_1$ [deg]'); axs[0].set_yticks([-180, -90, 0, 90, 180])
axs[1].plot(t, deg(np.asarray(r.th0, float)), color=STEEL); axs[1].set_ylabel(r'$\theta_0$ [deg]')
axs[2].plot(t, np.asarray(r.u, float), color=TERRA, lw=.8); axs[2].set_ylabel('Duty cycle'); axs[2].set_xlabel('Time [s]')
tc = float(r.tcap)
for a in axs:
    a.axvline(tc, color=GREEN, ls='--', lw=1)
axs[0].text(tc + .15, 120, f'catch at {tc:.2f} s', color=GREEN, fontsize=8)
axs[0].set_xlim(0, t[-1])
fig.tight_layout(); fig.savefig(HERE / 'fig_pid_swingup.pdf'); plt.close(fig)
print('wrote fig_pid_tuning.pdf, fig_pid_sens.pdf, fig_pid_push.pdf, fig_pid_swingup.pdf')
