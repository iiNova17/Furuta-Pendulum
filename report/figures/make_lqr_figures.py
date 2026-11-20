"""Figures for the LQR design section (section 9 of furuta_model.tex).

Reads lqr_data.mat, which holds the simulation results of the controller harness
(matlab/harness: lqr_workbench, lqr_corner_test, corner_plants and the design-space sweeps),
and writes fig_lqr_map.pdf, fig_lqr_corners.pdf, fig_lqr_delay.pdf and fig_lqr_fric.pdf here.
Run from the repository root:  python report/figures/make_lqr_figures.py
"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.io import loadmat

HERE = Path(__file__).resolve().parent
NAVY, STEEL, LIGHT, TERRA, GREY = '#143C6E', '#43638B', '#4C87AD', '#CC7451', '#888888'
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['CMU Serif', 'Latin Modern Roman', 'Computer Modern Roman', 'DejaVu Serif'],
                     'mathtext.fontset': 'cm', 'font.size': 9, 'axes.labelsize': 9, 'legend.fontsize': 8, 'xtick.labelsize': 8,
                     'ytick.labelsize': 8, 'axes.grid': True, 'grid.color': '#dddddd', 'grid.linewidth': .6,
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.prop_cycle': matplotlib.cycler(color=[NAVY, TERRA, STEEL, LIGHT]),
                     'savefig.bbox': 'tight', 'savefig.pad_inches': .02, 'lines.linewidth': 1.1})
TW = 6.3
D = loadmat(HERE / 'lqr_data.mat', squeeze_me=True, struct_as_record=False)
BLUES = LinearSegmentedColormap.from_list('b', ['#e8eef6', LIGHT, NAVY])
WARM = LinearSegmentedColormap.from_list('w', ['#fbf3e6', '#f5e6b2', TERRA, '#7a2e14'])


def cells(ax, Z, fmt, dark):
    for (r, c), v in np.ndenumerate(Z):
        if np.isfinite(v):
            ax.text(c, r, fmt % v, ha='center', va='center', fontsize=6.3, color='w' if dark(v) else '#222222')


# ---- 1. design map: ITAE and peak current over (max theta1, max duty) -------------------------
M = D['map']; th1, du = M.th1, M.du
fig, axs = plt.subplots(1, 2, figsize=(TW, 2.9))
for ax, Z, cm, lab, fmt, dark, lim in [
        (axs[0], np.where(np.asarray(getattr(M, 'pass'), bool), M.itae, np.nan), BLUES.reversed(), r'ITAE of $\theta_1$ after the push [rad$\,$s$^2$]', '%.2f', lambda v: v < .13, None),
        (axs[1], M.ipk, WARM, 'Peak armature current [A]', '%.2f', lambda v: v > 1.62, (1.1, 1.95))]:
    im = ax.imshow(Z, origin='lower', aspect='auto', cmap=cm, vmin=None if lim is None else lim[0], vmax=None if lim is None else lim[1])
    ax.set_xticks(range(len(th1))); ax.set_xticklabels([f'{v:g}' for v in th1])
    ax.set_yticks(range(len(du))); ax.set_yticklabels([f'{v:.2f}' for v in du])
    ax.set_xlabel(r'$\theta_{1,\max}$ [deg]'); ax.set_ylabel(r'$d_{\max}$ [-]'); ax.grid(False)
    cb = fig.colorbar(im, ax=ax, fraction=.05, pad=.02); cb.set_label(lab, fontsize=7.5); cb.ax.tick_params(labelsize=7)
fail = ~np.asarray(getattr(M, 'pass'), bool)
r, c = np.nonzero(fail)
axs[0].plot(c, r, 'x', color=GREY, ms=5, mew=1.2)
axs[1].contour(M.ipk, levels=[1.5], colors=['#7a2e14'], linewidths=1.2, linestyles='--')
for ax in axs:
    ax.plot(list(th1).index(5.0), int(np.argmin(abs(du - .15))), 's', mfc='none', mec='k', ms=11, mew=1.2)
    ax.plot(list(th1).index(4.5), int(np.argmin(abs(du - .16))), 'o', mfc='none', mec='#2d6a3e', ms=12, mew=1.5)
fig.tight_layout(w_pad=1.2); fig.savefig(HERE / 'fig_lqr_map.pdf'); plt.close(fig)

# ---- 1b. the same map as a surface: height = ITAE, colour = peak current ---------------------
from matplotlib import cm as mcm
from matplotlib.colors import Normalize
X, Y = np.meshgrid(th1, du)
ok = np.asarray(getattr(M, 'pass'), bool)
Z = np.where(ok, M.itae, np.nan)
norm = Normalize(1.15, 1.6)
fig = plt.figure(figsize=(TW, 4.6))
ax = fig.add_axes([-.02, -.04, .86, 1.08], projection='3d', computed_zorder=False)
ax.plot_surface(X, Y, Z, facecolors=WARM(norm(M.ipk)), rstride=1, cstride=1, linewidth=.3,
                edgecolor='#555555', antialiased=True, shade=False, alpha=.95)
z0 = .07
ax.contour(X, Y, M.ipk, levels=[1.5], zdir='z', offset=z0, colors=['#7a2e14'], linewidths=1.3, linestyles='--')
ax.contour(X, Y, M.ipk, levels=[1.35], zdir='z', offset=z0, colors=['#2d6a3e'], linewidths=1.3)
ax.scatter(X[~ok], Y[~ok], np.full((~ok).sum(), z0), marker='x', color='#3a1206', s=22, depthshade=False)
for (xv, yv, mk, col, lab, dz, dx) in [(5.0, .15, 's', 'k', 'start', .03, 1.3), (4.5, .16, 'o', '#2d6a3e', 'stiff design', .06, -.6)]:
    j, i = int(np.argmin(abs(du - yv))), list(th1).index(xv)
    ax.plot([xv, xv], [yv, yv], [z0, M.itae[j, i]], color=col, lw=1, zorder=10)
    ax.scatter([xv], [yv], [M.itae[j, i]], marker=mk, color=col, s=36, depthshade=False, zorder=11)
    ax.plot([xv, xv], [yv, yv], [M.itae[j, i], M.itae[j, i] + dz], color=col, lw=.8, zorder=10)
    ax.text(xv + dx, yv, M.itae[j, i] + dz + .006, lab, color=col, fontsize=7.5, ha='center', zorder=12, fontweight='bold')
ax.set_xlabel(r'$\theta_{1,\max}$ [deg]', labelpad=2); ax.set_ylabel(r'$d_{\max}$ [-]', labelpad=4)
fig.text(-.02, .5, r'ITAE after the push [rad$\,$s$^2$]', rotation=90, va='center', fontsize=9)
ax.set_zlim(z0, .25); ax.view_init(elev=24, azim=-148); ax.set_box_aspect(None, zoom=.95)
ax.tick_params(labelsize=7, pad=1)
sm = mcm.ScalarMappable(norm=norm, cmap=WARM)
cb = fig.colorbar(sm, cax=fig.add_axes([.86, .2, .018, .6])); cb.set_label('Peak armature current [A]', fontsize=7.5); cb.ax.tick_params(labelsize=7)
fig.savefig(HERE / 'fig_lqr_surface.pdf', bbox_inches='tight', pad_inches=.05); plt.close(fig)

# ---- 2. twelve consistent worst cases: stiff vs soft ----------------------------------------
fig, ax = plt.subplots(figsize=(TW, 3.6))
n = 12; y = np.arange(n)[::-1]
for k, (key, col, lab) in enumerate([('stiff', TERRA, r'$\theta_{1,\max}=4.5^\circ$, $d_{\max}=0.16$'),
                                      ('soft', NAVY, r'$\theta_{1,\max}=8^\circ$, $d_{\max}=0.10$')]):
    T = D[key]; ipk = np.maximum(T.bi, T.pi); ok = np.asarray(T.bp, bool) & np.asarray(T.pp, bool)
    yy = y + (k - .5) * .28
    ax.hlines(yy, 0.75, ipk, color=col, alpha=.35, lw=1.6)
    ax.plot(ipk[ok], yy[ok], 'o', color=col, ms=6, label=lab + ' (passed)')
    ax.plot(ipk[~ok], yy[~ok], 'x', color=col, ms=7, mew=1.8, label=lab + ' (failed)')
ax.axvline(1.5, color='#7a2e14', ls='--', lw=1.2); ax.text(1.505, n - .45, 'trip level 1.5 A', color='#7a2e14', fontsize=7.5)
labels = [p.replace('pendulum', r'$\tilde G$').replace('arm friction', r'$\tilde B_{eq}$') for p in D['soft'].plant]
ax.set_yticks(y); ax.set_yticklabels(labels, fontsize=7.5)
for b, txt in enumerate(['nominal timing', '+1 controller period of delay', 'pendulum sensor lag 10 ms']):
    if b % 2 == 0:
        ax.axhspan(y[4 * b + 3] - .5, y[4 * b] + .5, color='#f3f3f3', zorder=0)
    ax.text(2.14, y[4 * b] + .47, txt, ha='right', va='center', fontsize=7, color='#222222', fontweight='bold')
ax.set_xlim(.75, 2.15); ax.set_ylim(-.6, n - .4); ax.grid(axis='y', visible=False)
ax.set_xlabel('Peak armature current over the balance and push tests [A]')
ax.legend(loc='upper center', bbox_to_anchor=(.5, -.16), ncol=2, frameon=False)
fig.savefig(HERE / 'fig_lqr_corners.pdf'); plt.close(fig)

# ---- 3. delay: theory (perfect state) vs simulation --------------------------------------------
fig, axs = plt.subplots(1, 2, figsize=(TW, 2.8))
dm = D['dm']; ax = axs[0]
cs = ax.contourf(dm.th1, dm.du, dm.dm, levels=np.arange(18, 42, 2), cmap=BLUES)
cl = ax.contour(dm.th1, dm.du, dm.dm, levels=[20, 25, 30, 35], colors='w', linewidths=.7)
ax.clabel(cl, fmt='%d ms', fontsize=6.5)
for (x, yv, lab, col) in [(8, .10, 'soft', NAVY), (4.5, .16, 'stiff', TERRA)]:
    ax.plot(x, yv, 'o', color=col, mec='w', ms=7); ax.annotate(lab, (x, yv), xytext=(5, 4), textcoords='offset points', fontsize=7.5, color='k')
ax.set_xlabel(r'$\theta_{1,\max}$ [deg]'); ax.set_ylabel(r'$d_{\max}$ [-]'); ax.grid(False)
cb = fig.colorbar(cs, ax=ax, fraction=.05, pad=.02); cb.set_label('Delay margin [ms]', fontsize=7.5); cb.ax.tick_params(labelsize=7)
MD = D['delay']; ax = axs[1]
im = ax.imshow(MD.ipk, origin='lower', aspect='auto', cmap=WARM, vmin=1.1, vmax=2.05,
               extent=(-.1, 1.1, MD.lag[0] - .5, MD.lag[-1] + .5))
S, L = np.meshgrid(MD.s, MD.lag); trip = np.asarray(MD.trip, bool)
ax.plot(S[trip], L[trip], 'x', color='#3a1206', ms=5, mew=1.2)
ax.plot(S[~trip], L[~trip], 'o', color='#2d6a3e', ms=4)
ax.set_xticks([0, .2, .4, .6, .8, 1]); ax.set_xticklabels(['soft', '', '', '', '', 'stiff'])
ax.set_xlabel('Design (linear blend of the two weightings)'); ax.set_ylabel('Pendulum sensor lag [ms]'); ax.grid(False)
cb = fig.colorbar(im, ax=ax, fraction=.05, pad=.02); cb.set_label('Peak armature current [A]', fontsize=7.5); cb.ax.tick_params(labelsize=7)
fig.tight_layout(w_pad=1.2); fig.savefig(HERE / 'fig_lqr_delay.pdf'); plt.close(fig)

# ---- 4. friction compensation: push response -------------------------------------------------
ts = D['ts']
fig, axs = plt.subplots(2, 1, figsize=(TW, 3.3), sharex=True)
for key, col, ls, lab in [('f0', TERRA, '-', r'$k_f = 0$'), ('f1', NAVY, '-', r'$k_f = 1$')]:
    s = getattr(ts, key)
    axs[0].plot(s.t, np.degrees(s.th0), color=col, ls=ls, label=lab)
    axs[1].plot(s.t, np.degrees(s.th1), color=col, ls=ls, label=lab)
for ax in axs:
    ax.axvline(2, color=GREY, ls=':', lw=1)
axs[0].set_ylabel(r'$\theta_0$ [deg]'); axs[1].set_ylabel(r'$\theta_1$ [deg]'); axs[1].set_xlabel('Time [s]')
axs[0].legend(loc='upper right', ncol=2, frameon=False); axs[1].set_xlim(0, 6)
axs[1].annotate('push', (2, axs[1].get_ylim()[0]), xytext=(3, 4), textcoords='offset points', fontsize=7.5, color=GREY)
fig.align_ylabels(axs); fig.tight_layout(h_pad=.4); fig.savefig(HERE / 'fig_lqr_fric.pdf'); plt.close(fig)
print('wrote fig_lqr_map.pdf, fig_lqr_surface.pdf, fig_lqr_corners.pdf, fig_lqr_delay.pdf, fig_lqr_fric.pdf')
