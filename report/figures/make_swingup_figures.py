"""Figures for the swing-up design section (section 10 of furuta_model.tex).

Reads swingup_data.mat, written by matlab/harness/controllers/hybrid/swingup_report_data.m from the
harness results (swingup_catch_region, swingup_sweep, swingup_corner_test), and writes
fig_swingup_catch.pdf, fig_swingup_catch3d.pdf, fig_swingup_run.pdf, fig_swingup_map.pdf, fig_swingup_surface.pdf and
fig_swingup_corners.pdf here.  Run from the repository root:  python report/figures/make_swingup_figures.py
"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize, ListedColormap
from matplotlib import cm as mcm
from matplotlib import colors as mcolors
from scipy.io import loadmat

HERE = Path(__file__).resolve().parent
NAVY, STEEL, LIGHT, TERRA, GREY, GREEN = '#143C6E', '#43638B', '#4C87AD', '#CC7451', '#888888', '#2d6a3e'
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['CMU Serif', 'Latin Modern Roman', 'Computer Modern Roman', 'DejaVu Serif'],
                     'mathtext.fontset': 'cm', 'font.size': 9, 'axes.labelsize': 9, 'legend.fontsize': 8, 'xtick.labelsize': 8,
                     'ytick.labelsize': 8, 'axes.grid': True, 'grid.color': '#dddddd', 'grid.linewidth': .6,
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.prop_cycle': matplotlib.cycler(color=[NAVY, TERRA, STEEL, LIGHT]),
                     'savefig.bbox': 'tight', 'savefig.pad_inches': .02, 'lines.linewidth': 1.1})
TW = 6.3
D = loadmat(HERE / 'swingup_data.mat', squeeze_me=True, struct_as_record=False)
BLUES = LinearSegmentedColormap.from_list('b', ['#e8eef6', LIGHT, NAVY])
WARM = LinearSegmentedColormap.from_list('w', ['#fbf3e6', '#f5e6b2', TERRA, '#7a2e14'])
F = D['final']
a_fin, wc_fin, E0 = float(F.a_push), float(F.wc), float(F.E0)
G, Jp = float(D['p'].G), float(D['p'].Jp)


def edges(v):
    v = np.asarray(v, float); d = np.diff(v)
    return np.concatenate([[v[0] - d[0] / 2], v[:-1] + d / 2, [v[-1] + d[-1] / 2]])


# ---- 1. catch region of the LQR with the energy curves and the hand-over states -------------
C = D['catch']; th, w = np.asarray(C.th1, float), np.asarray(C.w1, float)
held = np.asarray(C.held) == 1
trip = np.asarray(C.fault) > 0
fig, ax = plt.subplots(figsize=(TW, 3.4))
ax.pcolormesh(edges(th), edges(w), np.where(~held, 1.0, np.nan), cmap=ListedColormap(['#eeeeee']), shading='flat')
im = ax.pcolormesh(edges(th), edges(w), np.where(held, C.ipk, np.nan), cmap=WARM, vmin=0.4, vmax=1.8, shading='flat')
I, J = np.nonzero(trip)
ax.plot(th[J], w[I], 'x', color=GREY, ms=3.5, mew=.8, label='firmware trip')
tt = np.linspace(-40, 40, 400); c = np.cos(np.radians(tt))
for e, ls, lab in [(0.0, ':', r'$e = 0$'), (E0, '-', rf'$e = E_0 = {E0:.3f}$')]:
    wv = np.sqrt(np.maximum(4 * G / Jp * (e - (c - 1) / 2), 0))
    for side, lb in [(tt < 0, lab + ' (rising)'), (tt > 0, None)]:
        ax.plot(tt[side], -np.sign(tt[side]) * wv[side], ls, color=NAVY, lw=1.3, label=lb)
S = D['sup']; ct, cw = np.degrees(float(S.catch_th1)), float(S.catch_w1)
ax.plot([-ct, ct, ct, -ct, -ct], [-cw, -cw, cw, cw, -cw], '--', color=GREEN, lw=1.2, label='catch guard')
H = D['final_corners']
ok = np.asarray(H.pass_ if hasattr(H, 'pass_') else getattr(H, 'pass'), bool)
ax.plot(np.asarray(H.th1_catch)[ok], np.asarray(H.w1_catch)[ok], 'o', mfc='w', mec='k', ms=4.5, mew=1, label='hand-over, 12 worst cases')
ax.set_xlim(th[0] - 1.25, th[-1] + 1.25); ax.set_ylim(w[0] - .5, w[-1] + .5)
ax.set_xlabel(r'$\theta_1$ at hand-over [deg]'); ax.set_ylabel(r'$\dot\theta_1$ at hand-over [rad/s]'); ax.grid(False)
cb = fig.colorbar(im, ax=ax, fraction=.04, pad=.02); cb.set_label('Peak current, held [A]', fontsize=7.5); cb.ax.tick_params(labelsize=7)
ax.legend(loc='upper right', fontsize=6.8, framealpha=.95)
fig.tight_layout(); fig.savefig(HERE / 'fig_swingup_catch.pdf'); plt.close(fig)

# ---- 1b. the same catch region in 3D: height = peak current, colour = peak arm angle (held states) ----
TH, W = np.meshgrid(th, w)
ipk_c = np.asarray(C.ipk, float); th0_c = np.asarray(C.th0max, float)
norm = Normalize(0, 100)
fig = plt.figure(figsize=(TW, 4.6))
ax = fig.add_axes([-.02, -.04, .86, 1.08], projection='3d', computed_zorder=False)
floor = np.where(held, np.nan, 0.0)                                   # floor: light gray = falls, darker = trips
ax.plot_surface(TH, W, floor, facecolors=np.where(trip[..., None], mcolors.to_rgba('#b5b5b5'), mcolors.to_rgba('#ececec')),
                rstride=1, cstride=1, linewidth=0, shade=False, zorder=1)
for e, ls in [(0.0, ':'), (E0, '-')]:                                 # energy curves, rising pendulum
    wv = np.sqrt(np.maximum(4 * G / Jp * (e - (c - 1) / 2), 0))
    for side in (tt < 0, tt > 0):
        s_ = side & (wv <= w[-1])
        ax.plot(tt[s_], -np.sign(tt[s_]) * wv[s_], np.zeros(s_.sum()), ls, color=NAVY, lw=1.3, zorder=4)
ax.plot([-ct, ct, ct, -ct, -ct], [-cw, -cw, cw, cw, -cw], [0] * 5, '--', color=GREEN, lw=1.4, zorder=4)
Zh = np.where(held, ipk_c, np.nan)
ax.plot_surface(TH, W, Zh, facecolors=BLUES(norm(np.where(held, th0_c, 0))),
                rstride=1, cstride=1, linewidth=.3, edgecolor='#555555', shade=False, alpha=.95, zorder=5)
ax.contour(TH, W, np.ma.masked_invalid(Zh), levels=[1.5], colors=[TERRA], linewidths=1.6, linestyles='--', zorder=6)
hx, hy = np.asarray(H.th1_catch)[ok], np.asarray(H.w1_catch)[ok]
ax.scatter(hx, hy, np.zeros(len(hx)), marker='o', facecolor='w', edgecolor='k', s=18, linewidths=.9, depthshade=False, zorder=7)
ax.set_xlabel(r'$\theta_1$ at hand-over [deg]', labelpad=2); ax.set_ylabel(r'$\dot\theta_1$ at hand-over [rad/s]', labelpad=4)
ax.set_zlabel('Peak current, held [A]', labelpad=2)
ax.set_xlim(th[0], th[-1]); ax.set_ylim(w[0], w[-1]); ax.set_zlim(0, 2.0)
ax.view_init(elev=34, azim=-50); ax.set_box_aspect(None, zoom=.95); ax.tick_params(labelsize=7, pad=1)
sm = mcm.ScalarMappable(norm=norm, cmap=BLUES)
cb = fig.colorbar(sm, cax=fig.add_axes([.86, .2, .018, .6])); cb.set_label(r'Peak arm angle $|\theta_0|_{\max}$ [deg]', fontsize=7.5); cb.ax.tick_params(labelsize=7)
fig.savefig(HERE / 'fig_swingup_catch3d.pdf', bbox_inches='tight', pad_inches=.05); plt.close(fig)

# ---- 2. the final design, nominal run -------------------------------------------------------
R = D['run']; t = R.t; tc = float(R.tcap)
fig, axs = plt.subplots(3, 1, figsize=(TW, 4.6), sharex=True)
axs[0].plot(t, R.E, color=NAVY, label='true'); axs[0].plot(t, R.E_est, ':', color=TERRA, label='estimated')
axs[0].axhline(E0, ls='--', color=GREY, lw=.8); axs[0].text(5.95, E0 + .03, r'$E_0$', ha='right', fontsize=7.5)
axs[0].set_ylabel(r'$e = E/2\tilde G$'); axs[0].set_ylim(-1.1, .25); axs[0].legend(loc='center right')
axs[1].plot(t, R.th1, color=NAVY, label=r'$\theta_1$ (0 upright)'); axs[1].plot(t, R.th0, color=TERRA, label=r'$\theta_0$')
axs[1].set_ylabel('Angle [deg]'); axs[1].set_ylim(-190, 190); axs[1].set_yticks([-180, -90, 0, 90, 180]); axs[1].legend(loc='lower right', ncol=2)
axs[2].plot(t, R.i, color=STEEL, lw=.8); axs[2].axhline(1.5, ls='--', color='#7a2e14', lw=.9); axs[2].axhline(-1.5, ls='--', color='#7a2e14', lw=.9)
axs[2].set_ylabel('Current [A]'); axs[2].set_xlabel('Time [s]'); axs[2].set_xlim(0, 6)
for a in axs:
    a.axvline(tc, color=GREEN, lw=1)
axs[0].text(tc + .05, -.95, 'catch', color=GREEN, fontsize=7.5)
fig.tight_layout(h_pad=.4); fig.savefig(HERE / 'fig_swingup_run.pdf'); plt.close(fig)

# ---- 3. design map: time to catch and arm travel over (a_push, wc) --------------------------
M = D['map']; ap, wc = np.asarray(M.vals1, float), np.asarray(M.vals2, float)
P = np.asarray(getattr(M, 'pass'), bool)
fig, axs = plt.subplots(1, 2, figsize=(TW, 2.9))
for ax, Z, cmap, lab, fmt, dark, lim in [
        (axs[0], np.where(P, M.tcap, np.nan), BLUES, 'Time to catch [s]', '%.1f', lambda v: v > 2.6, (0.7, 4.8)),
        (axs[1], np.where(P, M.th0, np.nan), WARM, r'Largest $|\theta_0|$ [deg]', '%.0f', lambda v: v > 85, (20, 100))]:
    im = ax.imshow(Z, origin='lower', aspect='auto', cmap=cmap, vmin=lim[0], vmax=lim[1])
    for (r, cc), v in np.ndenumerate(Z):
        if np.isfinite(v):
            ax.text(cc, r, fmt % v, ha='center', va='center', fontsize=6.3, color='w' if dark(v) else '#222222')
    r, cc = np.nonzero(~P); ax.plot(cc, r, 'x', color=GREY, ms=5, mew=1.2)
    ax.set_xticks(range(len(ap))); ax.set_xticklabels([f'{v:g}' for v in ap])
    ax.set_yticks(range(len(wc))); ax.set_yticklabels([f'{v:g}' for v in wc])
    ax.set_xlabel(r'$a_p$ [rad/s$^2$]'); ax.set_ylabel(r'$\omega_c$ [rad/s]'); ax.grid(False)
    cb = fig.colorbar(im, ax=ax, fraction=.05, pad=.02); cb.set_label(lab, fontsize=7.5); cb.ax.tick_params(labelsize=7)
    ax.plot(int(np.argmin(abs(ap - a_fin))), int(np.argmin(abs(wc - wc_fin))), 'o', mfc='none', mec=GREEN, ms=12, mew=1.5)
fig.tight_layout(w_pad=1.2); fig.savefig(HERE / 'fig_swingup_map.pdf'); plt.close(fig)

# ---- 4. the same map in 3D: bar height = time to catch, colour = peak current --------------
norm = Normalize(0.5, 1.6)
fig = plt.figure(figsize=(TW * .9, 4.0)); ax = fig.add_subplot(projection='3d')
ipk = np.asarray(M.ipk, float); tcap = np.asarray(M.tcap, float)
da, dw = .7 * np.diff(ap).min(), .7 * np.diff(wc).min()
for r_ in range(len(wc)):
    for c_ in range(len(ap)):
        if P[r_, c_]:
            fin = np.isclose(ap[c_], a_fin) and np.isclose(wc[r_], wc_fin)
            ax.bar3d(ap[c_] - da / 2, wc[r_] - dw / 2, 0, da, dw, tcap[r_, c_], color=WARM(norm(ipk[r_, c_])),
                     edgecolor=GREEN if fin else '#555555', linewidth=1.4 if fin else .3, shade=True)
X, Y = np.meshgrid(ap, wc)
ax.scatter(X[~P], Y[~P], np.zeros(np.count_nonzero(~P)), marker='x', color=GREY, s=18, depthshade=False)
ax.set_xlabel(r'$a_p$ [rad/s$^2$]', labelpad=2); ax.set_ylabel(r'$\omega_c$ [rad/s]', labelpad=2); ax.set_zlabel('Time to catch [s]', labelpad=2)
ax.set_zlim(0, 5); ax.view_init(elev=30, azim=-60); ax.tick_params(labelsize=7, pad=0)
m = mcm.ScalarMappable(norm=norm, cmap=WARM); m.set_array([])
cb = fig.colorbar(m, ax=ax, fraction=.03, pad=.16, shrink=.6); cb.set_label('Peak current [A]', fontsize=7.5); cb.ax.tick_params(labelsize=7)
fig.savefig(HERE / 'fig_swingup_surface.pdf'); plt.close(fig)

# ---- 5. candidates in the 12 worst cases: arm travel and time to catch ----------------------
cands = [c for c in np.atleast_1d(D['cand']) if hasattr(c, 'th0')]
fig, axs = plt.subplots(1, 2, figsize=(TW, 2.6), sharey=True)
mk = {'nominal timing': 'o', '+1 tick delay': 's', 'sensor lag 10 ms': '^'}
labels = []
for k, c in enumerate(cands):
    p = np.asarray(getattr(c, 'pass'), bool); tim = list(np.atleast_1d(c.timing))
    labels.append(f'{float(c.a):g} / {float(c.wc):g}' if hasattr(c, 'a') else str(k + 1))
    for j in range(len(p)):
        y = k + (j % 4 - 1.5) * .12
        style = dict(marker=mk[tim[j]], ms=4)
        if p[j]:
            axs[0].plot(c.th0[j], y, ls='', mfc='w', mec=NAVY, mew=1, **style)
            axs[1].plot(c.tcap[j], y, ls='', mfc='w', mec=NAVY, mew=1, **style)
        else:
            axs[0].plot(min(c.th0[j], 125), y, ls='', color=TERRA, **style)
            axs[1].plot(np.nan_to_num(c.tcap[j], nan=8), y, ls='', color=TERRA, **style)
axs[0].axvline(100, ls='--', color='#7a2e14', lw=.9); axs[0].text(99, len(cands) - .45, 'SafeStop', ha='right', fontsize=7)
axs[0].set_xlabel(r'Largest $|\theta_0|$ [deg]'); axs[1].set_xlabel('Time to catch [s]')
axs[0].set_yticks(range(len(cands))); axs[0].set_yticklabels(labels); axs[0].set_ylabel(r'$a_p$ [rad/s$^2$] / $\omega_c$ [rad/s]')
for name, m_ in mk.items():
    axs[1].plot([], [], m_, mfc='w', mec=GREY, ls='', label=name)
axs[1].legend(loc='lower right', fontsize=6.5)
fig.tight_layout(w_pad=.8); fig.savefig(HERE / 'fig_swingup_corners.pdf'); plt.close(fig)
print('wrote fig_swingup_catch/run/map/surface/corners.pdf')
