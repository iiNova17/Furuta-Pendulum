"""Physical-unit signals for the dashboard.

Pure functions (no Qt) so they are unit-tested and shared by live data and
saved recordings. Every conversion uses the device configuration; nothing is
hard-coded except the AS5600 resolution (4096 counts/turn).

Conventions (course): pendulum 0 deg = upright, 180 deg = hanging; arm angle
positive in the armSign direction from the marked center.
"""
import math
import numpy as np
from scipy import signal as sp

PEND_COUNTS = 4096


class Spec:
    def __init__(self, key, label, unit, raw_unit, color, group):
        self.key, self.label, self.unit, self.raw_unit, self.color, self.group = key, label, unit, raw_unit, color, group


SPECS = [
    Spec('arm_deg', 'Arm angle', 'deg', 'counts', '#1f6fb4', 'Angles'),
    Spec('pend_deg', 'Pendulum angle', 'deg', 'raw 0-4095', '#c2551c', 'Angles'),
    Spec('arm_speed', 'Arm speed', 'deg/s', 'counts/s', '#4a9bd8', 'Speeds'),
    Spec('pend_speed', 'Pendulum speed', 'deg/s', 'raw/s', '#e08a4f', 'Speeds'),
    Spec('current', 'Current (raw)', 'A', 'mV', '#9aa6b2', 'Current'),
    Spec('current_smooth', 'Current (smoothed)', 'A', 'mV', '#7b3fa0', 'Current'),
    Spec('current_mcu', 'Current (MCU filter)', 'A', 'mV', '#2a8f62', 'Current'),
    Spec('requested', 'Requested duty', '%', 'fraction', '#b8860b', 'Drive'),
    Spec('applied', 'Applied duty', '%', 'fraction', '#b52931', 'Drive'),
    Spec('mag_status', 'Magnet status', 'bits', 'bits', '#5d6d7e', 'Magnet'),
    Spec('agc', 'Magnet AGC', 'code', 'code', '#8e44ad', 'Magnet'),
    Spec('ctrl', 'Onboard controller state', 'code', 'code', '#1f7a4d', 'Onboard'),
]
BY_KEY = {s.key: s for s in SPECS}
DEFAULT_KEYS = ['arm_deg', 'pend_deg', 'current_smooth', 'applied']


def finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def butter_delay_ms(fc):
    """Low-frequency group delay of a 2nd-order Butterworth low-pass: sqrt(2)/(2*pi*fc)."""
    return math.sqrt(2) / (2 * math.pi * fc) * 1000


def lowpass(t, x, fc):
    """Causal 2nd-order Butterworth at fc on (approximately) uniform samples.

    Returns (y, applied). When fc is at/above 0.45*fs the input is returned
    unchanged and applied=False.
    """
    x = np.asarray(x, float)
    if len(x) < 4 or not fc or fc <= 0:
        return x, False
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if not len(dt):
        return x, False
    fs = 1 / float(np.median(dt))
    if fc >= .45 * fs:
        return x, False
    good = np.isfinite(x)
    if not good.any():
        return x, False
    filled = np.interp(np.arange(len(x)), np.flatnonzero(good), x[good])
    sos = sp.butter(2, fc, fs=fs, output='sos')
    y, _ = sp.sosfilt(sos, filled, zi=sp.sosfilt_zi(sos) * filled[0])
    y[~good] = np.nan
    return y, True


def derivative(t, x):
    if len(x) < 3:
        return np.full(len(x), np.nan)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.gradient(np.asarray(x, float), t)


def wrap_deg(x):
    """Wrap to (-180, 180]."""
    y = np.mod(np.asarray(x, float) + 180, 360) - 180
    return np.where(y == -180, 180., y)


class Series:
    def __init__(self, spec, values, unit, note=''):
        self.spec, self.values, self.unit, self.note = spec, np.asarray(values, float), unit, note

    @property
    def label(self):
        return self.spec.label


def compute(rows, config, center=None, centered=False, raw=False, smooth_hz=20., scale_verified=False):
    """Convert core rows (wire.CORE) to a dict key -> Series plus 't' (s from first row).

    center: arm center count (device centerCount). When not centered the arm
    angle is relative to the first row and labelled as such.
    """
    config = config or {}
    out = {}
    if rows is None or not len(rows):
        return np.empty(0), out
    t = (rows['device_us'].astype(np.int64) - int(rows['device_us'][0])).astype(float) * 1e-6
    count = rows['count'].astype(np.int64)
    pend_raw = rows['pend_raw'].astype(float)
    cpr = config.get('countsPerRev')
    arm_sign = config.get('armSign', 1) or 1
    pend_sign = config.get('pendSign', 1) or 1
    down = config.get('pendDownRaw')
    zero = config.get('currentZeroMv')
    sens = config.get('sensitivityMvA')
    ftype = (config.get('filter') or {}).get('type', 'bypass')

    # Arm angle
    if centered and center is not None:
        rel = (count - int(center)).astype(float); arm_note = ''
    else:
        rel = (count - count[0]).astype(float); arm_note = 'not centered: relative to first sample'
    if raw or not finite(cpr) or cpr <= 0:
        arm = rel; arm_unit = 'counts'
        if not raw and not finite(cpr):
            arm_note = '; '.join(filter(None, [arm_note, 'no counts/rev in profile']))
    else:
        arm = rel * 360. / cpr * arm_sign; arm_unit = 'deg'
        if not scale_verified:
            arm_note = '; '.join(filter(None, [arm_note, 'scale unverified']))
    out['arm_deg'] = Series(BY_KEY['arm_deg'], arm, arm_unit, arm_note)

    # Pendulum angle, 0 = upright
    pend_valid = (rows['validity'] & 2) != 0
    if raw:
        pend = pend_raw.copy(); pend_unit = 'raw'; pend_note = ''
        unwrapped = np.unwrap(pend_raw, period=PEND_COUNTS); speed_unit = 'raw/s'
    elif finite(down) and down >= 0:
        pend = wrap_deg((pend_raw - down) * 360. / PEND_COUNTS * pend_sign + 180)
        pend_unit = 'deg'; pend_note = ''
        unwrapped = np.unwrap(pend * math.pi / 180) * 180 / math.pi; speed_unit = 'deg/s'
    else:
        pend = pend_raw * 360. / PEND_COUNTS; pend_unit = 'deg'; pend_note = 'hanging-down reference not set: sensor angle'
        unwrapped = np.unwrap(pend * math.pi / 180) * 180 / math.pi; speed_unit = 'deg/s'
    pend = np.where(pend_valid, pend, np.nan)
    out['pend_deg'] = Series(BY_KEY['pend_deg'], pend, pend_unit, pend_note)

    # Speeds: derivative then the same software smoothing as current.
    smooth_note = f'smoothed {smooth_hz:g} Hz' if smooth_hz else ''
    arm_speed, ok = lowpass(t, derivative(t, arm), smooth_hz)
    out['arm_speed'] = Series(BY_KEY['arm_speed'], arm_speed, arm_unit + '/s', smooth_note if ok else 'unsmoothed')
    pend_speed, ok = lowpass(t, derivative(t, unwrapped), smooth_hz)
    pend_speed = np.where(pend_valid, pend_speed, np.nan)
    out['pend_speed'] = Series(BY_KEY['pend_speed'], pend_speed, speed_unit, smooth_note if ok else 'unsmoothed')

    # Current
    mv = rows['current_mv'].astype(float)
    fmv = rows['filtered_mv'].astype(float)
    amps = not raw and finite(zero) and finite(sens) and sens > 0
    if amps:
        cur = (mv - zero) / sens; mcu = (fmv - zero) / sens; cur_unit = 'A'; cur_note = ''
    else:
        cur = mv; mcu = fmv; cur_unit = 'mV'; cur_note = '' if raw else 'current zero not set'
    out['current'] = Series(BY_KEY['current'], cur, cur_unit, cur_note)
    sm, ok = lowpass(t, cur, smooth_hz)
    out['current_smooth'] = Series(BY_KEY['current_smooth'], sm, cur_unit,
                                   '; '.join(filter(None, [cur_note, smooth_note if ok else 'cutoff above Nyquist: unsmoothed'])))
    out['current_mcu'] = Series(BY_KEY['current_mcu'], mcu, cur_unit,
                                '; '.join(filter(None, [cur_note, 'MCU filter is bypass (= raw)' if ftype == 'bypass' else f'MCU {ftype}'])))

    # Drive
    k = 1 if raw else 100
    out['requested'] = Series(BY_KEY['requested'], rows['requested'].astype(float) * k, 'fraction' if raw else '%')
    out['applied'] = Series(BY_KEY['applied'], rows['applied'].astype(float) * k, 'fraction' if raw else '%')

    # Magnet
    out['mag_status'] = Series(BY_KEY['mag_status'], rows['mag_status'].astype(float), 'bits', 'MD=0x20 ML=0x10 MH=0x08')
    out['agc'] = Series(BY_KEY['agc'], rows['agc'].astype(float), 'code')

    # Onboard controller state (firmware 1.6; older records carry 0 in this byte)
    ctrl = rows['ctrl'].astype(float) if 'ctrl' in (rows.dtype.names or ()) else np.zeros(len(rows))
    out['ctrl'] = Series(BY_KEY['ctrl'], ctrl, 'code', '0 off · 1 SafeStop · 2 SwingUp · 3 Stabilize')
    return t, out


def best_current_key(config):
    """Prefer the MCU-filtered current when a firmware filter is active."""
    return 'current_mcu' if (config or {}).get('filter', {}).get('type', 'bypass') != 'bypass' else 'current_smooth'


def stats(t, x, t0=None, t1=None):
    """Mean and sample SD of finite x within [t0, t1]."""
    x = np.asarray(x, float)
    sel = np.isfinite(x)
    if t0 is not None:
        sel &= t >= t0
    if t1 is not None:
        sel &= t <= t1
    v = x[sel]
    if not len(v):
        return None, None
    return float(np.mean(v)), (float(np.std(v, ddof=1)) if len(v) > 1 else 0.)


def angle_stats(t, x, t0=None, t1=None):
    """Circular mean (wrapped to (-180, 180]) and SD of wrapped deviations, in degrees.

    Needed because a hanging pendulum (180 deg) crosses the +-180 wrap point.
    """
    x = np.asarray(x, float)
    sel = np.isfinite(x)
    if t0 is not None:
        sel &= t >= t0
    if t1 is not None:
        sel &= t <= t1
    v = x[sel] * math.pi / 180
    if not len(v):
        return None, None
    m = math.atan2(float(np.mean(np.sin(v))), float(np.mean(np.cos(v))))
    dev = wrap_deg((v - m) * 180 / math.pi)
    return float(wrap_deg(np.array([m * 180 / math.pi]))[0]), (float(np.std(dev, ddof=1)) if len(v) > 1 else 0.)


def continuous_angle(t, y):
    """Plot helper: re-wrap an angle to (m-180, m+180] around its circular mean m.

    A hanging pendulum sits at the +-180 wrap point; in (-180, 180] it would
    split into an upper and a lower band. Around its own mean it is drawn as
    one continuous trace (e.g. 179.9 and 180.1 instead of 179.9 and -179.9).
    Values are the same angles modulo 360; readouts keep (-180, 180].
    """
    m, _ = angle_stats(t, y)
    if m is None:
        return np.asarray(y, float)
    return m + wrap_deg(np.asarray(y, float) - m)


RAW_FIELDS = ['device_us', 'seq', 'count', 'pend_raw', 'adc_raw', 'current_mv', 'filtered_mv', 'requested', 'applied',
              'mag_status', 'agc', 'magnitude', 'validity', 'limits', 'state', 'pend_age_us', 'adc_age_us', 'ctrl']
HEX_FIELDS = {'mag_status', 'validity', 'limits'}


def field(row, name):
    """row[name], or 0 for a field older recordings do not have (e.g. 'ctrl' before firmware 1.6)."""
    return row[name] if name in (row.dtype.names or ()) else 0


def raw_text(name, value):
    if name in HEX_FIELDS:
        return f'0x{int(value):02x}'
    if isinstance(value, (float, np.floating)):
        return f'{float(value):.3f}'
    return str(int(value))


def export_csv(path, rows, series, t):
    """All raw fields plus every converted signal, one row per sample."""
    import csv
    keys = [k for k in BY_KEY if k in series]
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_s'] + RAW_FIELDS + [f'{k} [{series[k].unit}]' for k in keys])
        for i in range(len(rows)):
            r = rows[i]
            w.writerow([f'{t[i]:.6f}'] + [raw_text(n, field(r, n)) for n in RAW_FIELDS] +
                       ['' if not np.isfinite(series[k].values[i]) else f'{series[k].values[i]:.6g}' for k in keys])


def connect_mask(y, wrapping=False):
    """pyqtgraph connect array: no line across NaN or across a +-180 wrap."""
    y = np.asarray(y, float)
    if len(y) < 2:
        return 'finite'
    c = np.isfinite(y[:-1]) & np.isfinite(y[1:])
    if wrapping:
        c &= np.abs(np.diff(y)) < 180
    return np.r_[c, False]


def fmt(mean, sd, unit):
    """'0.04 ± 0.06 A' with decimals chosen from the SD."""
    if mean is None:
        return '—'
    if not sd:
        return f'{mean:.6g} ± 0 {unit}'.strip()
    scale = sd if sd and sd > 0 else max(abs(mean), 1e-9) * 1e-3
    exponent = math.floor(math.log10(scale))
    # One significant digit of the SD (two when it starts with 1 or 2).
    decimals = int(min(4, max(0, -exponent + (1 if scale / 10 ** exponent < 2.5 else 0))))
    return f'{mean:.{decimals}f} ± {sd:.{decimals}f} {unit}'.strip()


def magnet_text(status):
    status = int(status)
    if not status & 0x20:
        return 'no magnet'
    if status & 0x10:
        return 'weak'
    if status & 0x08:
        return 'strong'
    return 'normal'


def normalize(x):
    """Map to [-1, 1] around the mean for the combined view. Returns (y, half_range)."""
    x = np.asarray(x, float)
    f = x[np.isfinite(x)]
    if not len(f):
        return x, 0.
    m = float(np.mean(f)); h = float(np.max(np.abs(f - m)))
    if h <= 0:
        return x - m, 0.
    return (x - m) / h, h
