#!/usr/bin/env python3

'''
Validate a dataflash log from a flight of the
copter-drag-estimation-flight.lua applet.

The applet measures body-frame drag during the level drift phases of
its flight pattern and fits the EKF3 drag model

   accel = -sign(V)*(0.5*rho/BCOEF)*V^2 - MCOEF*(rho/1.225)*V

per body axis.  This tool independently repeats that analysis from
the logged data and compares the result against what the script
reported, so a bad fit can be caught before the saved EK3_DRAG_*
parameters are trusted:

 - drift phases are located from the applet's 1Hz "DRGE:" status
   messages in the MSG log stream
 - the wind for each drift is re-measured from the EKF velocity at
   the end of the drift (terminal velocity, moving with the air mass)
 - body-frame relative airspeed and specific force are re-derived
   from XKF1, ATT and IMU and re-fitted by least squares
 - measured acceleration is plotted against airspeed per axis, with
   the script's model and the re-fitted model overlaid, plus
   residual and time-series plots

Usage:
  copter_drag_estimation_validate.py LOG.BIN
  copter_drag_estimation_validate.py LOG.BIN --save-dir /tmp/out --no-show

AP_FLAKE8_CLEAN
'''

import argparse
import math
import os
import re
import sys

import numpy as np

from pymavlink import mavutil

RHO_SSL = 1.225
AIR_GAS_CONSTANT = 287.05
C_TO_KELVIN = 273.15


class Series(object):
    '''accumulates fields from one log message type, converts to numpy'''
    def __init__(self, fields):
        self.fields = fields
        self.data = {f: [] for f in fields}
        self.time = []

    def append(self, m):
        self.time.append(m.TimeUS * 1.0e-6)
        for f in self.fields:
            self.data[f].append(getattr(m, f))

    def finalise(self):
        self.time = np.array(self.time)
        for f in self.fields:
            self.data[f] = np.array(self.data[f])

    def __len__(self):
        return len(self.time)


class DriftSegment(object):
    def __init__(self, name, start, end):
        self.name = name
        self.start = start
        self.end = end
        # filled in by process_segment:
        self.wind = None      # (north, east) m/s
        self.time = None
        self.vx = None        # body-frame relative airspeed, filtered
        self.vy = None
        self.ax = None        # body-frame specific force, filtered
        self.ay = None


class LogData(object):
    def __init__(self):
        self.imu = Series(['AccX', 'AccY'])
        self.att = Series(['Roll', 'Pitch', 'Yaw'])
        self.vel = Series(['VN', 'VE'])
        self.wind_est = Series(['VWN', 'VWE'])
        self.baro = Series(['Press', 'Temp'])
        self.states = []          # (time, statename) from DRGE 1Hz messages
        self.script_wind = []     # (time, north, east)
        self.script_fit = {}      # BCOEF_X/BCOEF_Y/MCOEF as reported
        self.script_diag = {}     # bias/rho diagnostics
        self.params = {}          # last-logged EK3_DRAG_* values
        self.params_saved_time = None


RE_STATE = re.compile(r"DRGE: (initial drift|run \d+: (?:drive|drift)) accel")
RE_WIND = re.compile(r"DRGE: wind (\d+\.\d+)m/s from (\d+)deg")
RE_COEF = re.compile(r"DRGE: (BCOEF_X|BCOEF_Y|MCOEF)=([\d.-]+)")
RE_DIAG = re.compile(r"DRGE: bias x=([\d.-]+) y=([\d.-]+) rho=([\d.-]+)")


def load_log(filename):
    '''extract everything needed from the dataflash log'''
    data = LogData()
    mlog = mavutil.mavlink_connection(filename)
    types = set(['MSG', 'IMU', 'ATT', 'XKF1', 'XKF2', 'BARO', 'PARM'])
    while True:
        m = mlog.recv_match(type=types)
        if m is None:
            break
        mtype = m.get_type()
        if mtype == 'MSG':
            text = m.Message
            if not text.startswith('DRGE: '):
                continue
            now = m.TimeUS * 1.0e-6
            mstate = RE_STATE.match(text)
            if mstate is not None:
                data.states.append((now, mstate.group(1)))
                continue
            mwind = RE_WIND.match(text)
            if mwind is not None:
                speed = float(mwind.group(1))
                from_deg = float(mwind.group(2))
                to_rad = math.radians(from_deg + 180)
                data.script_wind.append((now, speed * math.cos(to_rad), speed * math.sin(to_rad)))
                continue
            mcoef = RE_COEF.match(text)
            if mcoef is not None:
                data.script_fit[mcoef.group(1)] = float(mcoef.group(2))
                continue
            mdiag = RE_DIAG.match(text)
            if mdiag is not None:
                data.script_diag = {
                    'bias_x': float(mdiag.group(1)),
                    'bias_y': float(mdiag.group(2)),
                    'rho': float(mdiag.group(3)),
                }
                continue
            if text.startswith('DRGE: params saved'):
                data.params_saved_time = now
        elif mtype == 'IMU':
            if m.I == 0:
                data.imu.append(m)
        elif mtype == 'ATT':
            data.att.append(m)
        elif mtype == 'XKF1':
            if m.C == 0:
                data.vel.append(m)
        elif mtype == 'XKF2':
            if m.C == 0:
                data.wind_est.append(m)
        elif mtype == 'BARO':
            if m.I == 0:
                data.baro.append(m)
        elif mtype == 'PARM':
            if m.Name.startswith('EK3_DRAG_'):
                data.params[m.Name] = m.Value
    for s in (data.imu, data.att, data.vel, data.wind_est, data.baro):
        s.finalise()
    return data


def find_drift_segments(data, min_duration=4.0, gap=2.5):
    '''group the 1Hz drift state messages into time segments.  The
    state messages are emitted at 1Hz so the first drift message can
    be up to a second after the drift actually began; the preceding
    (drive) state message bounds the true start more tightly'''
    segments = []
    current = None
    prev_time = None
    for (now, name) in data.states:
        if 'drift' in name:
            if current is not None and name == current.name and now - current.end <= gap:
                current.end = now
            else:
                current = DriftSegment(name, now, now)
                if prev_time is not None and now - prev_time <= gap:
                    current.start = prev_time
                else:
                    current.start = now - 1.0
                segments.append(current)
        prev_time = now
    return [s for s in segments if s.end - s.start >= min_duration]


def euler_earth_to_body_xy(roll_deg, pitch_deg, yaw_deg, vn, ve, vd):
    '''rotate earth-frame vectors into body frame, returning x and y
    components.  Rotation is the transpose of the ZYX body-to-earth
    DCM built from the logged euler angles'''
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)
    bx = np.cos(p)*np.cos(y)*vn + np.cos(p)*np.sin(y)*ve - np.sin(p)*vd
    by = ((np.sin(r)*np.sin(p)*np.cos(y) - np.cos(r)*np.sin(y))*vn +
          (np.sin(r)*np.sin(p)*np.sin(y) + np.cos(r)*np.cos(y))*ve +
          np.sin(r)*np.cos(p)*vd)
    return bx, by


def lowpass(t, x, cutoff_hz):
    '''single-pole low-pass filter matching the applet's filtering'''
    y = np.empty_like(x)
    y[0] = x[0]
    rc = 1.0 / (2 * math.pi * cutoff_hz)
    for i in range(1, len(x)):
        dt = t[i] - t[i-1]
        alpha = dt / (dt + rc)
        y[i] = y[i-1] + (x[i] - y[i-1]) * alpha
    return y


def process_segment(data, seg, settle_time, lpf_hz, wind_window):
    '''derive filtered body-frame airspeed and specific force pairs
    for one drift segment'''
    mask = (data.imu.time >= seg.start + settle_time) & (data.imu.time <= seg.end)
    t = data.imu.time[mask]
    if len(t) < 10:
        return False
    accx = data.imu.data['AccX'][mask]
    accy = data.imu.data['AccY'][mask]

    vn = np.interp(t, data.vel.time, data.vel.data['VN'])
    ve = np.interp(t, data.vel.time, data.vel.data['VE'])
    roll = np.interp(t, data.att.time, data.att.data['Roll'])
    pitch = np.interp(t, data.att.time, data.att.data['Pitch'])
    yaw = np.interp(t, data.att.time, data.att.data['Yaw'])

    # wind: this drift ends at terminal velocity, moving with the air
    wind_mask = t >= t[-1] - wind_window
    seg.wind = (np.mean(vn[wind_mask]), np.mean(ve[wind_mask]))

    reln = vn - seg.wind[0]
    rele = ve - seg.wind[1]
    vx, vy = euler_earth_to_body_xy(roll, pitch, yaw, reln, rele, 0.0)

    seg.time = t
    seg.vx = lowpass(t, vx, lpf_hz)
    seg.vy = lowpass(t, vy, lpf_hz)
    seg.ax = lowpass(t, accx, lpf_hz)
    seg.ay = lowpass(t, accy, lpf_hz)
    return True


def model_accel(v, rho, bcoef, mcoef):
    '''the EKF3 drag model the applet inverts'''
    return -np.sign(v) * (0.5 * rho / bcoef) * v * v - mcoef * (rho / RHO_SSL) * v


def fit_axis(v, a, rho, spd_min):
    '''repeat the applet's least-squares fit: a = A*u + B*w + C'''
    mask = np.abs(v) >= spd_min
    if np.count_nonzero(mask) < 20:
        return None
    v = v[mask]
    a = a[mask]
    u = -0.5 * rho * v * np.abs(v)
    w = -(rho / RHO_SSL) * v
    M = np.column_stack([u, w, np.ones_like(u)])
    (A, B, C), _, _, _ = np.linalg.lstsq(M, a, rcond=None)
    if A <= 0:
        return None
    return {'bcoef': 1.0/A, 'mcoef': B, 'bias': C, 'n': len(v)}


def residual_stats(v, a, rho, bcoef, mcoef, spd_min):
    '''residuals of measured accel against a coefficient set, with a
    quadratic trend fit as an error-direction diagnostic'''
    mask = np.abs(v) >= spd_min
    v = v[mask]
    a = a[mask]
    resid = a - model_accel(v, rho, bcoef, mcoef)
    # trend: resid ~ c0 + c1*(-w) + c2*(-u); c1 => MCOEF error, c2 => 1/BCOEF error
    M = np.column_stack([np.ones_like(v), (rho / RHO_SSL) * v, 0.5 * rho * v * np.abs(v)])
    (c0, c1, c2), _, _, _ = np.linalg.lstsq(M, resid, rcond=None)
    return {
        'rms': math.sqrt(np.mean(resid*resid)),
        'mcoef_error': -c1,
        'inv_bcoef_error': -c2,
        'v': v,
        'resid': resid,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="dataflash log (.BIN) of the drag estimation flight")
    parser.add_argument("--spd-min", type=float, default=2.0, help="minimum airspeed for fit samples (DRGE_SPD_MIN)")
    parser.add_argument("--lpf-hz", type=float, default=2.0, help="low-pass filter corner for the signals")
    parser.add_argument("--settle-time", type=float, default=1.5, help="seconds to drop from the start of each drift")
    parser.add_argument("--wind-window", type=float, default=2.0, help="seconds of terminal drift averaged for the wind")
    parser.add_argument("--rho", type=float, default=None, help="air density override (kg/m^3)")
    parser.add_argument("--save-dir", default=None, help="directory to save plots into")
    parser.add_argument("--no-show", action='store_true', help="do not display plots interactively")
    args = parser.parse_args()

    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    data = load_log(args.log)
    if len(data.imu) == 0 or len(data.att) == 0 or len(data.vel) == 0:
        print("Log is missing IMU, ATT or XKF1 data")
        sys.exit(1)
    if len(data.states) == 0:
        print("No DRGE drift-phase messages found - is this a drag estimation flight log?")
        sys.exit(1)

    if args.rho is not None:
        rho = args.rho
    elif len(data.baro):
        rho = float(np.median(data.baro.data['Press'] /
                              (AIR_GAS_CONSTANT * (data.baro.data['Temp'] + C_TO_KELVIN))))
    else:
        rho = RHO_SSL
    print("Air density: %.3f kg/m^3%s" % (rho, " (override)" if args.rho is not None else ""))
    if data.script_diag.get('rho') is not None:
        print("Script used: %.3f kg/m^3" % data.script_diag['rho'])

    segments = [s for s in find_drift_segments(data)
                if process_segment(data, s, args.settle_time, args.lpf_hz, args.wind_window)]
    if not segments:
        print("No usable drift segments found")
        sys.exit(1)

    print("\nDrift segments:")
    for s in segments:
        wspd = math.hypot(*s.wind)
        wdir = (math.degrees(math.atan2(s.wind[1], s.wind[0])) + 180) % 360
        print("  %-16s %7.1f-%7.1fs  wind %.1fm/s from %.0fdeg" %
              (s.name, s.start, s.end, wspd, wdir))
    if data.script_wind:
        spds = [math.hypot(n, e) for (_, n, e) in data.script_wind]
        print("Script wind measurements: %s m/s" % ", ".join("%.1f" % s for s in spds))
        if max(spds) - min(spds) > 1.0:
            print("WARNING: wind measurements spread over %.1fm/s - gusty air, distrust the fit" %
                  (max(spds) - min(spds)))

    vx = np.concatenate([s.vx for s in segments])
    ax_ = np.concatenate([s.ax for s in segments])
    vy = np.concatenate([s.vy for s in segments])
    ay = np.concatenate([s.ay for s in segments])

    refit = {
        'x': fit_axis(vx, ax_, rho, args.spd_min),
        'y': fit_axis(vy, ay, rho, args.spd_min),
    }

    coeff_sets = []
    if len(data.script_fit) == 3:
        coeff_sets.append(('script', data.script_fit['BCOEF_X'], data.script_fit['BCOEF_Y'],
                           data.script_fit['MCOEF']))
    if all(k in data.params for k in ('EK3_DRAG_BCOEF_X', 'EK3_DRAG_BCOEF_Y', 'EK3_DRAG_MCOEF')):
        if data.params['EK3_DRAG_BCOEF_X'] > 1:
            coeff_sets.append(('params', data.params['EK3_DRAG_BCOEF_X'],
                               data.params['EK3_DRAG_BCOEF_Y'], data.params['EK3_DRAG_MCOEF']))
    if refit['x'] is not None and refit['y'] is not None:
        coeff_sets.append(('re-fit', refit['x']['bcoef'], refit['y']['bcoef'],
                           0.5 * (refit['x']['mcoef'] + refit['y']['mcoef'])))

    print("\n%-8s %9s %9s %8s" % ("source", "BCOEF_X", "BCOEF_Y", "MCOEF"))
    for (name, bx, by, mc) in coeff_sets:
        print("%-8s %9.2f %9.2f %8.3f" % (name, bx, by, mc))
    if refit['x'] is not None:
        print("re-fit per-axis mcoef: x=%.3f y=%.3f bias: x=%.3f y=%.3f" %
              (refit['x']['mcoef'], refit['y']['mcoef'], refit['x']['bias'], refit['y']['bias']))

    if not coeff_sets:
        print("No coefficients to validate against (script fit not in log, params unset)")
        sys.exit(1)

    # residuals of the measured data against each coefficient set
    print("\nresiduals of measured accel against each model (m/s/s RMS, error direction):")
    stats = {}
    for (name, bx, by, mc) in coeff_sets:
        sx = residual_stats(vx, ax_, rho, bx, mc, args.spd_min)
        sy = residual_stats(vy, ay, rho, by, mc, args.spd_min)
        stats[name] = (sx, sy)
        print("%-8s X: rms=%.3f dMCOEF=%+.3f dBCOEF=%+.1f | Y: rms=%.3f dMCOEF=%+.3f dBCOEF=%+.1f" %
              (name,
               sx['rms'], sx['mcoef_error'],
               -sx['inv_bcoef_error'] * bx * bx if sx['inv_bcoef_error'] else 0,
               sy['rms'], sy['mcoef_error'],
               -sy['inv_bcoef_error'] * by * by if sy['inv_bcoef_error'] else 0))

    primary = coeff_sets[0]
    if refit['x'] is not None and primary[0] != 're-fit':
        db = 100 * abs(refit['x']['bcoef'] - primary[1]) / primary[1]
        print("\nre-fit vs %s: BCOEF_X differs by %.0f%%" % (primary[0], db))
        if db > 25:
            print("WARNING: independent re-fit disagrees with the script's fit")

    # ---- plots ----
    figs = []

    fig, axes = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    figs.append(('drag_curves', fig))
    for (axis_i, axis_name, v, a) in ((0, 'X', vx, ax_), (1, 'Y', vy, ay)):
        pax = axes[axis_i]
        for s in segments:
            sv = s.vx if axis_name == 'X' else s.vy
            sa = s.ax if axis_name == 'X' else s.ay
            pax.plot(sv, sa, '.', markersize=2, label=s.name)
        vv = np.linspace(np.min(v), np.max(v), 200)
        for (name, bx, by, mc) in coeff_sets:
            bc = bx if axis_name == 'X' else by
            pax.plot(vv, model_accel(vv, rho, bc, mc), label="%s BC=%.1f MC=%.3f" % (name, bc, mc))
        pax.axvspan(-args.spd_min, args.spd_min, color='grey', alpha=0.15)
        pax.set_ylabel('body %s specific force (m/s/s)' % axis_name)
        pax.grid(True)
        pax.legend(fontsize='small')
    axes[1].set_xlabel('body-frame relative airspeed (m/s)')
    fig.suptitle('measured drag vs airspeed (grey band excluded from fit)')

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    figs.append(('residuals', fig))
    name = primary[0]
    for (axis_i, axis_name) in ((0, 'X'), (1, 'Y')):
        st = stats[name][axis_i]
        axes[axis_i].plot(st['v'], st['resid'], '.', markersize=2)
        axes[axis_i].axhline(0, color='k', linewidth=0.5)
        axes[axis_i].set_ylabel('%s residual (m/s/s)' % axis_name)
        axes[axis_i].grid(True)
    axes[1].set_xlabel('body-frame relative airspeed (m/s)')
    fig.suptitle('measured minus %s model (want: flat, zero-mean)' % name)

    ncols = 2
    nrows = (len(segments) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3 * nrows), squeeze=False)
    figs.append(('timeseries', fig))
    for i, s in enumerate(segments):
        pax = axes[i // ncols][i % ncols]
        # plot whichever axis carries the drag for this segment
        if np.max(np.abs(s.vx)) >= np.max(np.abs(s.vy)):
            v, a, axis_name = s.vx, s.ax, 'X'
        else:
            v, a, axis_name = s.vy, s.ay, 'Y'
        (name, bx, by, mc) = primary
        bc = bx if axis_name == 'X' else by
        pax.plot(s.time - s.start, a, label='measured Acc%s' % axis_name)
        pax.plot(s.time - s.start, model_accel(v, rho, bc, mc), label='%s model' % name)
        pax.set_title("%s (%s axis)" % (s.name, axis_name), fontsize='small')
        pax.grid(True)
        pax.legend(fontsize='x-small')
    for i in range(len(segments), nrows * ncols):
        axes[i // ncols][i % ncols].set_axis_off()
    fig.suptitle('drift deceleration: measured vs model')
    fig.tight_layout()

    if len(data.wind_est) and data.params_saved_time is not None:
        fig, pax = plt.subplots(figsize=(10, 5))
        figs.append(('wind_estimate', fig))
        spd = np.hypot(data.wind_est.data['VWN'], data.wind_est.data['VWE'])
        pax.plot(data.wind_est.time, spd, label='EKF wind estimate (XKF2)')
        for s in segments:
            pax.plot(s.end, math.hypot(*s.wind), 'rx')
        pax.plot([], [], 'rx', label='terminal drift wind (truth)')
        pax.axvline(data.params_saved_time, color='g', linestyle='--', label='params saved')
        pax.set_xlabel('time (s)')
        pax.set_ylabel('wind speed (m/s)')
        pax.grid(True)
        pax.legend(fontsize='small')
        fig.suptitle('EKF wind estimate (fusion starts when params saved)')

    if args.save_dir is not None:
        os.makedirs(args.save_dir, exist_ok=True)
        base = os.path.splitext(os.path.basename(args.log))[0]
        for (name, fig) in figs:
            path = os.path.join(args.save_dir, "%s-%s.png" % (base, name))
            fig.savefig(path, dpi=100)
            print("Saved %s" % path)
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
