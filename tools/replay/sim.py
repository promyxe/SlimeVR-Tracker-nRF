#!/usr/bin/env python3
"""Monte Carlo drift simulation: realistic-ish sensors in a magnetically messy room.

Generates raw-collection CSVs (same format as receiver scripts/data_collect_hid.py),
runs vqf_replay on them and compares VQF's 9D heading against ground truth.

    python3 sim.py ./vqf_replay [--runs 6] [--minutes 10] [--scenario office] [--param name=value]... [--sensor name=value]...

Model (per run, randomized by seed). Numbers are order-of-magnitude
assumptions for an ICM-45686-class gyro and QMC6309-class mag, not datasheet
guarantees; edit SENSOR below to match measurements:
- motion: band-limited 3D body rates (~0.05-3 Hz) with still periods,
  plus a smooth random walk through a 4 x 4 m room at a fixed height;
- gyro: white noise, residual bias + random walk + warm-up temperature drift,
  per-axis scale error and small misalignment;
- accel: white noise + small bias (linear acceleration from motion included);
- mag: Earth field + magnetic dipoles (furniture, floor rebar) evaluated at the
  tracker position, residual hard/soft iron, white noise; 50 Hz.
"""
import argparse
import csv
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np

HZ = 400.0
MAG_HZ = 50.0
EARTH = np.array([0.0, 0.20, -0.42])  # gauss, ENU, magnetic north on +y (mid-Europe-ish)

SENSOR = dict(
    gyro_noise_dps_rthz=0.004,
    gyro_bias_residual_dps=0.05,   # sigma of bias left after calibration
    gyro_bias_rw_dps_rts=0.0005,   # random walk
    gyro_tempco_dps_per_c=0.005,   # sigma per axis
    warmup_c=8.0, warmup_tau_s=300.0,
    gyro_scale_sigma=0.003, gyro_misalign_deg=0.1,
    accel_noise_g_rthz=80e-6, accel_bias_g=0.005,
    # Residual calibration error dominates heading accuracy: 10 mG hard iron already gives
    # +-7 deg orientation-dependent heading error (try --sensor mag_hard_iron_g=0.01).
    mag_noise_g=0.003, mag_hard_iron_g=0.003, mag_soft_iron=0.003,
)

# Dipoles: (position m, moment G*m^3). Field = (3(m.r)r/|r|^2 - m)/|r|^3.
SCENARIOS = {
    "clean": dict(height=1.0, dipoles=[]),
    "office": dict(height=1.0, dipoles=[
        ((0.5, 3.5, 0.7), (0.0, 0.0, 0.03)),    # steel desk
        ((3.8, 0.4, 0.5), (0.02, 0.0, 0.01)),   # radiator
        ((2.0, 4.2, 1.2), (0.0, -0.015, 0.0)),  # shelf
    ]),
    "feet": dict(height=0.10, dipoles=[       # rebar grid under the floor
        ((x + 0.25, y + 0.25, -0.15), (0.0, 0.0, 0.0012 * (1 if (x + y) % 2 == 0 else -0.6)))
        for x in range(0, 4) for y in range(0, 4)
    ]),
}
DESK = ((0.5, 3.5, 0.7), (0.0, 0.0, 0.03))
# Effects the static-field scenarios above do not have (all assumptions, tune to measurements):
#   mains:  50 Hz field from a cable/power strip, amplitude at 0.3 m, ~1/r falloff. Mag sampling at
#           50 Hz aliases it into a slow beat (mains vs crystal mismatch 0.05%: 40 s period).
#   pocket: body-fixed magnet (phone/keys) appearing and disappearing for 1-3 min at a time.
#   magnet: body-fixed magnet always present (strap clip, mount), far above factory hard iron.
#   transient: a magnet passes close by every 30 s (world-fixed field pulse, ~2 s, 0.3-12 G peak,
#           strong ones saturate the 8 G range); info["events"] = [(center_s, peak_g)].
#   mag_tempco: QMC6309 offset drift per degree C during warm-up (body-fixed, like hard iron).
#   dwell:  stay near (x, y) for the whole run (sitting at the desk).
SCENARIOS.update({
    "mains": dict(height=0.5, dipoles=[DESK], mains=dict(at=(0.7, 3.3, 0.3), amp_g=0.005)),
    "pocket": dict(height=0.6, dipoles=[], pocket_g=0.03),
    "magnet": dict(height=0.6, dipoles=[], magnet_g=0.1),
    "transient": dict(height=0.6, dipoles=[], transient_every_s=30.0),
    "transient_ctrl": dict(height=0.6, dipoles=[], transient_every_s=30.0, transient_scale=0.0),  # same run, no field
    "warmup": dict(height=1.0, dipoles=[], mag_tempco_g=0.0005),
    "desk": dict(height=0.5, dipoles=[DESK], dwell=(0.9, 3.0), mains=dict(at=(0.7, 3.3, 0.3), amp_g=0.005)),
})
MAINS_HZ = 50.0 * (1 + 5e-4)  # relative to the tracker clock


def bandlimited(rng, n, fs, f_lo, f_hi, rms, dims=3):
    spec = rng.standard_normal((dims, n // 2 + 1)) + 1j * rng.standard_normal((dims, n // 2 + 1))
    f = np.fft.rfftfreq(n, 1 / fs)
    spec[:, (f < f_lo) | (f > f_hi)] = 0
    x = np.fft.irfft(spec, n)
    return (x / (x.std(axis=1, keepdims=True) + 1e-12) * rms).T


def activity_envelope(rng, n, fs):
    """1 while moving, 0 while still, smooth 0.5 s transitions."""
    env = np.zeros(n)
    i = 0
    moving = True
    while i < n:
        dur = rng.uniform(5, 60) if moving else rng.uniform(2, 20)
        j = min(n, i + int(dur * fs))
        env[i:j] = 1.0 if moving else 0.0
        moving = not moving
        i = j
    k = int(0.5 * fs)
    return np.convolve(env, np.ones(k) / k, mode="same")


def integrate(omega_rad, fs):
    """Body-rate quaternion integration q_{k+1} = q_k * exp(w dt / 2). Returns (n, 4) w,x,y,z."""
    n = len(omega_rad)
    q = np.empty((n, 4))
    w, x, y, z = 1.0, 0.0, 0.0, 0.0
    dt = 1.0 / fs
    for k in range(n):
        q[k] = (w, x, y, z)
        ox, oy, oz = omega_rad[k]
        ang = math.sqrt(ox * ox + oy * oy + oz * oz) * dt
        if ang > 1e-12:
            s = math.sin(ang / 2) / (ang / dt)
            c = math.cos(ang / 2)
            dx, dy, dz = ox * s, oy * s, oz * s
            w, x, y, z = (w * c - x * dx - y * dy - z * dz,
                          w * dx + x * c + y * dz - z * dy,
                          w * dy - x * dz + y * c + z * dx,
                          w * dz + x * dy - y * dx + z * c)
            norm = math.sqrt(w * w + x * x + y * y + z * z)
            w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return q


def rotmats(q):
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], -2)  # body -> world


def field_at(pos, dipoles):
    b = np.tile(EARTH, (len(pos), 1))
    for p, m in dipoles:
        r = pos - np.array(p)
        d = np.linalg.norm(r, axis=1, keepdims=True)
        m = np.array(m)
        b += (3 * (r @ m)[:, None] * r / d ** 2 - m) / d ** 3
    return b


def simulate(seed, minutes, scenario, path):
    rng = np.random.default_rng(seed)
    n = int(minutes * 60 * HZ)
    s = SENSOR
    t = np.arange(n) / HZ

    env = activity_envelope(rng, n, HZ)
    omega_dps = bandlimited(rng, n, HZ, 0.05, 3.0, 45.0) * env[:, None]
    q_true = integrate(np.radians(omega_dps), HZ)
    R = rotmats(q_true)

    pos = np.empty((n, 3))
    pos[:, :2] = 2.0 + bandlimited(rng, n, HZ, 0.005, 0.3, 0.9, dims=2) * np.clip(env, 0.2, 1)[:, None]
    pos[:, :2] = np.clip(pos[:, :2], 0.2, 3.8)
    sc = SCENARIOS[scenario]
    if "dwell" in sc:
        pos[:, :2] = np.array(sc["dwell"]) + (pos[:, :2] - 2.0) * 0.1  # small fidgeting
    pos[:, 2] = sc["height"]
    lin_acc = bandlimited(rng, n, HZ, 0.2, 5.0, 1.5) * env[:, None]  # m/s^2, world

    # gyro errors
    temp = 25 + s["warmup_c"] * (1 - np.exp(-t / s["warmup_tau_s"]))
    bias = (rng.normal(0, s["gyro_bias_residual_dps"], 3)
            + np.cumsum(rng.normal(0, s["gyro_bias_rw_dps_rts"] / math.sqrt(HZ), (n, 3)), axis=0)
            + (temp - 25)[:, None] * rng.normal(0, s["gyro_tempco_dps_per_c"], 3))
    mis = np.radians(s["gyro_misalign_deg"]) * rng.standard_normal((3, 3))
    gyro_mat = np.diag(1 + rng.normal(0, s["gyro_scale_sigma"], 3)) + (mis - np.diag(np.diag(mis)))
    gyro = omega_dps @ gyro_mat.T + bias + rng.normal(0, s["gyro_noise_dps_rthz"] * math.sqrt(HZ / 2), (n, 3))

    # accel (g): specific force in body frame
    f_world = (lin_acc + np.array([0, 0, 9.80665])) / 9.80665
    accel = np.einsum("nji,nj->ni", R, f_world) + rng.normal(0, s["accel_bias_g"], 3) \
        + rng.normal(0, s["accel_noise_g_rthz"] * math.sqrt(HZ / 2), (n, 3))

    # mag (gauss): world field at position, into body, then residual calibration error
    every = int(HZ / MAG_HZ)
    idx = np.arange(0, n, every)
    b_world = field_at(pos[idx], SCENARIOS[scenario]["dipoles"])
    b_body = np.einsum("nji,nj->ni", R[idx], b_world)
    soft = np.eye(3) + rng.normal(0, s["mag_soft_iron"], (3, 3))
    hard = rng.normal(0, s["mag_hard_iron_g"], 3)
    mag = np.zeros((n, 3))
    body_extra = np.zeros((len(idx), 3))
    magnet_on = []  # (start_s, end_s) of the pocket magnet
    if "mains" in sc:
        r = np.linalg.norm(pos[idx] - np.array(sc["mains"]["at"]), axis=1)
        amp = sc["mains"]["amp_g"] * np.minimum(0.3 / np.maximum(r, 0.05), 6.0)
        direction = np.array([0.6, 0.6, 0.53])
        hum = (amp * np.sin(2 * np.pi * MAINS_HZ * t[idx] + rng.uniform(0, 2 * np.pi)))[:, None] * direction
        b_body += np.einsum("nji,nj->ni", R[idx], hum)
    if "pocket_g" in sc:
        on = np.zeros(len(idx), bool)
        k, state = 0, False
        while k < len(idx):
            dur = int(rng.uniform(60, 180) * MAG_HZ)
            on[k:k + dur] = state
            if state:
                magnet_on.append((k / MAG_HZ, min(k + dur, len(idx)) / MAG_HZ))
            state, k = not state, k + dur
        body_extra += on[:, None] * sc["pocket_g"] * np.array([0.7, -0.5, 0.5])
    if "magnet_g" in sc:
        body_extra += sc["magnet_g"] * np.array([-0.4, 0.8, 0.45])
    if "mag_tempco_g" in sc:
        body_extra += (temp[idx] - 25)[:, None] * sc["mag_tempco_g"] * rng.normal(0, 1, 3)
    events = []
    if "transient_every_s" in sc:
        tm = t[idx]
        world = np.zeros((len(idx), 3))
        for tc in np.arange(40.0, tm[-1] - 10, sc["transient_every_s"]):
            peak = float(np.exp(rng.uniform(np.log(0.3), np.log(12.0))))
            d = rng.normal(0, 1, 3)
            world += (sc.get("transient_scale", 1.0) * peak * np.exp(-((tm - tc) / 0.7) ** 2))[:, None] * (d / np.linalg.norm(d))
            events.append((float(tc), peak))
        b_body += np.einsum("nji,nj->ni", R[idx], world)
    mag[idx] = b_body @ soft.T + hard + body_extra + rng.normal(0, s["mag_noise_g"], (len(idx), 3))

    seq = np.arange(n) & 0xFFFF
    data = np.column_stack([seq, gyro, accel, mag, temp, np.zeros(n)])
    with open(path, "w") as f:
        f.write("seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp,discontinuity\n")
        np.savetxt(f, data, fmt=["%d"] + ["%.6f"] * 10 + ["%d"], delimiter=",")
    Path(path).with_suffix(".meta.txt").write_text(f"gyro_odr_hz={HZ}\nmag_odr_hz={MAG_HZ}\n")
    still = float((env < 0.01).mean())
    dist_deg = np.degrees(np.arctan2(b_world[:, 0], b_world[:, 1]))
    return q_true, dict(still=still, field_dir_spread=float(dist_deg.std()),
                        field_norm_spread=float(np.linalg.norm(b_world, axis=1).std() / np.linalg.norm(EARTH)),
                        magnet_on=magnet_on, events=events)


def heading_error(q_est, q_true):
    """Yaw of the world-frame error rotation q_est * conj(q_true), degrees."""
    w1, x1, y1, z1 = q_est.T
    w2, x2, y2, z2 = q_true.T
    x2, y2, z2 = -x2, -y2, -z2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def evaluate(trace, q_true, settle_s=30.0):
    rows = np.loadtxt(trace, delimiter=",", skiprows=1)
    t = rows[:, 0]
    qt = q_true[np.round(t * HZ).astype(int)]
    out = {}
    for name, cols in (("vqf", slice(1, 5)),):
        e = heading_error(rows[:, cols], qt)
        keep = t >= settle_s
        ref = e[keep & (t < settle_s + 10)]
        offset = math.degrees(math.atan2(np.sin(np.radians(ref)).mean(), np.cos(np.radians(ref)).mean()))
        err = np.abs((e[keep] - offset + 180) % 360 - 180)
        out[name] = (err.max(), float(np.sqrt((err ** 2).mean())), err[-1])
    out["mag_disturbed"] = rows[:, 8].mean() * 100
    return out


def main():
    global MAG_HZ
    ap = argparse.ArgumentParser()
    ap.add_argument("tool")
    ap.add_argument("--runs", type=int, default=6)
    ap.add_argument("--minutes", type=float, default=10)
    ap.add_argument("--scenario", action="append", choices=sorted(SCENARIOS))
    ap.add_argument("--param", action="append", default=[], help="VQF parameter override name=value")
    ap.add_argument("--sensor", action="append", default=[], help="SENSOR model override name=value")
    ap.add_argument("--mag-hz", type=float, default=MAG_HZ, help="mag sample rate (firmware default 50)")
    ap.add_argument("--tool-arg", action="append", default=[], help="extra vqf_replay argument, e.g. --mag-track")
    args = ap.parse_args()
    MAG_HZ = args.mag_hz
    for kv in args.sensor:
        k, v = kv.split("=", 1)
        if k not in SENSOR:
            ap.error(f"unknown sensor parameter {k}")
        SENSOR[k] = float(v)
    tmp = Path(tempfile.mkdtemp(prefix="vqf_sim_"))
    extra = [a for p in args.param for a in ("--param", p)] + args.tool_arg

    print(f"{args.runs} runs x {args.minutes:g} min per scenario; heading error vs truth after 30 s, deg "
          f"(median over runs of: max / rms / final)")
    print(f"{'scenario':10s}{'still%':>7s}{'dir sd':>8s}{'norm sd':>8s}{'mag dist%':>10s}"
          f"{'VQF max/rms/end':>22s}")
    for scenario in args.scenario or ["clean", "office", "feet", "mains", "pocket", "warmup", "desk"]:
        res = []
        for seed in range(args.runs):
            path = tmp / f"{scenario}_{seed}.csv"
            q_true, info = simulate(seed, args.minutes, scenario, path)
            trace = str(path) + ".trace.csv"
            subprocess.run([args.tool, *extra, "--out", trace, str(path)], check=True, capture_output=True)
            r = evaluate(trace, q_true)
            res.append((info, r))
        med = lambda xs: float(np.median(xs))
        fmt = lambda k: "/".join(f"{med([r[k][i] for _, r in res]):.1f}" for i in range(3))
        print(f"{scenario:10s}{med([i['still'] for i, _ in res]) * 100:7.0f}"
              f"{med([i['field_dir_spread'] for i, _ in res]):8.1f}{med([i['field_norm_spread'] for i, _ in res]) * 100:7.0f}%"
              f"{med([r['mag_disturbed'] for _, r in res]):10.0f}{fmt('vqf'):>22s}")
        print(f"{'':10s} worst run max error: {max(r['vqf'][0] for _, r in res):.1f}")


if __name__ == "__main__":
    main()
