#!/usr/bin/env python3
"""Evaluate a full-stack bsim run (receiver HID output vs tracker output vs truth).

    python3 fullstack.py rx_hid.csv --trk 1:trk1.csv[:truth.npy] [--trk 2:...] [--settle 60]

rx_hid.csv: receiver SLIME_SIM_OUT (t_us,report_hex), 64-byte reports of 4x16-byte records
            [type, tracker_id, payload...]. Types 1/4 carry the Q15 quaternion (x,y,z,w).
trkN.csv:   tracker SLIME_SIM_OUT (t_us,qw,qx,qy,qz); tracker id from "Tracker ID: x" in trkN.log,
            optional trkN.offset (us) = bsim -start_offset.
Latency = receiver HID time - time the tracker produced the closest quaternion.
"""
import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "replay"))
from sim import HZ, heading_error  # noqa: E402


def qmul(a, b):
    w1, x1, y1, z1 = a.T
    w2, x2, y2, z2 = b.T
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=1)


def truth_at(tr, s):
    """Truth quaternions at fractional sample index s (normalized lerp)."""
    i = np.clip(np.floor(s).astype(int), 0, len(tr) - 2)
    a = (s - i)[:, None]
    q0, q1 = tr[i], tr[i + 1]
    q1 = q1 * np.sign(np.sum(q0 * q1, axis=1))[:, None]
    q = (1 - a) * q0 + a * q1
    return q / np.linalg.norm(q, axis=1)[:, None]


def e2e_latency_ms(t, q, tr, off_s, step=5):
    """Lag (sample to receiver output) that best aligns body-frame rotation increments with truth."""
    conj = np.array([1, -1, -1, -1])
    d_rx = qmul(q[:-step] * conj, q[step:])
    moving = np.abs(d_rx[:, 0]) < np.cos(np.radians(1.0))  # > 2 deg rotation per increment
    if moving.sum() < 50:
        return None
    errs = []
    for lag in np.arange(0.0, 25.0, 0.25):
        qt = truth_at(tr, (t - off_s - lag / 1e3) * HZ)
        d_tr = qmul(qt[:-step] * conj, qt[step:])
        dot = np.clip(np.abs(np.sum(d_rx[moving] * d_tr[moving], axis=1)), 0, 1)
        errs.append((np.mean(np.arccos(dot)), lag))
    return min(errs)[1]


def decode(path, t0=0.0):
    quats, counts = defaultdict(list), defaultdict(lambda: defaultdict(int))
    seq = defaultdict(lambda: [0, 0])  # receiver sequence-gap stats from type 3: [received, lost]
    for line in Path(path).read_text().splitlines():
        t, _, hexs = line.partition(",")
        if not t.isdigit() or int(t) < t0 * 1e6:
            continue
        rep = bytes.fromhex(hexs)
        for i in range(0, len(rep) - 15, 16):
            rec = rep[i:i + 16]
            if rec[0] == 0xFF:
                continue
            counts[rec[1]][rec[0]] += 1
            if rec[0] == 3:
                seq[rec[1]][0] += rec[4]
                seq[rec[1]][1] += rec[5]
            if rec[0] in (1, 4):
                x, y, z, w = np.frombuffer(rec[2:10], "<i2")
                quats[rec[1]].append((int(t), int(w), int(x), int(y), int(z)))
    return quats, counts, seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rx_csv")
    ap.add_argument("--trk", action="append", default=[], help="N:trkN.csv[:truth.npy]")
    ap.add_argument("--settle", type=float, default=60.0)
    ap.add_argument("--from", dest="t0", type=float, default=0.0, help="link stats only after this time (s)")
    args = ap.parse_args()

    quats, counts, seq = decode(args.rx_csv, args.t0)
    for spec in args.trk:
        n, trk_csv, *truth = spec.split(":")
        ids = re.findall(r"Tracker ID: (\d+)", Path(trk_csv).with_suffix(".log").read_text(errors="ignore"))
        if not ids:
            print(f"trk{n}: never paired")
            continue
        tid = int(ids[-1])
        rx = np.array(quats.get(tid, []), dtype=float).reshape(-1, 5)
        trk = np.loadtxt(trk_csv, delimiter=",", skiprows=1)
        f = Path(trk_csv).with_suffix(".offset")  # bsim -start_offset: tracker clock lags the phy
        off = float(f.read_text()) if f.exists() else 0.0
        trk[:, 0] += off
        types = dict(counts.get(tid, {}))
        if len(rx) < 2:
            print(f"trk{n}: no quaternions at receiver (records {types})")
            continue
        t = rx[:, 0] / 1e6
        trk_live = trk[(trk[:, 0] >= rx[0, 0]) & (trk[:, 0] <= rx[-1, 0])]  # produced since the link came up
        dur = (trk_live[-1, 0] - trk_live[0, 0]) / 1e6
        gaps = np.diff(t)
        # latency: tracker sample (last 100 ms) closest to the received quaternion
        tq = trk[:, 1:5] / np.linalg.norm(trk[:, 1:5], axis=1)[:, None]
        lat, hit, stale = [], set(), []
        for row in rx:
            hi = np.searchsorted(trk[:, 0], row[0], "right")
            lo = np.searchsorted(trk[:, 0], row[0] - 100e3)
            if hi > lo:
                d = np.abs(tq[lo:hi] @ (row[1:5] / np.linalg.norm(row[1:5])))
                k = lo + np.flatnonzero(d >= d.max() - 1e-6)[-1]  # newest if at rest
                stale.append(np.degrees(2 * np.arccos(min(d[-1], 1.0))))  # vs tracker's latest
                hit.add(k)
                lat.append((row[0] - trk[k, 0]) / 1e3)
        lat = np.array(lat)
        print(f"trk{n} (id {tid}): {len(rx) / (t[-1] - t[0]):.0f} quat/s at rx vs {len(trk_live) / dur:.0f}/s produced, "
              f"delivered {100 * len(hit) / len(trk_live):.1f}% of samples, gaps >20ms {np.sum(gaps > 0.02)} max {1e3 * gaps.max():.0f}ms, "
              f"records {types}")
        got, lost = seq.get(tid, (0, 0))
        if got + lost:
            print(f"  receiver loss (seq gaps, as shown in server): {100 * lost / (got + lost):.2f}% ({lost}/{got + lost})")
        if len(lat):
            print(f"  latency ms: median {np.median(lat):.1f} p95 {np.percentile(lat, 95):.1f} "
                  f"max {lat.max():.1f}; rx vs tracker-now deg: p95 {np.percentile(stale, 95):.2f} max {max(stale):.2f}")
        if truth:
            tr = np.load(truth[0])
            idx = np.round((t - off / 1e6) * HZ).astype(int)
            keep = (idx < len(tr)) & (t >= args.settle)
            if keep.any():
                q = rx[keep, 1:5] / 32768
                err = np.abs(heading_error(q, tr[idx[keep]]))
                print(f"  heading error at rx after {args.settle:g}s: max {err.max():.2f} "
                      f"rms {np.sqrt(np.mean(err ** 2)):.2f} final {err[-1]:.2f} deg")
                q = q / np.linalg.norm(q, axis=1)[:, None]
                lag = e2e_latency_ms(t[keep], q, tr, off / 1e6)
                print("  end-to-end latency (IMU sample -> receiver, incl. fusion lag): "
                      + ("n/a (too little motion)" if lag is None else f"{lag:.2f} ms"))


if __name__ == "__main__":
    main()
