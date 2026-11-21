#!/usr/bin/env python3
"""Heading error of the simulated firmware's output against ground truth.

    python3 compare.py fw_quat.csv truth.npy [--settle 30]

fw_quat.csv: written by the nrf52_bsim tracker (SLIME_SIM_OUT, t_us,qw,qx,qy,qz).
truth.npy:   (n, 4) true body->world quaternions at the recording rate (400 Hz),
             saved from tools/replay/sim.py's simulate().
By default the constant heading offset between the firmware's earth frame and
the simulation's is removed using the 10 s after --settle. With --absolute it is
not: both use magnetic north on +y, so once the magnetometer is calibrated the
raw error is the real heading error (offset removal would bake in any gyro drift
accumulated before calibration).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "replay"))
from sim import HZ, heading_error  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fw_csv")
    ap.add_argument("truth_npy")
    ap.add_argument("--settle", type=float, default=30.0)
    ap.add_argument("--absolute", action="store_true", help="do not remove a heading offset")
    args = ap.parse_args()

    fw = np.loadtxt(args.fw_csv, delimiter=",", skiprows=1)
    truth = np.load(args.truth_npy)
    t = fw[:, 0] / 1e6
    idx = np.round(t * HZ).astype(int)
    keep = idx < len(truth)
    t, idx, q = t[keep], idx[keep], fw[keep, 1:5]
    err = heading_error(q, truth[idx])

    after = t >= args.settle
    ref = err[after & (t < args.settle + 10)]
    offset = 0.0 if args.absolute else np.degrees(
        np.arctan2(np.sin(np.radians(ref)).mean(), np.cos(np.radians(ref)).mean()))
    e = np.abs((err[after] - offset + 180) % 360 - 180)
    print(f"samples={len(t)} span={t[0]:.1f}-{t[-1]:.1f}s output_rate={len(t) / (t[-1] - t[0]):.0f}Hz")
    print(f"heading error after {args.settle:g}s: max={e.max():.2f} rms={np.sqrt((e ** 2).mean()):.2f} "
          f"final={e[-1]:.2f} deg")
    for s in range(int(args.settle), int(t[-1]) + 1, 30):
        sel = (t[after] >= s) & (t[after] < s + 30)
        if sel.any():
            print(f"  {s:4d}-{s + 30:<4d}s  max {e[sel].max():6.2f}  mean {e[sel].mean():6.2f}")


if __name__ == "__main__":
    main()
