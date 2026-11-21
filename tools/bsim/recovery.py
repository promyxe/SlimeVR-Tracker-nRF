#!/usr/bin/env python3
"""Heading recovery after magnetic disturbances (sim.py "transient" scenario).

    python3 recovery.py fw_quat.csv truth.npy events.json [--control ctrl_quat.csv] [--tol 2] [--hold 2]

events.json: [[center_s, peak_g], ...] from simulate(...)[1]["events"].
Per event: heading error before (median of -8..-3 s), peak during (+-2 s), and recovery time:
seconds after the pulse has faded (center + 1.5 s) until |error| <= tol for hold seconds.
--control: the same run without the field pulses (scenario transient_ctrl); then "error" is the
extra heading error caused by the disturbance and tol defaults to 1 deg.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "replay"))
from sim import HZ, heading_error  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fw_csv")
    ap.add_argument("truth_npy")
    ap.add_argument("events_json")
    ap.add_argument("--control")
    ap.add_argument("--tol", type=float)
    ap.add_argument("--hold", type=float, default=2.0)
    a = ap.parse_args()

    truth = np.load(a.truth_npy)

    def signed_err(path):
        fw = np.loadtxt(path, delimiter=",", skiprows=1)
        t = fw[:, 0] / 1e6
        idx = np.round(t * HZ).astype(int)
        keep = idx < len(truth)
        return t[keep], heading_error(fw[keep, 1:5], truth[idx[keep]])

    t, err = signed_err(a.fw_csv)
    if a.control:
        tc_, ec = signed_err(a.control)
        err = err - np.interp(t, tc_, ec)
        a.tol = a.tol if a.tol is not None else 1.0
    a.tol = a.tol if a.tol is not None else 2.0
    err = np.abs((err + 180) % 360 - 180)
    rec = []
    for tc, peak in json.load(open(a.events_json)):
        before = np.median(err[(t > tc - 8) & (t < tc - 3)])
        during = err[(t > tc - 2) & (t < tc + 2)].max()
        after = (t >= tc + 1.5) & (t < tc + 28)
        ta, ea = t[after], err[after]
        ok = ea <= a.tol
        r = None
        for k in np.flatnonzero(ok):
            hold = (ta >= ta[k]) & (ta < ta[k] + a.hold)
            if ta[k] + a.hold <= ta[-1] and ok[hold].all():
                r = ta[k] - (tc + 1.5)
                break
        rec.append(r)
        rs = f"{r:5.1f} s" if r is not None else " >26 s"
        print(f"event {tc:5.0f}s peak {peak:5.2f} G: before {before:4.1f} peak {during:5.1f} deg, recovered in {rs}")
    done = [r for r in rec if r is not None]
    print(f"recovered {len(done)}/{len(rec)}, median {np.median(done) if done else float('nan'):.1f} s, "
          f"max {max(done) if done else float('nan'):.1f} s")


if __name__ == "__main__":
    main()
