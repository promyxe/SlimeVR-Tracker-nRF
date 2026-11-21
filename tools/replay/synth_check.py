#!/usr/bin/env python3
"""Synthetic ground-truth check for vqf_replay, plus a mag-disturbance sweep.

Generates raw-collection CSVs for a tracker turning about the vertical axis
with an unobservable vertical gyro bias, runs vqf_replay on them and compares
the 9D yaw against the known truth.

    python3 synth_check.py path/to/vqf_replay           # assertions only
    python3 synth_check.py path/to/vqf_replay --sweep   # + magRejectionFactor table

Motion always carries a wobble: a perfectly constant spin rate is
indistinguishable from "at rest with a large bias" for VQF's rest detector.
"""
import csv
import math
import subprocess
import sys
import tempfile
from pathlib import Path

HZ = 400.0
MAG_HZ = 50.0
BIAS_Z_DPS = 0.2  # residual vertical bias: VQF cannot see it without mag
FIELD = (0.0, 0.2, -0.4)  # gauss, ENU (VQF puts magnetic north on +y)
WOBBLE_DPS, WOBBLE_HZ = 15.0, 0.3
FACTORS = (2, 10, 100, 1000)


class Motion:
    """Yaw rate = piecewise-constant base + sinusoidal wobble, with analytic yaw."""

    def __init__(self, *segments):
        self.segments = segments  # (start_s, base_dps), sorted

    def rate(self, t):
        base = [b for s, b in self.segments if s <= t][-1]
        return base + WOBBLE_DPS * math.sin(2 * math.pi * WOBBLE_HZ * t)

    def yaw(self, t):
        yaw = 0.0
        for i, (start, base) in enumerate(self.segments):
            end = self.segments[i + 1][0] if i + 1 < len(self.segments) else math.inf
            if t > start:
                yaw += base * (min(t, end) - start)
        w = 2 * math.pi * WOBBLE_HZ
        return yaw + WOBBLE_DPS / w * (1 - math.cos(w * t))


def rot_z(v, ang):
    c, s = math.cos(ang), math.sin(ang)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def write_recording(path, duration_s, motion, field_at):
    """field_at(t) -> world field; body field = Rz(-yaw) * world."""
    mag_every = int(HZ / MAG_HZ)
    with open(path, "w") as f:
        f.write("seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp,discontinuity\n")
        for i in range(int(duration_s * HZ)):
            t = i / HZ
            m = rot_z(field_at(t), -math.radians(motion.yaw(t))) if i % mag_every == 0 else (0.0, 0.0, 0.0)
            f.write(f"{i & 0xFFFF},0,0,{motion.rate(t) + BIAS_Z_DPS:.6f},0,0,1,"
                    f"{m[0]:.6f},{m[1]:.6f},{m[2]:.6f},25,0\n")
    Path(path).with_suffix(".meta.txt").write_text(f"gyro_odr_hz={HZ}\nmag_odr_hz={MAG_HZ}\n")


def run(tool, csv_path, *args):
    trace = str(csv_path) + ".trace.csv"
    out = subprocess.run([tool, *args, "--out", trace, str(csv_path)],
                         check=True, capture_output=True, text=True).stdout
    summary = dict(kv.split("=", 1) for line in out.splitlines() for kv in line.split())
    return summary, trace


def yaw_errors(trace, motion, settle_s, column="yaw9_deg"):
    """yaw - truth after settle, with the constant initial heading offset removed."""
    rows = [(float(r["t"]), float(r[column])) for r in csv.DictReader(open(trace))]
    err = [math.remainder(y - motion.yaw(t), 360.0) for t, y in rows if t >= settle_s]
    head = err[:50]
    offset = math.degrees(math.atan2(sum(math.sin(math.radians(e)) for e in head),
                                     sum(math.cos(math.radians(e)) for e in head)))
    return [math.remainder(e - offset, 360.0) for e in err]


def main():
    tool = sys.argv[1]
    tmp = Path(tempfile.mkdtemp(prefix="vqf_replay_"))

    # Mean 20 dps for 180 s.
    motion = Motion((0, 20.0))
    clean = tmp / "clean.csv"
    write_recording(clean, 180, motion, lambda t: FIELD)
    s, trace = run(tool, clean)
    assert max(map(abs, yaw_errors(trace, motion, 20))) < 2, "9D yaw should track truth with a clean field"
    s, trace = run(tool, clean, "--no-mag")
    drift = yaw_errors(trace, motion, 20)[-1]
    expected = BIAS_Z_DPS * (180 - 20)
    assert abs(drift - expected) < 3, f"without mag, yaw must drift by bias*t ({drift:.1f} vs {expected:.1f})"
    print(f"ok: clean field holds yaw; without mag it drifts {drift:.0f} deg as predicted")

    if "--sweep" not in sys.argv:
        return

    # 20 dps first so VQF adopts a mag reference (needs > magNewMinGyr; firmware
    # seeds it from online calibration instead), then mean 5 dps < magNewMinGyr
    # so the disturbed field is never adopted as a new reference.
    motion = Motion((0, 20.0), (20, 5.0))
    scenarios = {
        "norm x1.3 (direction still right)": lambda t: tuple(c * 1.3 for c in FIELD) if 60 <= t < 300 else FIELD,
        "horizontal turned 30 deg (direction wrong)": lambda t: rot_z(FIELD, math.radians(30)) if 60 <= t < 300 else FIELD,
        "direction wanders +-10 deg (walking around)": lambda t: rot_z(FIELD, math.radians(10 * math.sin(2 * math.pi * t / 40))) if t >= 60 else FIELD,
    }
    print(f"\nyaw error in deg (max during / at end), 360 s run, disturbance 60-300 s, "
          f"{BIAS_Z_DPS} dps vertical bias, magRejectionFactor:")
    print(f"{'scenario':45s}" + "".join(f"{k:>14d}" for k in FACTORS))
    for idx, (name, field) in enumerate(scenarios.items()):
        path = tmp / f"sweep{idx}.csv"
        write_recording(path, 360, motion, field)
        cells = []
        for k in FACTORS:
            err = yaw_errors(run(tool, path, "--param", f"magRejectionFactor={k}")[1], motion, 25)
            cells.append(f"{max(map(abs, err)):6.1f} / {abs(err[-1]):4.1f}")
        print(f"{name:45s}" + "".join(f"{c:>14s}" for c in cells))


if __name__ == "__main__":
    main()
