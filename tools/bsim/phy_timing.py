#!/usr/bin/env python3
"""Radio timing of a full-stack bsim run, from the phy dumps (-dump).

    python3 phy_timing.py <dump_dir> <rx.log> [--from 8]

Device 0 = receiver, others = trackers (non-ESB devices such as a WiFi interferer are ignored).
TDMA layout (slot/frame ticks, 32768 Hz) is taken from the receiver's last "TDMA reconfig" log line.
Note: bsim runs firmware code in zero time, so this shows radio timing, not CPU/ISR jitter.
"""
import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path

import numpy as np

TICK_US = 1e6 / 32768


def load(path):
    return list(csv.DictReader(open(path), skipinitialspace=True))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("rxlog")
    ap.add_argument("--from", dest="t0", type=float, default=8.0)
    a = ap.parse_args()

    cfg = re.findall(r"TDMA reconfig: .*?active=(\d+) slot=(\d+) frame=(\d+)", Path(a.rxlog).read_text(errors="ignore"))
    active, slot, frame = map(int, cfg[-1])
    frame_us = frame * TICK_US
    print(f"layout: {active} trackers, slot {slot} ticks ({slot * TICK_US:.0f} us), frame {frame} ticks ({frame_us:.0f} us)")

    t0 = a.t0 * 1e6
    rx_tx = sorted((int(r["start_tx_time"]), int(r["end_tx_time"]))
                   for r in load(Path(a.dump) / "d_2G4_00.Txv2.csv") if int(r["start_tx_time"]) >= t0)
    ack_starts = np.array([s for s, _ in rx_tx])
    trk = {}
    for f in sorted(Path(a.dump).glob("d_2G4_*.Txv2.csv")):
        d = int(f.name[6:8])
        rows = [r for r in load(f) if int(r["start_tx_time"]) >= t0 and r["phy_address"] != "0x00000000"]
        if d == 0 or not rows:
            continue
        trk[d] = np.array([(int(r["start_tx_time"]), int(r["end_tx_time"]), int(r["packet_size"])) for r in rows])

    # airtime by payload size
    air = defaultdict(list)
    for v in trk.values():
        for s, e, n in v:
            air[n].append(e - s)
    print("airtime us by payload bytes: " + ", ".join(f"{n}B {np.median(x):.0f} (n={len(x)})" for n, x in sorted(air.items())))

    # slot phase per tracker (circular), common origin = receiver frame time 0
    print("per tracker: TX start phase in frame (us), spread p5..p95, airtime share of slot")
    phases = {}
    for d, v in sorted(trk.items()):
        ph = np.mod(v[:, 0], frame_us)
        ref = ph[0]
        rel = np.mod(ph - ref + frame_us / 2, frame_us) - frame_us / 2  # unwrap around first sample
        centre = np.mod(ref + np.median(rel), frame_us)
        phases[d] = (centre, np.percentile(rel, 5), np.percentile(rel, 95), np.median(v[:, 1] - v[:, 0]))
        print(f"  dev{d}: phase {centre:7.0f}  spread {np.percentile(rel, 95) - np.percentile(rel, 5):5.0f} us  "
              f"air/slot {100 * phases[d][3] / (slot * TICK_US):.0f}%  tx/s {len(v) / ((v[-1, 0] - v[0, 0]) / 1e6):.0f}")

    # guard: from end of one tracker's latest TX to start of the next tracker's earliest TX
    order = sorted(phases.items(), key=lambda kv: kv[1][0])
    guards = []
    for (d1, p1), (d2, p2) in zip(order, order[1:] + order[:1]):
        end1 = p1[0] + p1[2] + p1[3]
        start2 = p2[0] + p2[1]
        guards.append(np.mod(start2 - end1, frame_us))
        print(f"  guard dev{d1} -> dev{d2}: {guards[-1]:.0f} us")

    # receiver ACK turnaround (ESB ACKs: receiver TX right after a tracker TX ends)
    ends = np.concatenate([v[:, 1] for v in trk.values()])
    k = np.searchsorted(ack_starts, ends)
    ok = k < len(ack_starts)
    gap = ack_starts[k[ok]] - ends[ok]
    gap = gap[gap < 500]
    if len(gap):
        print(f"ACK turnaround us: median {np.median(gap):.0f} (n={len(gap)} of {len(ends)} tracker TX)")
    busy = sum(np.sum(v[:, 1] - v[:, 0]) for v in trk.values()) + sum(e - s for s, e in rx_tx)
    span = max(v[-1, 1] for v in trk.values()) - t0
    print(f"channel busy {100 * busy / span:.1f}% of time; data airtime {sum(len(v) for v in trk.values()) / (span / 1e6):.0f} pkt/s")


if __name__ == "__main__":
    main()
