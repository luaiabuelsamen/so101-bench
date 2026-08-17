"""Rule 1 pairing: the scripted expert under the identical guarded protocol.

    python scripts/eval_expert_guarded.py

The learned policy's guarded 0/30 is a defect until proven a boundary; the
proof requires a working reference under the same conditions. Same seed
(3000), same DR stream, same crush tiers, same guard (v3 constants
re-expressed at 30 Hz control cadence), applied through scene.hold() so the
expert's jaw commands are capped exactly as the policy's were.

Readout: expert guarded ~ expert raw  -> guard is compatible with a
competent controller; the policy's 0/30 is the policy's problem (train/
deploy shift is real). Expert guarded ~ 0/30 too -> the guard or its
integration is broken and the policy result was a bug, filed as one.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from so101_bench import DemoEnv, ExpertConfig  # noqa: E402
from so101_bench.guard import JawGuard  # noqa: E402

CRUSH_TIERS = [-1.0, 120.0, 60.0]
EPISODES = 30


def wilson(k, n):
    z, p = 1.96, k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--grip-mode", default="force",
                    choices=["clamp", "force", "oracle"],
                    help="clamp is open-loop in time and cannot tolerate the "
                         "guard's rate limit (its grasp window expires before "
                         "the slowed jaw arrives); force is the closed-loop "
                         "teacher and the fair reference")
    ap.add_argument("--json", default="results/expert_guarded_pairing.json")
    args = ap.parse_args()

    rows = []
    print(f"grip_mode={args.grip_mode}")
    print(f"{'crush':>6} {'mode':>8} {'placed':>8} {'picked':>7} "
          f"{'crushed':>8} {'peakF med':>10}")
    for crush in CRUSH_TIERS:
        for guarded in (False, True):
            env = DemoEnv(
                seed=3000, cameras=(), image_size=96,
                crush_newtons=None if crush <= 0 else crush,
                expert=ExpertConfig(grip_mode=args.grip_mode),
            )
            env.scene.jaw_guard_factory = (
                JawGuard.at_control_rate if guarded else None
            )
            forces, placed, picked, crushed = [], 0, 0, 0
            for _ in range(EPISODES):
                r, _ = env.rollout(record=False)
                placed += r.placed
                picked += r.picked
                crushed += env.scene.crushed
                forces.append(r.grip_force_n)
            env.close()
            lo, hi = wilson(placed, EPISODES)
            pf = float(np.median(forces))
            rows.append(dict(crush=crush, guarded=guarded, placed=int(placed),
                             picked=int(picked), crushed=int(crushed),
                             episodes=EPISODES, ci=[float(lo), float(hi)],
                             peak_force_median=pf))
            print(f"{crush:6.0f} {'guard' if guarded else 'raw':>8} "
                  f"{placed:5d}/{EPISODES} {picked:7d} {crushed:8d} {pf:10.1f}",
                  flush=True)

    Path("results").mkdir(exist_ok=True)
    with open(args.json, "w") as fh:
        json.dump(dict(grip_mode=args.grip_mode, rows=rows), fh,
                  indent=2, default=float)
    print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
