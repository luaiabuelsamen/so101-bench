"""Measure scripted-expert success rate under domain randomisation.

    python scripts/evaluate.py --episodes 40
    python scripts/evaluate.py --episodes 40 --grip-mode force
    python scripts/evaluate.py --episodes 40 --no-randomise
"""

from __future__ import annotations

import argparse
import json
import logging
import time

import numpy as np

from so101_bench import DemoEnv, ExpertConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=40)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-randomise", action="store_true")
    parser.add_argument("--grip-mode", choices=("clamp", "force"), default="clamp")
    parser.add_argument("--grasp-dz", type=float, default=0.0)
    parser.add_argument("--json", type=str, default=None, help="write results here")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    env = DemoEnv(
        seed=args.seed,
        randomise=not args.no_randomise,
        cameras=(),
        expert=ExpertConfig(grip_mode=args.grip_mode, grasp_dz=args.grasp_dz),
    )

    start = time.time()
    results = env.evaluate(args.episodes)
    elapsed = time.time() - start

    picked = sum(r.picked for r in results)
    placed = sum(r.placed for r in results)
    force = np.array([r.grip_force_n for r in results])
    n = len(results)

    print(f"{'ep':>4} {'pick':>6} {'place':>6} {'rise/mm':>8} {'grip/N':>8}")
    for i, r in enumerate(results):
        print(
            f"{i:4d} {str(r.picked):>6} {str(r.placed):>6} "
            f"{1000 * r.peak_rise_m:8.1f} {r.grip_force_n:8.2f}"
        )
    print(f"\npicked {picked}/{n} ({100 * picked / n:.0f}%)")
    print(f"placed {placed}/{n} ({100 * placed / n:.0f}%)")
    print(f"grip force {force.min():.2f}..{force.max():.2f} N (mean {force.mean():.2f})")
    print(f"randomisation {'off' if args.no_randomise else 'on'}, "
          f"grip mode {args.grip_mode}, {elapsed / n:.2f} s/episode")

    if args.json:
        with open(args.json, "w") as fh:
            json.dump(
                {
                    "episodes": n,
                    "picked": picked,
                    "placed": placed,
                    "grip_mode": args.grip_mode,
                    "randomised": not args.no_randomise,
                    "seconds_per_episode": elapsed / n,
                },
                fh,
                indent=2,
            )
        print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
