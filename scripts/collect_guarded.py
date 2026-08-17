"""Collect the demos_v3 recipe THROUGH the guard: shielded imitation data.

    python scripts/collect_guarded.py --episodes 200 --root data/demos_v3g

Fix 2 from the phase-3 review: if deployment runs under the guard, collect
under the guard, so the teacher's grasp FEELS like the capped grip in the
data and train/deploy distributions match by construction. Recipe mirrors
demos_v3 exactly (force-mode teacher, 96 px, stride 2, recovery kicks
p=0.02 sigma=1.5 on joints 1-3, per-episode grip target ~ U[12,45] counts
-- the guard truncates the deep end of that distribution, which is the
point) with one difference: scene.jaw_guard_factory arms guard v4 for every
episode, filtering commands at the hold() choke point so the RECORDED
actions are the applied (guarded) ones -- bus semantics.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import os
import sys

import numpy as np

from so101_bench import DemoEnv, ExpertConfig
from so101_bench.guard import JawGuard


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--root", type=str, required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--image-size", type=int, default=96)
    parser.add_argument("--max-attempts", type=int, default=None)
    parser.add_argument("--no-guard", action="store_true",
                        help="same recipe without the guard (control twin)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    rng = np.random.default_rng(args.seed)

    env = DemoEnv(
        seed=args.seed,
        cameras=("front", "gripper_fpv"),
        image_size=args.image_size,
        stride=2,
        expert=ExpertConfig(grip_mode="force"),
    )
    if not args.no_guard:
        env.scene.jaw_guard_factory = JawGuard.at_control_rate

    def kick(scene):
        if rng.random() < 0.02:
            j = int(rng.integers(1, 4))
            scene.data.qvel[j] += rng.normal(0, 1.5)

    def per_episode(e):
        e.expert.config = dataclasses.replace(
            e.expert.config, grip_target_counts=float(rng.uniform(12, 45))
        )

    with env:
        report = env.collect(
            args.episodes,
            root=args.root,
            only_success=True,
            max_attempts=args.max_attempts,
            hook=kick,
            per_episode=per_episode,
        )
    print(f"kept {report.kept} of {report.attempted} attempts")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)          # pyarrow finaliser SIGABRTs after flush; see collect.py


if __name__ == "__main__":
    main()
