"""Collect demonstrations as a LeRobotDataset.

    python scripts/collect.py --episodes 200 --root ~/data/pick_place
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

from so101_bench import DemoEnv, ExpertConfig


def _exit_success() -> None:
    """Terminate without running interpreter finalisers.

    pyarrow's teardown aborts the process (SIGABRT, exit 134) in this
    environment: it attempts a lazy import while ``sys.meta_path`` is already
    torn down. It happens strictly after the dataset is flushed -- the written
    data verifies intact -- but a CLI that dies on a signal is unusable in a
    pipeline, and neither an explicit ``gc.collect()`` nor closing the dataset
    first avoids it. Flush what we own and leave.
    """
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--root", type=str, required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cameras", nargs="*", default=["front", "gripper_fpv"])
    parser.add_argument("--image-size", type=int, default=128)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--grip-mode", choices=("clamp", "force"), default="clamp")
    parser.add_argument("--obs-quantum", type=float, default=1.0,
                        help="observation quantum in counts; 0.25 is a 4x finer encoder")
    parser.add_argument("--keep-failures", action="store_true")
    parser.add_argument("--max-attempts", type=int, default=None)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    env = DemoEnv(
        seed=args.seed,
        cameras=args.cameras,
        image_size=args.image_size,
        stride=args.stride,
        obs_quantum=args.obs_quantum,
        expert=ExpertConfig(grip_mode=args.grip_mode),
    )
    with env:
        report = env.collect(
            args.episodes,
            root=args.root,
            only_success=not args.keep_failures,
            max_attempts=args.max_attempts,
        )
    print(f"{report.kept} episodes -> {report.root} "
          f"({100 * report.yield_fraction:.0f}% yield)")
    _exit_success()


if __name__ == "__main__":
    main()
