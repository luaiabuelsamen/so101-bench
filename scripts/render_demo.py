"""Render demonstration episodes to an MP4.

    python scripts/render_demo.py --episodes 2 --out demo.mp4
"""

from __future__ import annotations

import argparse

import numpy as np

from so101_bench import DemoEnv, ExpertConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--out", type=str, default="demo.mp4")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-randomise", action="store_true")
    parser.add_argument("--camera", type=str, default="front")
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--grip-mode", choices=("clamp", "force"), default="clamp")
    args = parser.parse_args()

    import imageio.v2 as imageio

    env = DemoEnv(
        seed=args.seed,
        randomise=not args.no_randomise,
        cameras=(args.camera, "gripper_fpv"),
        image_size=args.width,
        stride=2,
        expert=ExpertConfig(grip_mode=args.grip_mode),
    )
    frames: list[np.ndarray] = []
    with env:
        for i in range(args.episodes):
            result, recorder = env.rollout()
            wide = recorder.images[args.camera]
            wrist = recorder.images["gripper_fpv"]
            for a, b in zip(wide, wrist, strict=True):
                frames.append(np.hstack([a, b]))
            print(f"episode {i}: picked={result.picked} placed={result.placed}")
    imageio.mimsave(args.out, frames, fps=args.fps)
    print(f"wrote {args.out} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
