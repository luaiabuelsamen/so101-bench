"""Run a real-data ACT checkpoint on the physical SO-101 follower.

This is the only script in the repo that lets a learned policy command the arm.
Everything else either records or trains. Read the safety notes before running.

SAFETY
------
The follower is configured with `max_relative_target`, which clamps how far any
joint may be commanded from its present position in one control step. The
default here is 8 degrees, derived from the demonstrations rather than guessed:
per-step motion in data/real/pickplace_real_v0 has a 99th percentile of 5.1
degrees and a maximum of 10.0, so 8 admits essentially all demonstrated motion
while bounding a divergent policy to roughly demonstration speed.

That clamp bounds velocity, not target. A policy can still walk the arm
somewhere bad over many steps, so:

  - keep a hand on the power, and stop on anything unexpected
  - clear the workspace of anything you mind being knocked over
  - start from the same rest pose the demonstrations start from
  - --max-steps ends the episode on a timer regardless of what the policy does

Torque is released on exit, including on Ctrl-C, so the arm goes limp rather
than holding whatever pose it failed in. It will drop what it is holding.

WHAT TO EXPECT
--------------
These checkpoints are trained on 50 demonstrations. The simulated grid needed
280 to reach 6-27% success on an easier task with two cameras. Coherent
reaching would be a good outcome; completed pick-and-place would be surprising.
A policy that does nothing, or drifts to one pose and stays, is the expected
failure and is informative about the chain rather than about the method.

    python scripts/run_policy_real.py --checkpoint checkpoints/real50_base_s0 \
        --arm base --max-steps 300
"""

import argparse
import time
from pathlib import Path

import numpy as np
import torch

FPS = 30
GRIPPER = 5


def build_observation(arm, pos, a_prev, a_prev2, stats, k_hat, hist):
    """Mirror scripts/train_act_real.py's observation exactly."""
    s_n = (pos - stats["s_mean"]) / stats["s_std"]
    if arm == "base":
        return s_n
    if arm == "hist":
        ap = pos if a_prev is None else a_prev
        return np.concatenate([s_n, (ap - stats["a_mean"]) / stats["a_std"]])
    if arm == "ghist":
        past = hist[max(0, len(hist) - 9)]
        return np.concatenate([s_n, (past - stats["s_mean"]) / stats["s_std"]])
    delta = np.zeros(6, np.float32) if a_prev is None else a_prev - pos
    if arm == "excess":
        rate = np.zeros(6, np.float32) if a_prev2 is None else a_prev - a_prev2
        e = delta - k_hat * rate
        return np.concatenate([s_n, e / 8.0])
    return np.concatenate([s_n, delta / 8.0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--arm", required=True,
                    choices=("base", "hist", "delta", "excess", "ghist"))
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--robot-id", default="my_awesome_follower_arm")
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--max-steps", type=int, default=300, help="hard episode cap")
    ap.add_argument("--max-relative-target", type=float, default=8.0,
                    help="degrees per step per joint; 99.9th pct of the demos")
    ap.add_argument("--image-size", type=int, default=96)
    ap.add_argument("--dry-run", action="store_true",
                    help="run the full loop but never send an action to the arm")
    args = ap.parse_args()

    import cv2
    from lerobot.cameras.opencv import OpenCVCameraConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.robots import make_robot_from_config
    from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    stats = dict(np.load(Path(args.checkpoint) / "norm_stats.npz"))
    k_hat = stats.get("k_hat", np.zeros(6, np.float32))
    policy = ACTPolicy.from_pretrained(args.checkpoint).to(dev).eval()
    print(f"loaded {args.checkpoint} (arm={args.arm}) on {dev}")

    cfg = SO101FollowerConfig(
        port=args.port,
        id=args.robot_id,
        max_relative_target=args.max_relative_target,
        cameras={"front": OpenCVCameraConfig(
            index_or_path=args.camera, width=640, height=480, fps=FPS)},
    )
    robot = make_robot_from_config(cfg)
    robot.connect()
    print(f"connected. clamp {args.max_relative_target} deg/step, "
          f"cap {args.max_steps} steps ({args.max_steps/FPS:.0f}s)"
          + ("  [DRY RUN: no actions sent]" if args.dry_run else ""))

    a_prev = a_prev2 = None
    hist = []
    period = 1.0 / FPS
    try:
        for step in range(args.max_steps):
            t0 = time.perf_counter()
            obs = robot.get_observation()
            pos = np.array([obs[f"{m}.pos"] for m in robot.bus.motors], np.float32)[:6]
            hist.append(pos)

            frame = obs["front"]
            img = cv2.resize(np.asarray(frame), (args.image_size, args.image_size),
                             interpolation=cv2.INTER_AREA)
            img_t = torch.from_numpy(img.copy()).permute(2, 0, 1).float().div(255)

            state = build_observation(args.arm, pos, a_prev, a_prev2, stats, k_hat, hist)
            with torch.inference_mode():
                a_norm = policy.select_action({
                    "observation.state": torch.from_numpy(
                        state.astype(np.float32)).unsqueeze(0).to(dev),
                    "observation.images.front": img_t.unsqueeze(0).to(dev),
                }).squeeze(0).cpu().numpy()
            action = a_norm * stats["a_std"] + stats["a_mean"]

            if not args.dry_run:
                robot.send_action({m: float(action[i])
                                   for i, m in enumerate(list(robot.bus.motors)[:6])})
            a_prev2, a_prev = a_prev, action.astype(np.float32)

            if step % 30 == 0:
                print(f"  step {step:4d}  jaw cmd {action[GRIPPER]:6.1f}  "
                      f"jaw pos {pos[GRIPPER]:6.1f}  "
                      f"delta {action[GRIPPER]-pos[GRIPPER]:+6.1f}", flush=True)
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        robot.disconnect()      # disable_torque_on_disconnect releases the arm
        print("disconnected, torque released -- the arm is limp and will drop.")


if __name__ == "__main__":
    main()
