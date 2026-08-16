"""Failure-funnel decomposition of a trained policy, with confidence intervals.

    python scripts/funnel.py --arm base --episodes 100
    python scripts/funnel.py --arm delta --episodes 100

"Success is 10%" is a scalar; a fix needs to know WHERE the other 90% dies.
Each closed-loop episode is scored against an ordered ladder of stages, every
one measured from privileged simulator state (analysis only, never policy
input):

    reach    tool centre passes within 15 mm of the block in the horizontal
             plane while at grasping height
    touch    any pad-block contact force > 0.5 N
    seat     simultaneous fixed-pad AND moving-pad contact for >= 3 frames
             (the validated definition of a pinch from Phase 1b)
    lift     block rises > 30 mm
    place    block inside the container at the end

The report is the conditional drop-off between consecutive stages with Wilson
95% intervals: the stage with the largest conditional loss is the one the next
intervention must target. Re-run after any fix; the before/after funnel is the
evidence the fix worked for the stated reason.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent))

from so101_bench import DemoEnv  # noqa: E402
from so101_bench.scene import RAD_PER_TICK  # noqa: E402

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
STAGES = ["reach", "touch", "seat", "lift", "place"]


def pad_ids(scene):
    m = scene.model
    return (
        {m.geom(f"fixed_jaw_pad_{i}").id for i in range(1, 5)},
        {m.geom(f"moving_jaw_pad_{i}").id for i in range(1, 5)},
    )


def two_pad(scene, fixed, moving):
    d = scene.data
    tf = tm = False
    for i in range(d.ncon):
        c = d.contact[i]
        pair = {c.geom1, c.geom2}
        if scene.ids.block_geom in pair:
            tf |= bool(pair & fixed)
            tm |= bool(pair & moving)
    return tf and tm


def wilson(k, n):
    if n == 0:
        return (0.0, 0.0)
    z, p = 1.96, k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


@torch.no_grad()
def episode(scene, policy, st, arm, fixed, moving, horizon=260):
    scene.reset()
    policy.reset()
    block0 = scene.block_pos().copy()
    z0 = block0[2]
    flags = dict.fromkeys(STAGES, False)
    seat_run = 0
    a_prev = None
    for _ in range(horizon):
        s = torch.as_tensor(scene.state_ticks(), dtype=torch.float32)
        s_n = (s - torch.tensor(st["s_mean"])) / torch.tensor(st["s_std"])
        if arm != "base":
            d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
            d_n = (d - torch.tensor(st["d_mean"])) / torch.tensor(st["d_std"])
            s_in = torch.cat([s_n, d_n])
        else:
            s_in = s_n
        obs = {
            "observation.state": s_in.unsqueeze(0).to(DEVICE),
            "observation.images.front": torch.as_tensor(scene.image("front"))
            .permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
            "observation.images.gripper_fpv": torch.as_tensor(
                scene.image("gripper_fpv")
            ).permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
        }
        a_n = policy.select_action(obs).squeeze(0).cpu()
        action = a_n * torch.tensor(st["a_std"]) + torch.tensor(st["a_mean"])
        a_prev = action.clone()
        scene.hold(action.numpy() * RAD_PER_TICK, frames=2)

        tool = scene.tcp()
        blk = scene.block_pos()
        if (
            np.linalg.norm(tool[:2] - blk[:2]) < 0.015
            and tool[2] < blk[2] + 0.06
        ):
            flags["reach"] = True
        if scene.grip_force() > 0.5:
            flags["touch"] = True
        seat_run = seat_run + 1 if two_pad(scene, fixed, moving) else 0
        if seat_run >= 3:
            flags["seat"] = True
        if blk[2] - z0 > 0.03:
            flags["lift"] = True
    flags["place"] = flags["lift"] and scene.block_in_box()
    return flags


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arm", required=True)
    ap.add_argument("--ckpt", default="checkpoints/act")
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--seed", type=int, default=5000)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    from lerobot.policies.act.modeling_act import ACTPolicy

    path = Path(args.ckpt) / args.arm
    policy = ACTPolicy.from_pretrained(path).to(DEVICE).eval()
    st = np.load(path / "norm_stats.npz")
    env = DemoEnv(
        seed=args.seed, cameras=("front", "gripper_fpv"), image_size=96, stride=2,
        obs_quantum=16.0 if args.arm == "delta_q16" else 1.0,
    )
    fixed, moving = pad_ids(env.scene)

    counts = dict.fromkeys(STAGES, 0)
    for ep in range(args.episodes):
        flags = episode(env.scene, policy, st, args.arm, fixed, moving)
        for k in STAGES:
            counts[k] += flags[k]
        if (ep + 1) % 20 == 0:
            print(f"  {ep + 1}/{args.episodes}: " +
                  " ".join(f"{k} {counts[k]}" for k in STAGES), flush=True)
    env.close()

    n = args.episodes
    print(f"\n=== funnel: {args.arm}, n={n} ===")
    print(f"{'stage':>7} {'abs':>9} {'95% CI':>15} {'conditional on prev':>20}")
    prev = n
    out = {}
    for k in STAGES:
        lo, hi = wilson(counts[k], n)
        cond = counts[k] / prev if prev else 0.0
        clo, chi = wilson(counts[k], prev) if prev else (0.0, 0.0)
        print(f"{k:>7} {counts[k]:>4}/{n:<4} [{lo:.2f},{hi:.2f}] "
              f"{100 * cond:>8.0f}%  [{clo:.2f},{chi:.2f}]")
        out[k] = dict(count=int(counts[k]), n=int(n), ci=[lo, hi],
                      conditional=float(cond), cond_ci=[clo, chi])
        prev = max(counts[k], 1)
    worst = min(STAGES[1:], key=lambda k: out[k]["conditional"])
    print(f"\nlargest conditional loss: at '{worst}' "
          f"({100 * out[worst]['conditional']:.0f}% survive from the previous stage)")

    dest = args.json or f"results/funnel_{args.arm}.json"
    Path(dest).parent.mkdir(exist_ok=True)
    with open(dest, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"wrote {dest}")


if __name__ == "__main__":
    main()
