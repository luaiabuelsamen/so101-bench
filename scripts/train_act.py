"""Train lerobot ACT on the expert demos; evaluate CLOSED-LOOP in the env.

    python scripts/train_act.py --arm base      --steps 8000
    python scripts/train_act.py --arm delta     --steps 8000
    python scripts/train_act.py --arm delta_q16 --steps 8000

THE ARMS. Standard ACT observes the current state and images only -- it never
sees the previous action, so `delta = a[t-1] - s[t]` is NOT a derived feature
for this architecture (unlike for the sequence probe, where state+action
history subsumed it). Appending delta to the state injects exactly the
bus-observable channel this project characterised:

    base       observation.state = s[t]            (6-dim)   -- stock ACT
    delta      observation.state = [s[t], d[t]]    (12-dim)  -- native quantum
    delta_q16  same, with the observation quantised to 16 counts online at
               EVAL (training uses offline-quantised replay of the same data;
               closed-loop coarse behaviour cannot be replayed, only run)

Pre-registered prediction (from the scripted and probe results): no
significant success difference at wide crush windows; if delta helps anywhere
it is contact-transition timing and entry force at the tight threshold tier,
and a null here is reportable -- the probe already showed the event needs no
fine resolution and little beyond state context.

Training details follow the stack every SO-101 user runs: lerobot 0.3.4 ACT,
resnet18 backbone, chunk 30 at the recorded 15 Hz (demos stride-2 from the
30 Hz control loop), VAE on, AMP. delta_timestamps is set for "action" ONLY --
applying it to image columns in 0.3.4 materialises whole columns per item and
is ~20x slower (known bug, documented in the research repo).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent))

from so101_bench import DemoEnv, ExpertConfig  # noqa: E402

CHUNK = 30
FPS = 15                    # demos are stride-2 from the 30 Hz loop
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def load_dataset(root: str):
    import lerobot.datasets as _lds

    _lds.check_video_encoder_parameters_pyav = lambda *a, **k: None
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    delta_ts = {"action": [i / FPS for i in range(CHUNK)]}
    return LeRobotDataset(
        repo_id="so101_bench/pick_place", root=root, delta_timestamps=delta_ts
    )


class ArmDataset(torch.utils.data.Dataset):
    """Wraps the LeRobotDataset, building each arm's observation.state.

    delta[t] = a[t-1] - s[t] within an episode (0 at episode starts). The
    previous action is looked up by global index; episode boundaries come from
    ``episode_index`` equality, which is safe because frames are stored in
    order within an episode.
    """

    def __init__(self, ds, arm: str, quantum: float = 1.0):
        self.ds = ds
        self.arm = arm
        self.q = quantum

    def __len__(self):
        return len(self.ds)

    def _quant(self, x: torch.Tensor) -> torch.Tensor:
        return torch.round(x / self.q) * self.q if self.q > 1 else x

    def __getitem__(self, i):
        item = self.ds[i]
        s = self._quant(item["observation.state"])
        if self.arm == "base":
            item["observation.state"] = s
            return item
        if i > 0:
            prev = self.ds.hf_dataset[i - 1]
            same_ep = int(prev["episode_index"]) == int(item["episode_index"])
            a_prev = torch.as_tensor(prev["action"], dtype=s.dtype) if same_ep else None
        else:
            a_prev = None
        delta = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
        item["observation.state"] = torch.cat([s, delta])
        return item


def build_policy(state_dim: int, stats: dict):
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    cfg = ACTConfig(
        input_features={
            "observation.state": PolicyFeature(FeatureType.STATE, (state_dim,)),
            "observation.images.front": PolicyFeature(FeatureType.VISUAL, (3, 96, 96)),
            "observation.images.gripper_fpv": PolicyFeature(
                FeatureType.VISUAL, (3, 96, 96)
            ),
        },
        output_features={"action": PolicyFeature(FeatureType.ACTION, (6,))},
        chunk_size=CHUNK,
        n_action_steps=CHUNK,
        dim_model=256,
        n_heads=4,
        dim_feedforward=1024,
        n_encoder_layers=2,
        n_decoder_layers=1,
        vision_backbone="resnet18",
        device=DEVICE,
    )
    return ACTPolicy(cfg, dataset_stats=stats)


def arm_stats(base_stats: dict, arm: str) -> dict:
    """Extend dataset stats to the 12-dim state for the delta arms."""
    stats = {k: dict(v) for k, v in base_stats.items()}
    if arm == "base":
        return stats
    s = stats["observation.state"]
    out = {}
    for key in ("mean", "std", "min", "max"):
        v = torch.as_tensor(np.asarray(s[key]), dtype=torch.float32)
        if key == "mean":
            pad = torch.zeros_like(v)
        elif key == "std":
            pad = torch.full_like(v, 8.0)     # counts; delta is small
        elif key == "min":
            pad = torch.full_like(v, -200.0)
        else:
            pad = torch.full_like(v, 200.0)
        out[key] = torch.cat([v, pad])
    stats["observation.state"] = out
    return stats


def to_batch_images(item):
    """Ensure image tensors are float CHW in [0,1]."""
    out = {}
    for k, v in item.items():
        if k.startswith("observation.images"):
            v = torch.as_tensor(v)
            if v.dtype == torch.uint8:
                v = v.float() / 255.0
            if v.ndim == 3 and v.shape[-1] == 3:
                v = v.permute(2, 0, 1)
        out[k] = v
    return out


def train(args):
    ds = load_dataset(args.root)
    quantum = 16.0 if args.arm == "delta_q16" else 1.0
    wrapped = ArmDataset(ds, "delta" if args.arm != "base" else "base", quantum)
    state_dim = 6 if args.arm == "base" else 12
    policy = build_policy(state_dim, arm_stats(ds.meta.stats, args.arm)).to(DEVICE)
    policy.train()

    loader = torch.utils.data.DataLoader(
        wrapped,
        batch_size=args.batch,
        shuffle=True,
        num_workers=2,
        collate_fn=lambda items: torch.utils.data.default_collate(
            [to_batch_images(i) for i in items]
        ),
        drop_last=True,
    )
    opt = torch.optim.AdamW(policy.parameters(), lr=1e-4, weight_decay=1e-4)
    scaler = torch.amp.GradScaler(enabled=DEVICE == "cuda")

    step, t0 = 0, time.time()
    while step < args.steps:
        for batch in loader:
            batch = {
                k: (v.to(DEVICE) if isinstance(v, torch.Tensor) else v)
                for k, v in batch.items()
            }
            with torch.amp.autocast(DEVICE, enabled=DEVICE == "cuda"):
                loss, _ = policy.forward(batch)
            opt.zero_grad()
            scaler.scale(loss).backward()
            scaler.unscale_(opt)
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0)
            scaler.step(opt)
            scaler.update()
            step += 1
            if step % 500 == 0:
                print(
                    f"  step {step}/{args.steps} loss {loss.item():.4f} "
                    f"({step / (time.time() - t0):.1f} it/s)",
                    flush=True,
                )
            if step >= args.steps:
                break
    out = Path(args.out) / args.arm
    out.mkdir(parents=True, exist_ok=True)
    policy.save_pretrained(out)
    print(f"saved {out}")
    return policy


@torch.no_grad()
def evaluate(policy, args):
    """Closed-loop rollout in the env at the demo control rate (15 Hz)."""
    policy.eval()
    quantum = 16.0 if args.arm == "delta_q16" else 1.0
    results = []
    for crush in args.eval_crush:
        env = DemoEnv(
            seed=1000,
            cameras=("front", "gripper_fpv"),
            image_size=96,
            stride=2,
            obs_quantum=quantum,          # ONLINE coarsening at eval
            crush_newtons=None if crush <= 0 else crush,
            expert=ExpertConfig(),        # unused; env provides scene/reset
        )
        n_ok = n_crush = n_drop = 0
        for _ep in range(args.eval_episodes):
            scene = env.scene
            scene.reset()
            policy.reset()
            z0 = scene.block_pos()[2]
            a_prev = None
            peak = 0.0
            for _t in range(260):        # 260 x (2/30 s) ~ 17 s budget
                s = torch.as_tensor(scene.state_ticks(), dtype=torch.float32)
                if args.arm != "base":
                    d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
                    s_in = torch.cat([s, d])
                else:
                    s_in = s
                obs = {
                    "observation.state": s_in.unsqueeze(0).to(DEVICE),
                    "observation.images.front": torch.as_tensor(
                        scene.image("front")
                    ).permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
                    "observation.images.gripper_fpv": torch.as_tensor(
                        scene.image("gripper_fpv")
                    ).permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
                }
                action = policy.select_action(obs).squeeze(0).cpu()
                a_prev = action.clone()
                from so101_bench.scene import RAD_PER_TICK

                scene.hold(action.numpy() * RAD_PER_TICK, frames=2)
                peak = max(peak, scene.block_pos()[2] - z0)
                if scene.crushed:
                    break
            picked = peak > 0.03
            placed = picked and scene.block_in_box() and not scene.crushed
            n_ok += placed
            n_crush += scene.crushed
            n_drop += picked and not placed and not scene.crushed
            env.scene.reset()
        results.append(
            dict(crush=crush, success=n_ok, crushed=n_crush, dropped=n_drop,
                 episodes=args.eval_episodes)
        )
        print(
            f"  crush {crush}: success {n_ok}/{args.eval_episodes} "
            f"crush {n_crush} drop {n_drop}",
            flush=True,
        )
        env.close()
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", choices=("base", "delta", "delta_q16"),
                        required=True)
    parser.add_argument("--root", default="data/demos_native")
    parser.add_argument("--out", default="checkpoints/act")
    parser.add_argument("--steps", type=int, default=8000)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--eval-episodes", type=int, default=30)
    parser.add_argument("--eval-crush", type=float, nargs="*",
                        default=[-1.0, 120.0, 60.0])
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    policy = train(args)
    results = evaluate(policy, args)
    out = args.json or f"results/act_{args.arm}.json"
    Path(out).parent.mkdir(exist_ok=True)
    with open(out, "w") as fh:
        json.dump(dict(arm=args.arm, steps=args.steps, results=results), fh,
                  indent=2)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
