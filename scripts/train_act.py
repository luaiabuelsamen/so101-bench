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

    # no delta_timestamps: FastChunkDataset builds chunks itself, and the
    # windowing path is the 1.5 s/item bottleneck being bypassed
    return LeRobotDataset(repo_id="so101_bench/pick_place", root=root)


class FastChunkDataset(torch.utils.data.Dataset):
    """In-memory replacement for LeRobotDataset's training path.

    LeRobotDataset.__getitem__ costs ~1.5 s PER ITEM here (measured; the
    delta_timestamps windowing path), which starves the GPU completely -- two
    workers at 100% CPU, GPU at 0%, days per run. Raw ``hf_dataset[i]`` access
    is instant, so this class pulls the state/action/episode columns into
    numpy once, builds action chunks and pad masks itself, and only decodes
    the two 96px images per item (~ms). Same tensors out, ~100x faster.
    """

    def __init__(self, ds, arm: str, quantum: float = 1.0):
        self.arm = arm
        self.q = quantum
        # Normalisation lives HERE, not in the policy: in lerobot 0.3.4 the
        # Normalize step moved out of the policies into external processors,
        # and ACTPolicy silently ignores a dataset_stats kwarg. Training on
        # raw encoder counts (+-1000) put an l1 of ~560 through fp16 AMP,
        # which overflowed to NaN by step 100. States and action targets are
        # standardised with these stats; evaluation must apply the same
        # transform in and invert it out.
        st = ds.meta.stats
        self.s_mean = np.asarray(st["observation.state"]["mean"], np.float32)
        self.s_std = np.asarray(st["observation.state"]["std"], np.float32) + 1e-3
        self.a_mean = np.asarray(st["action"]["mean"], np.float32)
        self.a_std = np.asarray(st["action"]["std"], np.float32) + 1e-3
        self.d_mean = np.zeros(6, np.float32)
        self.d_std = np.full(6, 8.0, np.float32)
        hf = ds.hf_dataset.with_format(None)
        self.S = np.asarray(hf["observation.state"], dtype=np.float32)
        self.A = np.asarray(hf["action"], dtype=np.float32)
        ep = np.asarray(hf["episode_index"], dtype=np.int64)
        self.hf = ds.hf_dataset
        # episode end index (exclusive) per frame, for chunk padding
        ends = np.empty(len(ep), dtype=np.int64)
        start = 0
        for i in range(1, len(ep) + 1):
            if i == len(ep) or ep[i] != ep[i - 1]:
                ends[start:i] = i
                start = i
        self.ep_end = ends
        # previous action within the episode (zeros at episode starts)
        self.A_prev = np.zeros_like(self.A)
        same = np.zeros(len(ep), dtype=bool)
        same[1:] = ep[1:] == ep[:-1]
        self.A_prev[same] = self.A[np.flatnonzero(same) - 1]
        self.first = ~same

    def __len__(self):
        return len(self.S)

    def _quant(self, x: np.ndarray) -> np.ndarray:
        return np.round(x / self.q) * self.q if self.q > 1 else x

    def __getitem__(self, i):
        end = self.ep_end[i]
        n = min(CHUNK, end - i)
        chunk = np.empty((CHUNK, 6), dtype=np.float32)
        chunk[:n] = self.A[i : i + n]
        chunk[n:] = self.A[i + n - 1]
        chunk = (chunk - self.a_mean) / self.a_std
        pad = np.zeros(CHUNK, dtype=bool)
        pad[n:] = True

        s = self._quant(self.S[i])
        if self.arm == "base":
            state = (s - self.s_mean) / self.s_std
        elif self.arm == "hist":
            a_prev = s if self.first[i] else self.A_prev[i]
            state = np.concatenate(
                [(s - self.s_mean) / self.s_std, (a_prev - self.a_mean) / self.a_std]
            )
        else:
            delta = np.zeros(6, np.float32) if self.first[i] else self.A_prev[i] - s
            state = np.concatenate(
                [(s - self.s_mean) / self.s_std, (delta - self.d_mean) / self.d_std]
            )

        row = self.hf[int(i)]
        item = {
            "observation.state": torch.from_numpy(state),
            "action": torch.from_numpy(chunk),
            "action_is_pad": torch.from_numpy(pad),
        }
        for cam in ("front", "gripper_fpv"):
            img = np.asarray(row[f"observation.images.{cam}"])
            t = torch.from_numpy(img.copy())
            if t.ndim == 3 and t.shape[-1] == 3:      # HWC -> CHW
                t = t.permute(2, 0, 1)
            t = t.float()
            if t.max() > 1.5:                          # uint8 range
                t = t / 255.0
            item[f"observation.images.{cam}"] = t
        return item


def build_policy(state_dim: int, image_size: int = 96):
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    cfg = ACTConfig(
        input_features={
            "observation.state": PolicyFeature(FeatureType.STATE, (state_dim,)),
            "observation.images.front": PolicyFeature(
                FeatureType.VISUAL, (3, image_size, image_size)
            ),
            "observation.images.gripper_fpv": PolicyFeature(
                FeatureType.VISUAL, (3, image_size, image_size)
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
    return ACTPolicy(cfg)


def train(args):
    ds = load_dataset(args.root)
    quantum = 16.0 if args.arm == "delta_q16" else 1.0
    mode = {"base": "base", "base_hist": "hist"}.get(args.arm, "delta")
    wrapped = FastChunkDataset(ds, mode, quantum)
    state_dim = 6 if args.arm == "base" else 12
    policy = build_policy(state_dim, args.image_size).to(DEVICE)
    policy.train()

    loader = torch.utils.data.DataLoader(
        wrapped,
        batch_size=args.batch,
        shuffle=True,
        num_workers=2,
        drop_last=True,
    )
    opt = torch.optim.AdamW(policy.parameters(), lr=1e-4, weight_decay=1e-4)
    scaler = torch.amp.GradScaler(enabled=DEVICE == "cuda")

    loss_log = Path("results") / f"act_{args.arm}_loss.csv"
    loss_log.parent.mkdir(exist_ok=True)
    loss_log.write_text("step,loss,it_per_s\n")
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
            if step % 100 == 0:
                with loss_log.open("a") as fh:
                    fh.write(f"{step},{loss.item():.5f},"
                             f"{step / (time.time() - t0):.2f}\n")
            if step % 500 == 0:
                print(
                    f"  step {step}/{args.steps} loss {loss.item():.4f} "
                    f"({step / (time.time() - t0):.1f} it/s)",
                    flush=True,
                )
            if step % 2000 == 0:
                ck = Path(args.out) / (
                    args.arm if args.seed == 0 else f"{args.arm}_s{args.seed}"
                )
                ck.mkdir(parents=True, exist_ok=True)
                policy.save_pretrained(ck)      # rolling checkpoint
            if step >= args.steps:
                break
    tag = args.arm if args.seed == 0 else f"{args.arm}_s{args.seed}"
    out = Path(args.out) / tag
    out.mkdir(parents=True, exist_ok=True)
    policy.save_pretrained(out)
    np.savez(out / "norm_stats.npz", s_mean=wrapped.s_mean, s_std=wrapped.s_std,
             a_mean=wrapped.a_mean, a_std=wrapped.a_std,
             d_mean=wrapped.d_mean, d_std=wrapped.d_std)
    print(f"saved {out}")
    return policy, wrapped


@torch.no_grad()
def evaluate(policy, data, args):
    """Closed-loop rollout in the env at the demo control rate (15 Hz)."""
    policy.eval()
    quantum = 16.0 if args.arm == "delta_q16" else 1.0
    results = []
    for crush in args.eval_crush:
        env = DemoEnv(
            seed=1000,
            cameras=("front", "gripper_fpv"),
            image_size=args.image_size,
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
                s_n = (s - torch.from_numpy(data.s_mean)) / torch.from_numpy(data.s_std)
                if args.arm == "base":
                    s_in = s_n
                elif args.arm == "base_hist":
                    ap = a_prev if a_prev is not None else s
                    ap_n = (ap - torch.from_numpy(data.a_mean)) / torch.from_numpy(data.a_std)
                    s_in = torch.cat([s_n, ap_n])
                else:
                    d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
                    d_n = (d - torch.from_numpy(data.d_mean)) / torch.from_numpy(data.d_std)
                    s_in = torch.cat([s_n, d_n])
                obs = {
                    "observation.state": s_in.unsqueeze(0).to(DEVICE),
                    "observation.images.front": torch.as_tensor(
                        scene.image("front")
                    ).permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
                    "observation.images.gripper_fpv": torch.as_tensor(
                        scene.image("gripper_fpv")
                    ).permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
                }
                a_norm = policy.select_action(obs).squeeze(0).cpu()
                action = a_norm * torch.from_numpy(data.a_std) + torch.from_numpy(
                    data.a_mean
                )
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
            dict(crush=float(crush), success=int(n_ok), crushed=int(n_crush),
                 dropped=int(n_drop), episodes=int(args.eval_episodes))
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
    parser.add_argument(
        "--arm", choices=("base", "delta", "delta_q16", "base_hist"), required=True
    )
    parser.add_argument("--image-size", type=int, default=96)
    parser.add_argument("--root", default="data/demos_native")
    parser.add_argument("--out", default="checkpoints/act")
    parser.add_argument("--steps", type=int, default=8000)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--eval-episodes", type=int, default=30)
    parser.add_argument("--eval-crush", type=float, nargs="*",
                        default=[-1.0, 120.0, 60.0])
    parser.add_argument("--json", default=None)
    parser.add_argument("--seed", type=int, default=0,
                        help="training seed (weights, shuffling); env eval uses its own")
    args = parser.parse_args()
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    policy, data = train(args)
    results = evaluate(policy, data, args)
    out = args.json or f"results/act_{args.arm}.json"
    Path(out).parent.mkdir(exist_ok=True)
    with open(out, "w") as fh:
        json.dump(dict(arm=args.arm, steps=args.steps, results=results), fh,
                  indent=2)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
