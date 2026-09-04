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

    def __init__(self, ds, arm: str, quantum: float = 1.0,
                 max_episodes: int | None = None):
        self.arm = arm
        self.q = quantum
        self.max_episodes = max_episodes
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
        if max_episodes is not None:
            # front-truncation only: keeps frame indices aligned with
            # self.hf, which __getitem__ still indexes for images
            n = int(np.searchsorted(ep, max_episodes))
            self.S, self.A, ep = self.S[:n], self.A[:n], ep[:n]
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
        # a[t-2] within the episode, and the index of the frame k steps back
        # clamped to the episode start. Needed by hist2 and ghist; excess
        # recomputes its own copy below so its behaviour is untouched.
        self.A_prev2 = np.zeros_like(self.A)
        same_2 = np.zeros(len(ep), dtype=bool)
        same_2[2:] = ep[2:] == ep[:-2]
        self.A_prev2[same_2] = self.A[np.flatnonzero(same_2) - 2]
        self.first2 = ~same_2
        starts = np.empty(len(ep), dtype=np.int64)
        s0 = 0
        for i in range(1, len(ep) + 1):
            if i == len(ep) or ep[i] != ep[i - 1]:
                starts[s0:i] = s0
                s0 = i
        self.back_k = np.maximum(np.arange(len(ep)) - 8, starts)

        if arm == "excess":
            # E (load_prior): delta minus the free-motion lag baseline.
            # K_hat per joint fit by least squares on jaw-open frames (the
            # only contact in this task is the jaw's, so jaw-open ~= free
            # motion) -- label-free self-labeling, frozen before training.
            A_prev2 = np.zeros_like(self.A)
            same2 = np.zeros(len(ep), dtype=bool)
            same2[2:] = ep[2:] == ep[:-2]
            A_prev2[same2] = self.A[np.flatnonzero(same2) - 2]
            self.rate = np.where(same2[:, None], self.A_prev - A_prev2, 0.0)
            delta_all = np.where(same[:, None], self.A_prev - self.S, 0.0)
            free = self.A[:, 5] > np.quantile(self.A[:, 5], 0.30)
            self.k_hat = np.zeros(6, np.float32)
            for j in range(6):
                r, d_ = self.rate[free, j], delta_all[free, j]
                denom = float((r * r).sum())
                self.k_hat[j] = float((r * d_).sum() / denom) if denom > 1e-6 else 0.0

        if arm == "oracle":
            # ceiling arm: privileged true grip force as input, fixed scale
            # 40 N (forces 0-285 N -> ~0-7). Bounds what ANY observation of
            # contact could deliver on this task.
            self.F = np.asarray(
                hf["grip_force_N"], dtype=np.float32).reshape(-1)[:len(self.S)] / 40.0

        if arm == "token":
            # F (event_token): the frozen guard-v4 DETECTOR (no cap, no rate
            # limit -- recorded commands are already the applied ones) run
            # causally over each episode's (s[t], a[t-1]) jaw pairs, matching
            # what the runtime sees before acting at t.
            from so101_bench.guard import JawGuard
            seat = np.zeros(len(self.S), np.float32)
            start = 0
            for i in range(1, len(ep) + 1):
                if i == len(ep) or ep[i] != ep[i - 1]:
                    g = JawGuard(max_close_rate=1e9)
                    for t in range(start, i):
                        g(float(self.S[t, 5]), float(self.A_prev[t, 5]))
                        seat[t] = 0.0 if g.seat_jaw is None else 1.0
                    start = i
            self.seat = seat

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
        s_n = (s - self.s_mean) / self.s_std
        delta = np.zeros(6, np.float32) if self.first[i] else self.A_prev[i] - s
        delta_target = None
        if self.arm == "base":
            state = s_n
        elif self.arm == "hist":
            a_prev = s if self.first[i] else self.A_prev[i]
            state = np.concatenate(
                [s_n, (a_prev - self.a_mean) / self.a_std]
            )
        elif self.arm == "hist2":
            # B2: raw two-step action history. Same raw inputs as excess
            # (s[t], a[t-1], a[t-2]), different representation, so E - B2
            # isolates the free-motion subtraction with information held fixed.
            a_prev = s if self.first[i] else self.A_prev[i]
            a_prev2 = a_prev if self.first2[i] else self.A_prev2[i]
            state = np.concatenate(
                [s_n,
                 (a_prev - self.a_mean) / self.a_std,
                 (a_prev2 - self.a_mean) / self.a_std]
            )
        elif self.arm == "ghist":
            # G: position context only, no a[t-1] in any form. Tests whether
            # state history substitutes for the channel (zeng2026revisiting).
            past = (self.S[self.back_k[i]] - self.s_mean) / self.s_std
            state = np.concatenate([s_n, past])
        elif self.arm == "resid":
            state = s_n
            delta_target = (delta - self.d_mean) / self.d_std
        elif self.arm == "excess":
            excess = delta - self.k_hat * self.rate[i].astype(np.float32)
            state = np.concatenate(
                [s_n, (excess - self.d_mean) / self.d_std]
            )
        elif self.arm == "token":
            state = np.concatenate(
                [s_n, np.full(6, self.seat[i], np.float32)]
            )
        elif self.arm == "oracle":
            state = np.concatenate([s_n, np.full(6, self.F[i], np.float32)])
        else:
            state = np.concatenate(
                [s_n, (delta - self.d_mean) / self.d_std]
            )

        row = self.hf[int(i)]
        item = {
            "observation.state": torch.from_numpy(state),
            "action": torch.from_numpy(chunk),
            "action_is_pad": torch.from_numpy(pad),
        }
        if delta_target is not None:
            item["delta_target"] = torch.from_numpy(delta_target)
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


def build_policy(state_dim: int, image_size: int = 96, aux: bool = False):
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    class ResidualACT(ACTPolicy):
        """Arm D: base observation only; the encoder additionally predicts
        delta[t] (normalized) through a small head, weight 0.1. The channel
        enters as a training signal, never as an input. NOTE: re-loading
        this arm from disk needs strict=False (extra aux_head keys); the
        grid evaluates in-process so the plain path is unaffected."""

        def __init__(self, cfg):
            super().__init__(cfg)
            d = cfg.dim_model
            self.aux_head = torch.nn.Sequential(
                torch.nn.Linear(d, 64), torch.nn.ReLU(), torch.nn.Linear(64, 6)
            )
            self._enc = None
            self.model.encoder.register_forward_hook(
                lambda m, inp, out: setattr(self, "_enc", out)
            )

        def forward(self, batch):
            loss, info = super().forward(batch)
            enc = self._enc
            b = batch["observation.state"].shape[0]
            lat = enc.mean(dim=0) if enc.shape[1] == b else enc.mean(dim=1)
            aux_mse = torch.nn.functional.mse_loss(
                self.aux_head(lat), batch["delta_target"]
            )
            return loss + 0.1 * aux_mse, dict(info, aux_mse=float(aux_mse))

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
    return ResidualACT(cfg) if aux else ACTPolicy(cfg)


def train(args):
    ds = load_dataset(args.root)
    quantum = 16.0 if args.arm == "delta_q16" else 1.0
    mode = {"base": "base", "base_hist": "hist", "base_hist2": "hist2",
            "ghist": "ghist", "resid": "resid",
            "excess": "excess", "token": "token",
            "oracle": "oracle"}.get(args.arm, "delta")
    wrapped = FastChunkDataset(ds, mode, quantum,
                               max_episodes=args.max_episodes)
    state_dim = 6 if args.arm in ("base", "resid") else (
        18 if args.arm == "base_hist2" else 12)
    policy = build_policy(state_dim, args.image_size,
                          aux=(args.arm == "resid")).to(DEVICE)
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

    tag = args.arm if args.seed == 0 else f"{args.arm}_s{args.seed}"
    loss_log = Path(args.out) / f"{tag}_loss.csv"
    loss_log.parent.mkdir(parents=True, exist_ok=True)
    loss_log.write_text("step,loss,it_per_s\n")
    wb = None
    if args.wandb:
        try:
            import wandb as wb
            wb.init(project="so101-bench", name=f"{args.arm}-s{args.seed}",
                    config=vars(args))
        except Exception as e:
            print(f"wandb disabled: {e}")
            wb = None
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
                if wb is not None:
                    wb.log({"loss": loss.item(),
                            "it_per_s": step / (time.time() - t0)}, step=step)
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
    extra = {"k_hat": wrapped.k_hat} if hasattr(wrapped, "k_hat") else {}
    np.savez(out / "norm_stats.npz", s_mean=wrapped.s_mean, s_std=wrapped.s_std,
             a_mean=wrapped.a_mean, a_std=wrapped.a_std,
             d_mean=wrapped.d_mean, d_std=wrapped.d_std, **extra)
    print(f"saved {out}")
    return policy, wrapped


@torch.no_grad()
def evaluate(policy, data, args, shift_fn=None):
    """Closed-loop rollout in the env at the demo control rate (15 Hz).

    ``shift_fn(env, args.shift)`` mutates plant parameters after env
    construction (object-property shift evals, roadmap H)."""
    policy.eval()
    quantum = 16.0 if args.arm == "delta_q16" else 1.0
    results = []
    for crush in args.eval_crush:
        env = DemoEnv(
            seed=getattr(args, "eval_seed", 1000),
            cameras=("front", "gripper_fpv"),
            image_size=args.image_size,
            stride=2,
            obs_quantum=quantum,          # ONLINE coarsening at eval
            crush_newtons=None if crush <= 0 else crush,
            expert=ExpertConfig(),        # unused; env provides scene/reset
        )
        if shift_fn is not None:
            shift_fn(env, getattr(args, "shift", {}))
        n_ok = n_crush = n_drop = 0
        for _ep in range(args.eval_episodes):
            scene = env.scene
            scene.reset()
            policy.reset()
            z0 = scene.block_pos()[2]
            a_prev = None
            a_prev2 = None
            s_hist = []          # observed states this episode, for ghist
            peak = 0.0
            if args.arm == "token":
                from so101_bench.guard import JawGuard
                gd = JawGuard(max_close_rate=1e9)   # detector only, no cap
            for _t in range(260):        # 260 x (2/30 s) ~ 17 s budget
                s = torch.as_tensor(scene.state_ticks(), dtype=torch.float32)
                s_hist.append(s)
                s_n = (s - torch.from_numpy(data.s_mean)) / torch.from_numpy(data.s_std)
                if args.arm in ("base", "resid"):
                    s_in = s_n
                elif args.arm == "base_hist":
                    ap = a_prev if a_prev is not None else s
                    ap_n = (ap - torch.from_numpy(data.a_mean)) / torch.from_numpy(data.a_std)
                    s_in = torch.cat([s_n, ap_n])
                elif args.arm == "excess":
                    d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
                    r = (a_prev - a_prev2) if a_prev2 is not None else torch.zeros_like(s)
                    ex = d - torch.from_numpy(np.asarray(data.k_hat, np.float32)) * r
                    ex_n = (ex - torch.from_numpy(data.d_mean)) / torch.from_numpy(data.d_std)
                    s_in = torch.cat([s_n, ex_n])
                elif args.arm == "token":
                    jaw_prev = float(a_prev[5]) if a_prev is not None else float(s[5])
                    gd(float(s[5]), jaw_prev)
                    flag = 0.0 if gd.seat_jaw is None else 1.0
                    s_in = torch.cat([s_n, torch.full((6,), flag)])
                elif args.arm == "oracle":
                    f = float(scene.grip_force()) / 40.0
                    s_in = torch.cat([s_n, torch.full((6,), f)])
                elif args.arm == "base_hist2":
                    ap = a_prev if a_prev is not None else s
                    ap2 = a_prev2 if a_prev2 is not None else ap
                    am = torch.from_numpy(data.a_mean)
                    asd = torch.from_numpy(data.a_std)
                    s_in = torch.cat([s_n, (ap - am) / asd, (ap2 - am) / asd])
                elif args.arm == "ghist":
                    # s[t-8], clamped to the episode start, matching back_k in
                    # the dataset. s_hist already contains the current state.
                    past = s_hist[max(0, len(s_hist) - 9)]
                    past_n = (past - torch.from_numpy(data.s_mean)) / torch.from_numpy(
                        data.s_std
                    )
                    s_in = torch.cat([s_n, past_n])
                elif args.arm in ("delta", "delta_q16"):
                    d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
                    d_n = (d - torch.from_numpy(data.d_mean)) / torch.from_numpy(data.d_std)
                    s_in = torch.cat([s_n, d_n])
                else:
                    # Deliberately exhaustive. This chain used to end in a
                    # catch-all that built the delta observation, so an arm
                    # added to training without a branch here trained on its
                    # own observation and rolled out on delta's -- same width,
                    # no error, a plausible and wrong number.
                    raise ValueError(
                        f"no rollout observation defined for arm {args.arm!r}; "
                        "add a branch here when adding an arm"
                    )
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
                a_prev2 = a_prev
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
        "--arm",
        choices=("base", "delta", "delta_q16", "base_hist", "base_hist2",
                 "ghist", "resid", "excess", "token", "oracle"),
        required=True,
    )
    parser.add_argument("--image-size", type=int, default=96)
    parser.add_argument("--max-episodes", type=int, default=None,
                        help="train on only the first N episodes (front-"
                             "truncated; Task 2 uses 200 of demos_v3's 280)")
    parser.add_argument("--wandb", action="store_true",
                        help="log loss to wandb (needs WANDB_API_KEY in env)")
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
