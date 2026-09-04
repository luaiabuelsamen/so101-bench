"""Train ACT on a real SO-101 teleop dataset.

scripts/train_act.py is the sim grid's trainer and cannot read real recordings:
it demands two cameras (`front` + `gripper_fpv`), assumes `observation.state` is
6 positions, and pulls images straight out of `hf_dataset`, which holds nothing
for a video-backed dataset. This is the real-data path. The arm definitions are
kept deliberately identical in spirit so sim and real results stay comparable.

Real recordings differ in three ways that matter:

  cameras   one overhead `front`. No wrist view, so no camera can see the jaw
            or the object deforming -- the occlusion control of roadmap :217
            holds here by construction rather than by design.
  state     12 dims: 6 positions THEN 6 Present_Load. The channel is computed
            against the position slice only; the load half is never fed to the
            policy, since feeding the servo's own force register would be a
            different experiment (and it saturates -- see
            scripts/real_channel_saturation.py).
  images    stored as AV1 video, so every frame is decoded once up-front into a
            uint8 cache next to the dataset. 21k frames at 96px is ~600 MB and
            makes the loader trivial instead of ruinous.

Arms:
    base     s[t][:6]                          6-dim, stock ACT
    hist     [s[t][:6], a[t-1]]               12-dim, the action-history baseline
    delta    [s[t][:6], a[t-1]-s[t][:6]]      12-dim, the raw channel
    excess   [s[t][:6], lag excess]           12-dim, free-motion compensated
    ghist    [s[t][:6], s[t-k][:6]]           12-dim, ARM G: position context
                                              only, no a[t-1]. The control that
                                              separates "context" from "force"
                                              (see docs/reading_notes thread A).

    python scripts/train_act_real.py --arm base --root data/real/pickplace_real_v0 \
        --steps 3000 --out checkpoints/real_base
"""

import argparse
import glob
import json
import time
from pathlib import Path

import numpy as np
import torch

CHUNK = 30
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
ARMS = ("base", "hist", "delta", "excess", "ghist")


# --------------------------------------------------------------------------- data


def decode_video_cache(root: Path, image_size: int, n_frames: int) -> np.ndarray:
    """Decode the dataset's video once into a uint8 [N,H,W,3] cache."""
    cache = root / f"frames_{image_size}.npy"
    if cache.exists():
        arr = np.load(cache, mmap_mode="r")
        if len(arr) == n_frames:
            print(f"  using cached frames {cache.name} {arr.shape}")
            return arr
        print(f"  cache has {len(arr)} frames, need {n_frames}; re-decoding")

    import av
    import cv2

    sources = sorted(glob.glob(str(root / "videos" / "**" / "*.mp4"), recursive=True))
    if not sources:
        raise SystemExit(f"no video under {root}/videos")

    out = np.zeros((n_frames, image_size, image_size, 3), np.uint8)
    i = 0
    for src in sources:
        with av.open(src) as container:
            for frame in container.decode(video=0):
                if i >= n_frames:
                    break
                img = frame.to_ndarray(format="rgb24")
                out[i] = cv2.resize(img, (image_size, image_size), interpolation=cv2.INTER_AREA)
                i += 1
                if i % 5000 == 0:
                    print(f"  decoded {i}/{n_frames}", flush=True)
    if i != n_frames:
        raise SystemExit(f"decoded {i} frames but parquet has {n_frames}; refusing to guess alignment")
    np.save(cache, out)
    print(f"  wrote {cache.name} ({out.nbytes/1e6:.0f} MB)")
    return out


class RealChunkDataset(torch.utils.data.Dataset):
    def __init__(self, root: Path, arm: str, image_size: int = 96, ghist_k: int = 8,
                 trim_static: bool = True):
        import pyarrow as pa
        import pyarrow.parquet as pq

        files = sorted(glob.glob(str(root / "data" / "**" / "*.parquet"), recursive=True))
        t = pa.concat_tables([pq.read_table(f) for f in files])
        state = np.array(t["observation.state"].to_pylist(), np.float32)
        self.A = np.array(t["action"].to_pylist(), np.float32)
        ep = np.array(t["episode_index"].to_pylist(), np.int64)

        self.S = state[:, :6]              # positions; the load half is not an input
        self.arm, self.k = arm, ghist_k

        self.s_mean, self.s_std = self.S.mean(0), self.S.std(0) + 1e-3
        self.a_mean, self.a_std = self.A.mean(0), self.A.std(0) + 1e-3
        self.d_mean = np.zeros(6, np.float32)
        self.d_std = np.full(6, 8.0, np.float32)

        # episode end (exclusive) per frame, for chunk padding
        ends = np.empty(len(ep), np.int64)
        start = 0
        for i in range(1, len(ep) + 1):
            if i == len(ep) or ep[i] != ep[i - 1]:
                ends[start:i] = i
                start = i
        self.ep_end = ends

        same = np.zeros(len(ep), bool)
        same[1:] = ep[1:] == ep[:-1]
        self.first = ~same
        self.A_prev = np.zeros_like(self.A)
        self.A_prev[same] = self.A[np.flatnonzero(same) - 1]

        # index of the frame k steps back, clamped inside the episode
        starts = np.empty(len(ep), np.int64)
        s0 = 0
        for i in range(1, len(ep) + 1):
            if i == len(ep) or ep[i] != ep[i - 1]:
                starts[s0:i] = s0
                s0 = i
        self.back_k = np.maximum(np.arange(len(ep)) - ghist_k, starts)

        if arm == "excess":
            # k_hat per joint from free-motion frames, fit before training and
            # frozen. Jaw-open frames stand in for free motion: the only contact
            # in this task is the jaw's.
            A_prev2 = np.zeros_like(self.A)
            same2 = np.zeros(len(ep), bool)
            same2[2:] = ep[2:] == ep[:-2]
            A_prev2[same2] = self.A[np.flatnonzero(same2) - 2]
            self.rate = np.where(same2[:, None], self.A_prev - A_prev2, 0.0).astype(np.float32)
            delta_all = np.where(same[:, None], self.A_prev - self.S, 0.0)
            free = self.A[:, 5] > np.quantile(self.A[:, 5], 0.30)
            self.k_hat = np.zeros(6, np.float32)
            for j in range(6):
                r, d_ = self.rate[free, j], delta_all[free, j]
                den = float((r * r).sum())
                self.k_hat[j] = float((r * d_).sum() / den) if den > 1e-6 else 0.0
            print(f"  k_hat = {np.round(self.k_hat, 3).tolist()}")

        # 49 of 50 recorded episodes open with a stationary pause (median 1.9 s):
        # the operator settles before starting. A policy trained on those frames
        # correctly learns "at the start pose, hold still", and in closed loop
        # that is an absorbing state -- the observation never changes, so it
        # never leaves the pause. Teacher forcing hides this because recorded
        # observations advance regardless. Drop the dead prefix, keeping a short
        # lead-in so the transition into motion is still represented.
        self.index = np.arange(len(self.S))
        if trim_static:
            keep = []
            for e in sorted(set(ep.tolist())):
                idx = np.flatnonzero(ep == e)
                mot = np.abs(np.diff(self.A[idx], axis=0)).sum(1)
                moving = np.flatnonzero(mot > 1.0)
                first = max(0, int(moving[0]) - 5) if len(moving) else 0
                keep.append(idx[first:])
            self.index = np.concatenate(keep)
            print(f"  trim_static: {len(self.S)} -> {len(self.index)} frames "
                  f"({100*(1-len(self.index)/len(self.S)):.0f}% of frames were dead prefix)")

        self.frames = decode_video_cache(root, image_size, len(self.S))

    def __len__(self):
        return len(self.index)

    def __getitem__(self, i):
        i = int(self.index[i])
        end = self.ep_end[i]
        n = min(CHUNK, end - i)
        chunk = np.empty((CHUNK, 6), np.float32)
        chunk[:n] = self.A[i : i + n]
        chunk[n:] = self.A[i + n - 1]
        chunk = (chunk - self.a_mean) / self.a_std
        pad = np.zeros(CHUNK, bool)
        pad[n:] = True

        s = self.S[i]
        s_n = (s - self.s_mean) / self.s_std
        delta = np.zeros(6, np.float32) if self.first[i] else self.A_prev[i] - s

        if self.arm == "base":
            state = s_n
        elif self.arm == "hist":
            a_prev = s if self.first[i] else self.A_prev[i]
            state = np.concatenate([s_n, (a_prev - self.a_mean) / self.a_std])
        elif self.arm == "excess":
            excess = delta - self.k_hat * self.rate[i]
            state = np.concatenate([s_n, (excess - self.d_mean) / self.d_std])
        elif self.arm == "ghist":
            past = (self.S[self.back_k[i]] - self.s_mean) / self.s_std
            state = np.concatenate([s_n, past])
        else:  # delta
            state = np.concatenate([s_n, (delta - self.d_mean) / self.d_std])

        img = torch.from_numpy(np.asarray(self.frames[i]).copy()).permute(2, 0, 1).float() / 255.0
        return {
            "observation.state": torch.from_numpy(state.astype(np.float32)),
            "action": torch.from_numpy(chunk),
            "action_is_pad": torch.from_numpy(pad),
            "observation.images.front": img,
        }


# -------------------------------------------------------------------------- policy


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=ARMS, required=True)
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--steps", type=int, default=3000)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--lr", type=float, default=1e-4)
    ap.add_argument("--image-size", type=int, default=96)
    ap.add_argument("--ghist-k", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--keep-static", action="store_true",
                    help="keep the stationary prefix of each episode (default: drop it)")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    print(f"arm={args.arm} steps={args.steps} device={DEVICE}")
    ds = RealChunkDataset(Path(args.root).resolve(), args.arm, args.image_size,
                          args.ghist_k, trim_static=not args.keep_static)
    state_dim = 6 if args.arm == "base" else 12
    print(f"  {len(ds)} frames, state_dim={state_dim}")

    policy = build_policy(state_dim, args.image_size).to(DEVICE)
    policy.train()
    opt = torch.optim.AdamW(policy.parameters(), lr=args.lr, weight_decay=1e-4)
    loader = torch.utils.data.DataLoader(
        ds, batch_size=args.batch, shuffle=True, num_workers=2,
        pin_memory=(DEVICE == "cuda"), drop_last=True, persistent_workers=True,
    )

    losses, step, t0 = [], 0, time.time()
    while step < args.steps:
        for batch in loader:
            if step >= args.steps:
                break
            batch = {k: v.to(DEVICE, non_blocking=True) for k, v in batch.items()}
            loss, _ = policy.forward(batch)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0)
            opt.step()
            opt.zero_grad(set_to_none=True)
            losses.append(float(loss))
            step += 1
            if step % 100 == 0:
                recent = float(np.mean(losses[-100:]))
                print(f"  step {step:5d}/{args.steps}  loss {recent:.4f}  "
                      f"{(time.time()-t0)/step:.2f}s/step", flush=True)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    policy.save_pretrained(out)
    np.savez(out / "norm_stats.npz", s_mean=ds.s_mean, s_std=ds.s_std,
             a_mean=ds.a_mean, a_std=ds.a_std,
             k_hat=getattr(ds, "k_hat", np.zeros(6, np.float32)))
    first = float(np.mean(losses[:100])) if len(losses) >= 100 else float(np.mean(losses))
    last = float(np.mean(losses[-100:]))
    summary = dict(arm=args.arm, steps=args.steps, state_dim=state_dim,
                   loss_first100=first, loss_last100=last,
                   minutes=(time.time() - t0) / 60, frames=len(ds), seed=args.seed)
    (out / "train_summary.json").write_text(json.dumps(summary, indent=2))
    print(f"\nloss {first:.4f} -> {last:.4f} over {args.steps} steps "
          f"({summary['minutes']:.1f} min); saved {out}")


if __name__ == "__main__":
    main()
