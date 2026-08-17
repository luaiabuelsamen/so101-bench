"""Wen-style copycat probe on the trained H100 checkpoints.

    python scripts/probe_copycat.py --ckpt modal_outputs/checkpoints \
        --arms delta delta_s1 base_hist base_hist_s1 --root data/demos_v4

THE MEASUREMENT THE READING PASS DEMANDED. Wen et al. (NeurIPS 2020) define
the copycat shortcut: with expert-action autocorrelation and a recoverable
previous action, a history policy predicts a[t-1] instead of a[t]. Our
base_hist arm feeds raw a[t-1] and matches delta at scale -- the reviewer's
question is whether that gain is copycat. This probe answers it with the
literature's own diagnostics, teacher-forced over held-out-by-position
episodes of the training set.

FIGURE (drawn before code, rule 5). Two panels.
  (a) Per-arm median |a_hat[t]-a[t]| (err_true) vs |a_hat[t]-a[t-1]|
      (err_prev), encoder counts, jaw dim and all-dim; horizontal line =
      expert step size |a[t]-a[t-1]|. Copycat signature: err_prev < err_true.
      Faithful signature: err_true < err_prev ~= expert step size.
  (b) Break-following coefficient beta at commitment moments (top-decile jaw
      |a[t]-a[t-1]|): regression of (a_hat-a_prev) on (a-a_prev), jaw dim.
      Copycat -> beta ~ 0 (policy stays at the previous action exactly where
      the expert breaks autocorrelation); faithful -> beta ~ 1. Bootstrap CI
      over episodes.

PREDICTIONS REGISTERED UP FRONT. The observability thesis says both arms use
history as a force channel, not a shortcut: err_true < err_prev and beta >>
0 for BOTH delta and base_hist. The copycat account predicts base_hist shows
err_prev <= err_true or beta near 0 at breaks. Either outcome sharpens
docs/gap.md section 1.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

import sys
sys.path.insert(0, str(Path(__file__).parent))
from train_act import FastChunkDataset, load_dataset  # noqa: E402

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
JAW = 5


def arm_of(name: str) -> str:
    return "hist" if name.startswith("base_hist") else "delta"


@torch.no_grad()
def run_arm(ckpt_dir: Path, ds_raw, episodes: int, batch: int):
    from lerobot.policies.act.modeling_act import ACTPolicy

    pol = ACTPolicy.from_pretrained(ckpt_dir).to(DEVICE).eval()
    st = np.load(ckpt_dir / "norm_stats.npz")
    fcd = FastChunkDataset(ds_raw, arm=arm_of(ckpt_dir.name))
    assert np.allclose(fcd.s_mean, st["s_mean"], atol=1e-4), (
        "checkpoint norm stats disagree with dataset stats -- wrong dataset?")

    starts = np.flatnonzero(fcd.first)
    lo = int(starts[-episodes]) if episodes < len(starts) else 0
    idx = [i for i in range(lo, len(fcd)) if not fcd.first[i]]

    a_hat = np.empty((len(idx), 6), np.float32)
    for b0 in range(0, len(idx), batch):
        rows = idx[b0 : b0 + batch]
        obs = {}
        items = [fcd[i] for i in rows]
        obs["observation.state"] = torch.stack(
            [it["observation.state"] for it in items]).to(DEVICE)
        for cam in ("front", "gripper_fpv"):
            obs[f"observation.images.{cam}"] = torch.stack(
                [it[f"observation.images.{cam}"] for it in items]).to(DEVICE)
        chunk_n = pol.predict_action_chunk(obs)[:, 0].cpu().numpy()
        a_hat[b0 : b0 + len(rows)] = chunk_n * st["a_std"] + st["a_mean"]

    a_true = fcd.A[idx]
    a_prev = fcd.A_prev[idx]
    ep_of = np.cumsum(fcd.first)[idx]          # episode id per probed frame

    err_true = np.abs(a_hat - a_true)
    err_prev = np.abs(a_hat - a_prev)
    step = np.abs(a_true - a_prev)

    # break frames: top decile of expert jaw step size
    thr = np.quantile(step[:, JAW], 0.9)
    brk = step[:, JAW] >= max(thr, 1.0)
    num = (a_hat[brk, JAW] - a_prev[brk, JAW]) * (a_true[brk, JAW] - a_prev[brk, JAW])
    den = (a_true[brk, JAW] - a_prev[brk, JAW]) ** 2
    beta = float(num.sum() / den.sum())

    # bootstrap beta over episodes
    eps = np.unique(ep_of[brk])
    rng = np.random.default_rng(0)
    bs = []
    for _ in range(2000):
        pick = rng.choice(eps, len(eps))
        m = np.isin(ep_of, pick) & brk
        if den[np.isin(ep_of[brk], pick)].sum() > 0:
            mm = np.isin(ep_of[brk], pick)
            bs.append(float(num[mm].sum() / den[mm].sum()))
    blo, bhi = np.percentile(bs, [2.5, 97.5])

    return dict(
        arm=ckpt_dir.name, frames=len(idx), episodes=int(len(np.unique(ep_of))),
        err_true_all=float(np.median(err_true.mean(1))),
        err_prev_all=float(np.median(err_prev.mean(1))),
        err_true_jaw=float(np.median(err_true[:, JAW])),
        err_prev_jaw=float(np.median(err_prev[:, JAW])),
        expert_step_all=float(np.median(step.mean(1))),
        expert_step_jaw=float(np.median(step[:, JAW])),
        wen_ratio_jaw=float(np.median(err_prev[:, JAW])
                            / max(np.median(err_true[:, JAW]), 1e-6)),
        break_frames=int(brk.sum()), break_thr_counts=float(thr),
        beta=beta, beta_ci=[float(blo), float(bhi)],
    )


def figure(rows, out_png: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))
    names = [r["arm"] for r in rows]
    x = np.arange(len(rows))
    ax1.bar(x - 0.18, [r["err_true_jaw"] for r in rows], 0.36,
            label="|â - a[t]|  (truth)", color="#2b7bba")
    ax1.bar(x + 0.18, [r["err_prev_jaw"] for r in rows], 0.36,
            label="|â - a[t-1]|  (past)", color="#c44e52")
    ax1.axhline(rows[0]["expert_step_jaw"], ls="--", c="gray",
                label="expert step |a[t]-a[t-1]|")
    ax1.set_xticks(x, names, rotation=15)
    ax1.set_ylabel("median jaw error (encoder counts)")
    ax1.set_title("Copycat test: which target does the policy track?")
    ax1.legend(fontsize=8)

    ax2.errorbar(x, [r["beta"] for r in rows],
                 yerr=[[r["beta"] - r["beta_ci"][0] for r in rows],
                       [r["beta_ci"][1] - r["beta"] for r in rows]],
                 fmt="o", capsize=4, color="#2b7bba")
    ax2.axhline(1.0, ls="--", c="green", lw=0.8)
    ax2.axhline(0.0, ls="--", c="red", lw=0.8)
    ax2.set_xticks(x, names, rotation=15)
    ax2.set_ylabel(r"break-following $\beta$ (jaw, top-decile steps)")
    ax2.set_title("At commitment moments: copycat=0, faithful=1")
    ax2.set_ylim(-0.2, 1.3)
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=160)
    print(f"wrote {out_png}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ckpt", default="modal_outputs/checkpoints")
    ap.add_argument("--arms", nargs="*",
                    default=["delta", "delta_s1", "base_hist", "base_hist_s1"])
    ap.add_argument("--root", default="data/demos_v4")
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--json", default="results/probe_copycat.json")
    ap.add_argument("--fig", default="figures/paper/fig4_copycat.png")
    args = ap.parse_args()

    ds_raw = load_dataset(args.root)
    rows = []
    for arm in args.arms:
        r = run_arm(Path(args.ckpt) / arm, ds_raw, args.episodes, args.batch)
        rows.append(r)
        print(f"{r['arm']:>14}: err_true(jaw) {r['err_true_jaw']:5.2f}  "
              f"err_prev(jaw) {r['err_prev_jaw']:5.2f}  "
              f"wen_ratio {r['wen_ratio_jaw']:4.2f}  "
              f"beta {r['beta']:5.2f} [{r['beta_ci'][0]:.2f},{r['beta_ci'][1]:.2f}] "
              f"({r['break_frames']} break frames)", flush=True)

    Path(args.json).parent.mkdir(exist_ok=True)
    with open(args.json, "w") as fh:
        json.dump(rows, fh, indent=2)
    print(f"wrote {args.json}")
    figure(rows, Path(args.fig))


if __name__ == "__main__":
    main()
