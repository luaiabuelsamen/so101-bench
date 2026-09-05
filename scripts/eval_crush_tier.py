"""Evaluate existing grid checkpoints at a crush tier.

Every cell of the six-arm grid was evaluated with the crush limit disabled
(grid_spec.json sets eval_crush [-1], and all grid_*.json contain crush -1.0
only). The task is described as force-critical, but the comparison therefore
scores place rate under conditions where CONTACT DETECTION suffices and force
MAGNITUDE cannot pay. That is a defensible choice for isolating observation
design, and it means the grid cannot reward the property that distinguishes the
channel from generic temporal context.

The distinction matters because of what the position-history control turns out
to carry. Against ground-truth grip force on demos_v3:

    |s[t]-s[t-8]|  (arm G)   contact AUC 0.780   force R^2 0.008
    |delta|        (arm C)   contact AUC 0.948   force R^2 0.749
    |e|            (arm E)   contact AUC 0.967   force R^2 0.764

G detects contact worse and carries essentially no force magnitude, yet matches
C on place rate at crush -1. The hypothesis this script tests is that the two
arms are not substitutes but different mechanisms of comparable benefit, and
that a force constraint separates them.

PREDICTION, recorded before running: at a crush tier E and C beat G, because
regulating grip into a band requires magnitude and G has none. If G holds up
under crush, the channel's distinguishing property does not pay even where it
should, and that is a finding against the paper.

Training is not required -- these checkpoints exist. Rollouts only.

    python scripts/eval_crush_tier.py --episodes 50 --crush 120
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from train_act import DEVICE, evaluate  # noqa: E402

# (checkpoint, arm key for the observation builder)
ARMS = {
    "G ghist s0": ("checkpoints/grid_G_ghist_s0/ghist", "ghist"),
    "G ghist s1": ("checkpoints/grid_G_ghist_s1/ghist", "ghist"),
    "G ghist s2": ("checkpoints/grid_G_ghist_s2/ghist", "ghist"),
    "E excess s0": ("checkpoints/modal_E_s0/excess", "excess"),
    "B2 hist2 s0": ("checkpoints/grid_H_base_hist2_s0/base_hist2", "hist2"),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--crush", type=float, nargs="+", default=[120.0])
    ap.add_argument("--eval-seed", type=int, default=1000)
    ap.add_argument("--out", default="results/crush_tier_eval.json")
    # One arm per process. Evaluating several checkpoints in a single process
    # stalls at the second: the MuJoCo EGL context is not released between
    # evaluate() calls, and the next renderer blocks on it at 0% CPU.
    ap.add_argument("--only", default=None, help="evaluate just this arm key")
    args_cli = ap.parse_args()

    from lerobot.policies.act.modeling_act import ACTPolicy

    out = []
    print(f"{'arm':>13} {'crush':>7} {'success':>9} {'crushed':>8} {'dropped':>8}")
    for name, (ckpt, arm_key) in ARMS.items():
        if args_cli.only and name != args_cli.only:
            continue
        if not Path(ckpt).exists():
            print(f"{name:>13}   MISSING {ckpt}")
            continue
        pol = ACTPolicy.from_pretrained(ckpt).to(DEVICE).eval()
        st = np.load(Path(ckpt) / "norm_stats.npz")
        data = SimpleNamespace(**{k: st[k] for k in st.files})
        args = SimpleNamespace(arm=arm_key, image_size=96,
                               eval_seed=args_cli.eval_seed,
                               eval_crush=args_cli.crush,
                               eval_episodes=args_cli.episodes)
        for r in evaluate(pol, data, args):
            out.append(dict(arm=name, arm_key=arm_key, **r))
            print(f"{name:>13} {r['crush']:>7.0f} "
                  f"{r['success']:>6}/{args_cli.episodes} "
                  f"{r['crushed']:>8} {r['dropped']:>8}", flush=True)
        Path(args_cli.out).write_text(json.dumps(out, indent=2, default=float))
    print(f"\nwrote {args_cli.out}")


if __name__ == "__main__":
    main()
