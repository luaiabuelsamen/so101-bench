"""Roadmap H: object-property shift eval on EXISTING checkpoints.

    python scripts/eval_shift.py

Does the lag-excess policy regulate contact, or replay a visual script?
Existing best-local-seed checkpoints for arms A, B, C, E are evaluated
under physics shifts never seen in training: block mass x0.5 / x1.5,
block friction x0.5, and the 60 N crush tier. Eval-only; no training.

PRE-REGISTERED (before the run): if E learned contact regulation its
margin over A persists under mass and friction shifts (graceful
degradation, ordering E > C,B > A maintained); if it learned a scripted
visual sequence, shifts collapse E toward A. Reported as a best-seed
robustness PROBE (n=50/condition), not as seed-mean claims; orderings
only, no point differences below the grid's resolution discipline.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent))
from train_act import DEVICE, evaluate  # noqa: E402
from types import SimpleNamespace  # noqa: E402

ARMS = {
    # arm-name -> (checkpoint dir, eval-arm key, best local seed value)
    "A_base": ("checkpoints/grid_A_base_s1/base_s1", "base", 15),
    "B_hist": ("checkpoints/grid_B_base_hist_s1/base_hist_s1", "base_hist", 15),
    "C_delta": ("checkpoints/act_v3full_delta_s1/delta_s1", "delta", 31),
    "E_excess": ("checkpoints/modal_E_s0/excess", "excess", 36),
}

CONDITIONS = {
    "nominal": {},
    "mass_x0.5": dict(mass=0.5),
    "mass_x1.5": dict(mass=1.5),
    "fric_x0.5": dict(friction=0.5),
    "crush_60N": dict(crush=60.0),
}


def apply_shift(env, cfg):
    scene = env.scene
    m = scene.model
    gid = scene.ids.block_geom
    bid = m.geom_bodyid[gid]
    if "mass" in cfg:
        m.body_mass[bid] *= cfg["mass"]
        m.body_inertia[bid] *= cfg["mass"]
    if "friction" in cfg:
        m.geom_friction[gid, 0] *= cfg["friction"]


def main():
    from lerobot.policies.act.modeling_act import ACTPolicy

    out = []
    print(f"{'arm':>9} {'condition':>10} {'success':>9} {'crushed':>8} {'dropped':>8}")
    for arm_name, (ckpt, arm_key, grid_score) in ARMS.items():
        assert Path(ckpt).exists(), f"missing checkpoint {ckpt}"
        pol = ACTPolicy.from_pretrained(ckpt).to(DEVICE).eval()
        st = np.load(Path(ckpt) / "norm_stats.npz")
        data = SimpleNamespace(**{k: st[k] for k in st.files})
        for cond, cfg in CONDITIONS.items():
            args = SimpleNamespace(
                arm=arm_key, image_size=96, eval_seed=1000,
                eval_crush=[cfg.get("crush", -1.0)], eval_episodes=50,
                shift=cfg,
            )
            res = evaluate(pol, data, args, shift_fn=apply_shift)
            r = res[0]
            out.append(dict(arm=arm_name, grid_n100=grid_score, cond=cond,
                            **r))
            print(f"{arm_name:>9} {cond:>10} {r['success']:>6}/50 "
                  f"{r['crushed']:>8} {r['dropped']:>8}", flush=True)
    with open("results/shift_eval.json", "w") as fh:
        json.dump(out, fh, indent=2, default=float)
    print("wrote results/shift_eval.json")


if __name__ == "__main__":
    main()
