"""Figure 1 (teaser): identical scene, two observers.

    python scripts/fig_teaser.py

Top row: the stock observation (arm A, joint positions only) fails.
Bottom row: arm E (same network, + the delta channel the log already
contains, lag-subtracted) succeeds. Five frames each, with the jaw |delta|
trace under each row on a shared time axis; contact and (for E) the seat
event marked. The claim in five seconds: same scene, same data budget,
the only difference is what the policy is allowed to see.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent))
from so101_bench import DemoEnv, ExpertConfig  # noqa: E402
from so101_bench.scene import JAW_SHUT, RAD_PER_TICK  # noqa: E402

DEV = "cuda" if torch.cuda.is_available() else "cpu"
FRAME_PX = 256


def rollout(policy, st, arm, seed_env, episodes=12):
    """Run episodes; return per-episode dicts with frames + traces."""
    # policy sees the NATIVE 96px render it trained on; print frames come
    # from a second renderer on the same scene state (a 256->96 downsample
    # is not the training distribution -- E went 0-for-8 under it)
    import mujoco
    env = DemoEnv(seed=seed_env, cameras=("front", "gripper_fpv"),
                  image_size=96, stride=2, expert=ExpertConfig())
    hi_renderer = mujoco.Renderer(env.scene.model, FRAME_PX, FRAME_PX)
    scene = env.scene
    eps = []
    k_hat = torch.from_numpy(np.asarray(st.get("k_hat", np.zeros(6)), np.float32))
    for _ in range(episodes):
        scene.reset()
        policy.reset()
        z0 = scene.block_pos()[2]
        a_prev = a_prev2 = None
        frames, djaw, forces, rises = [], [], [], []
        for t in range(260):        # full eval-protocol length; placements land late
            s = torch.as_tensor(scene.state_ticks(), dtype=torch.float32)
            s_n = (s - torch.from_numpy(st["s_mean"])) / torch.from_numpy(st["s_std"])
            if arm == "base":
                s_in = s_n
            else:                                  # excess
                d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
                r = (a_prev - a_prev2) if a_prev2 is not None else torch.zeros_like(s)
                ex = d - k_hat * r
                s_in = torch.cat([s_n, (ex - torch.from_numpy(st["d_mean"]))
                                  / torch.from_numpy(st["d_std"])])
            obs = {"observation.state": s_in.unsqueeze(0).to(DEV)}
            for cam in ("front", "gripper_fpv"):
                obs[f"observation.images.{cam}"] = (
                    torch.as_tensor(scene.image(cam)).permute(2, 0, 1)
                    .float().div(255).unsqueeze(0).to(DEV))
            with torch.no_grad():
                a_n = policy.select_action(obs).squeeze(0).cpu()
            action = a_n * torch.from_numpy(st["a_std"]) + torch.from_numpy(st["a_mean"])
            a_prev2, a_prev = a_prev, action.clone()
            cmd = action.numpy().astype(float)
            cmd[5] = max(cmd[5], JAW_SHUT / RAD_PER_TICK)
            scene.hold(cmd * RAD_PER_TICK, frames=2)
            if t % 2 == 0:
                hi_renderer.update_scene(scene.data, camera="front")
                frames.append(hi_renderer.render().copy())
            djaw.append(abs(float(action[5]) - float(scene.state_ticks()[5])))
            forces.append(scene.grip_force())
            rises.append(scene.block_pos()[2] - z0)
        picked = max(rises) > 0.03
        placed = picked and scene.block_in_box() and not scene.crushed
        eps.append(dict(frames=frames, djaw=np.array(djaw),
                        forces=np.array(forces), rises=np.array(rises),
                        picked=picked, placed=placed))
    env.close()
    return eps


def _image_size_ok(scene):
    try:
        scene.image("front", size=FRAME_PX)
        return True
    except TypeError:
        return False


def pick_frames(ep, n=5):
    T = len(ep["djaw"])
    contact = np.flatnonzero(ep["forces"] > 0.5)
    c = int(contact[0]) if len(contact) else T // 2
    lift = int(np.argmax(ep["rises"]))
    idx = sorted({max(c - 40, 0), c, min(c + 25, T - 1),
                  min(lift, T - 1), T - 1})
    while len(idx) < n:
        idx.append(T - 1)
    return [min(i // 2, len(ep["frames"]) - 1) for i in idx[:n]], idx[:n], c


def main():
    import json
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from lerobot.policies.act.modeling_act import ACTPolicy

    rows = []
    for label, ckpt, arm, want in (
        ("stock observation: joint positions only",
         "checkpoints/grid_A_base_s0/base", "base", "fail"),
        ("with lag-excess channel, computed from the same log",
         "checkpoints/modal_E_s0/excess", "excess", "success"),
    ):
        pol = ACTPolicy.from_pretrained(ckpt).to(DEV).eval()
        st = dict(np.load(Path(ckpt) / "norm_stats.npz"))
        eps = rollout(pol, st, arm, seed_env=4200)
        if want == "success":
            sel = next((e for e in eps if e["placed"]), None)
        else:
            # match the caption: grips and releases WITHOUT lifting
            sel = next((e for e in eps if not e["picked"]), None) or \
                  next((e for e in eps if not e["placed"]), None)
        if sel is None:
            sel = eps[0]
        rows.append((label, sel))
        print(f"{label}: picked={sel['picked']} placed={sel['placed']}")

    fig = plt.figure(figsize=(10, 6.0))
    gs = fig.add_gridspec(5, 5, height_ratios=[2.2, 0.7, 0.45, 2.2, 0.7],
                          hspace=0.16, wspace=0.03)
    row_base = [0, 3]
    for r, (label, ep) in enumerate(rows):
        fidx, tidx, c = pick_frames(ep)
        for k in range(5):
            ax = fig.add_subplot(gs[row_base[r], k])
            ax.imshow(ep["frames"][fidx[k]])
            ax.set_xticks([]), ax.set_yticks([])
            if k == 0:
                ax.set_ylabel(["A (stock)", "E (ours)"][r], fontsize=9)
            if k == 2:
                ax.set_title(label, fontsize=10, pad=5)
        axt = fig.add_subplot(gs[row_base[r] + 1, :])
        t = (np.arange(len(ep["djaw"])) - c) / 15.0
        axt.plot(t, ep["djaw"], color="#444444", lw=1.2)
        axt.fill_between(t, 0, ep["djaw"], color="#444444", alpha=0.15)
        axt.axvline(0, color="green", ls="--", lw=0.9)
        for ti in tidx:
            axt.axvline((ti - c) / 15.0, color="#bbbbbb", lw=0.6, zorder=0)
        axt.set_ylabel("|δ|", fontsize=8)
        axt.set_yticks([])
        axt.set_xlim(t[0], t[-1])
        if r == 1:
            axt.set_xlabel("time from contact (s)", fontsize=8)
        else:
            axt.tick_params(labelbottom=False)
    fig.text(0.995, 0.004, "simulation", ha="right", va="bottom",
             fontsize=8, color="gray")
    out = Path("figures/paper/fig_teaser.png")
    fig.savefig(out, dpi=180, bbox_inches="tight")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
