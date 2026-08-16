"""Trajectory-diversity analysis of the demonstration dataset.

    python scripts/dataset_analysis.py --root data/demos_native

The UMI-style questions: how varied are the demonstrations, how collapsed is
the trajectory manifold, and how smooth/fast are the motions? Metrics are
computed in TOOL SPACE via forward kinematics on the recorded joint states
(the dataset stores encoder counts at 15 Hz), because joint-space numbers hide
what the gripper actually did.

Outputs PNGs to figures/dataset_analysis/ and prints per-episode outlier
checks (the "should we clean it?" answer).
"""

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))

from so101_bench.scene import RAD_PER_TICK, DomainRandomization, PickScene  # noqa: E402

FPS = 15.0
DT = 1.0 / FPS

# palette (dataviz reference, light mode)
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e3e2dd"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"

plt.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": GRID,
        "axes.labelcolor": INK2,
        "axes.titlecolor": INK,
        "xtick.color": INK2,
        "ytick.color": INK2,
        "text.color": INK,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "font.size": 9,
    }
)


def load_frames(root: str) -> pd.DataFrame:
    files = sorted(glob.glob(f"{root}/data/**/*.parquet", recursive=True))
    parts = []
    for f in files:
        try:
            parts.append(
                pd.read_parquet(
                    f, columns=["observation.state", "action", "episode_index",
                                "grip_force_N"]
                )
            )
        except Exception:
            pass                      # a file mid-write while collection runs
    df = pd.concat(parts, ignore_index=True)
    # drop the last episode if collection is still appending to it
    last = df["episode_index"].max()
    return df[df["episode_index"] < last].reset_index(drop=True)


def fk_tool(scene: PickScene, states: np.ndarray) -> np.ndarray:
    out = np.empty((len(states), 3))
    d = scene.data
    for i, s in enumerate(states):
        d.qpos[:6] = s * RAD_PER_TICK
        mujoco.mj_kinematics(scene.model, d)
        out[i] = scene.tcp()
    return out


def per_episode(df: pd.DataFrame, scene: PickScene):
    eps = []
    for ep, g in df.groupby("episode_index"):
        S = np.stack(g["observation.state"].to_numpy())
        A = np.stack(g["action"].to_numpy())
        grip = np.stack(g["grip_force_N"].to_numpy()).ravel()
        tool = fk_tool(scene, S)
        v = np.gradient(tool, DT, axis=0)
        speed = np.linalg.norm(v, axis=1)
        jerk = np.gradient(np.gradient(v, DT, axis=0), DT, axis=0)
        jaw_cmd = A[:, 5]
        close_i = int(np.argmax(jaw_cmd < np.percentile(jaw_cmd, 20)))
        eps.append(
            dict(
                ep=int(ep),
                n=len(g),
                duration_s=len(g) * DT,
                tool=tool,
                speed=speed,
                max_speed=float(speed.max()),
                mean_speed=float(speed.mean()),
                rms_jerk=float(np.sqrt((np.linalg.norm(jerk, axis=1) ** 2).mean())),
                peak_grip=float(grip.max()),
                grasp_x=float(tool[close_i, 0]),
                grasp_y=float(tool[close_i, 1]),
            )
        )
    return eps


def resample_path(tool: np.ndarray, n: int = 50) -> np.ndarray:
    t = np.linspace(0, 1, len(tool))
    ti = np.linspace(0, 1, n)
    return np.column_stack([np.interp(ti, t, tool[:, k]) for k in range(3)])


def fig_trajectories(eps, out: Path):
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    for ax, (i, j, xl, yl) in zip(
        axes,
        [(0, 1, "x [m]", "y [m]"), (0, 2, "x [m]", "z [m]")],
    ):
        for e in eps:
            ax.plot(e["tool"][:, i], e["tool"][:, j], color=BLUE, alpha=0.10,
                    lw=1.0)
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
    axes[0].set_title(f"Tool paths, top view ({len(eps)} episodes)")
    axes[1].set_title("Tool paths, side view")
    fig.tight_layout()
    fig.savefig(out / "trajectories.png", dpi=150)
    plt.close(fig)


def fig_pca(eps, out: Path):
    X = np.stack([resample_path(e["tool"]).ravel() for e in eps])
    Xc = X - X.mean(0)
    U, sv, _ = np.linalg.svd(Xc, full_matrices=False)
    var = sv**2 / (sv**2).sum()
    pc = U[:, :2] * sv[:2]
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    gx = np.array([e["grasp_x"] for e in eps])
    sc = axes[0].scatter(pc[:, 0], pc[:, 1], s=22, c=gx, cmap="Blues",
                         edgecolors=INK2, linewidths=0.4)
    fig.colorbar(sc, ax=axes[0], label="grasp x [m]", shrink=0.85)
    axes[0].set_xlabel(f"PC1 ({100 * var[0]:.0f}% var)")
    axes[0].set_ylabel(f"PC2 ({100 * var[1]:.0f}% var)")
    axes[0].set_title("Trajectory PCA (one point = one episode)")
    k = np.arange(1, 11)
    axes[1].bar(k, 100 * var[:10], color=BLUE, width=0.7)
    cum = 100 * np.cumsum(var[:10])
    axes[1].plot(k, cum, color=ORANGE, lw=2, marker="o", ms=4)
    axes[1].annotate(f"{cum[1]:.0f}% in 2 PCs", (2, cum[1]), textcoords="offset points",
                     xytext=(8, -12), color=INK2)
    axes[1].set_xlabel("component")
    axes[1].set_ylabel("% variance (bars) / cumulative (line)")
    axes[1].set_title("Explained variance -- manifold collapse check")
    fig.tight_layout()
    fig.savefig(out / "pca.png", dpi=150)
    plt.close(fig)
    return var


def fig_speed(eps, out: Path):
    fig, ax = plt.subplots(figsize=(8, 3.6))
    n = 100
    stack = []
    for e in eps:
        t = np.linspace(0, 1, len(e["speed"]))
        ti = np.linspace(0, 1, n)
        si = np.interp(ti, t, e["speed"])
        stack.append(si)
        ax.plot(ti, si, color=BLUE, alpha=0.08, lw=0.9)
    med = np.median(np.stack(stack), axis=0)
    ax.plot(np.linspace(0, 1, n), med, color=ORANGE, lw=2, label="median")
    ax.legend(frameon=False, loc="upper right")
    ax.set_xlabel("normalised episode time")
    ax.set_ylabel("tool speed [m/s]")
    ax.set_title("Speed profiles, all episodes")
    fig.tight_layout()
    fig.savefig(out / "speed_profiles.png", dpi=150)
    plt.close(fig)


def fig_distributions(eps, out: Path):
    metrics = [
        ("max_speed", "max tool speed [m/s]"),
        ("mean_speed", "mean tool speed [m/s]"),
        ("rms_jerk", "RMS tool jerk [m/s³]"),
        ("duration_s", "episode duration [s]"),
        ("peak_grip", "peak grip force [N]"),
        ("grasp_y", "grasp y position [m]"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(10.5, 5.6))
    for ax, (key, label) in zip(axes.ravel(), metrics):
        vals = np.array([e[key] for e in eps])
        ax.hist(vals, bins=24, color=BLUE, edgecolor=SURFACE, linewidth=0.6)
        med = np.median(vals)
        ax.axvline(med, color=ORANGE, lw=1.5)
        ax.annotate(f"med {med:.3g}", (med, ax.get_ylim()[1] * 0.92),
                    textcoords="offset points", xytext=(5, 0), color=INK2)
        ax.set_xlabel(label)
        ax.set_ylabel("episodes")
    fig.suptitle("Per-episode metric distributions", color=INK)
    fig.tight_layout()
    fig.savefig(out / "distributions.png", dpi=150)
    plt.close(fig)


def outlier_report(eps):
    print("\n--- outlier / cleaning check ---")
    flags = 0
    med_jerk = np.median([e["rms_jerk"] for e in eps])
    for e in eps:
        why = []
        if e["peak_grip"] < 5:
            why.append(f"peak grip only {e['peak_grip']:.1f} N")
        if e["rms_jerk"] > 6 * med_jerk:
            why.append(f"jerk {e['rms_jerk']:.2f} = {e['rms_jerk']/med_jerk:.0f}x median")
        if e["max_speed"] > 1.0:
            why.append(f"max speed {e['max_speed']:.2f} m/s")
        if why:
            flags += 1
            print(f"  ep {e['ep']:3d}: " + "; ".join(why))
    if flags == 0:
        print("  none flagged -- no cleaning needed beyond the success filter")
    return flags


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", default="data/demos_native")
    args = ap.parse_args()
    out = Path("figures/dataset_analysis")
    out.mkdir(parents=True, exist_ok=True)

    df = load_frames(args.root)
    scene = PickScene(seed=0, randomise=DomainRandomization.off())
    eps = per_episode(df, scene)
    print(f"{len(eps)} complete episodes, {len(df)} frames")

    fig_trajectories(eps, out)
    var = fig_pca(eps, out)
    fig_speed(eps, out)
    fig_distributions(eps, out)
    outlier_report(eps)

    print("\n--- summary ---")
    for key, unit in [("max_speed", "m/s"), ("mean_speed", "m/s"),
                      ("rms_jerk", "m/s^3"), ("duration_s", "s"),
                      ("peak_grip", "N")]:
        v = np.array([e[key] for e in eps])
        print(f"  {key:11s}: median {np.median(v):7.3f} {unit}  "
              f"IQR [{np.percentile(v, 25):.3f}, {np.percentile(v, 75):.3f}]")
    print(f"  PCA: {100 * var[0]:.0f}% in PC1, {100 * (var[0] + var[1]):.0f}% in 2 PCs, "
          f"{int(np.searchsorted(np.cumsum(var), 0.95) + 1)} PCs for 95%")
    print(f"\nfigures -> {out}/")


if __name__ == "__main__":
    main()
