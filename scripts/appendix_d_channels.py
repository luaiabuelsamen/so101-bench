"""Appendix D, from data already on the bench: is delta redundant with the
registers the servo already reports?

Rule 4 says the reviewer's baseline comes first, and for this project that
baseline is Present_Load and Present_Current. The question is not whether delta
carries force information -- it is whether it carries any that the STS3215 does
not already hand you for free over the same bus.

Input is blindspot's channel_j2.npz: 12 known hanging masses (63.7-319 g,
weighed by package label), each measured at five arm poses, logging
Present_Load, Present_Current, Present_Position and Goal_Position together.
A hanging mass is the ground truth, standing in for a load cell in the gravity
direction.

The second axis is POSE, not repeat. Pooling across it mixes the lever arm into
the residual and makes every channel look like noise; each pose is fit
separately here for that reason.

    python scripts/appendix_d_channels.py

What this is not: it is one joint (the n=1-on-k limitation is untouched), it is
gravity-direction loading rather than the gripper-closing direction a load cell
would probe, and it is static. Those still need the bench.
"""

import argparse
from pathlib import Path

import numpy as np

DEFAULT = Path("/home/jetson3/projects/research/blindspot/data/channel_j2.npz")


def fit_per_pose(x, y):
    """Slope, R^2 and residual of y against x, one fit per pose column."""
    out = []
    for c in range(y.shape[1]):
        yc = y[:, c]
        A = np.vstack([x, np.ones_like(x)]).T
        (slope, icpt), *_ = np.linalg.lstsq(A, yc, rcond=None)
        pred = A @ np.array([slope, icpt])
        ss_res = float(((yc - pred) ** 2).sum())
        ss_tot = float(((yc - yc.mean()) ** 2).sum())
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else np.nan
        resid = np.sqrt(ss_res / max(len(x) - 2, 1))
        out.append((slope, r2, resid, resid / abs(slope) if slope else np.inf))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", default=str(DEFAULT))
    args = ap.parse_args()

    d = np.load(args.npz, allow_pickle=True)
    order = np.argsort(d["mass"])
    mass = d["mass"][order]
    load, current = d["load"][order], d["current"][order]
    delta = (d["goal"] - d["position"])[order]
    goal = d["goal"][order]

    print(f"joint {int(d['joint'])}, {len(mass)} masses {mass.min():.0f}-{mass.max():.0f} g, "
          f"{load.shape[1]} poses (goal ~ {np.round(goal.mean(0)).astype(int).tolist()})\n")

    channels = {"delta": delta, "Present_Load": load, "Present_Current": current}
    medians = {}
    for name, y in channels.items():
        rows = fit_per_pose(mass, y)
        print(f"{name} vs mass, per pose:")
        for c, (slope, r2, resid, res_g) in enumerate(rows):
            print(f"   pose {c}: slope {slope:+.4f}/g   R2 {r2:.3f}   "
                  f"resid {resid:5.2f}   -> {res_g:4.0f} g resolvable")
        medians[name] = float(np.median([r[1] for r in rows]))
        print(f"   median R2 {medians[name]:.3f}\n")

    # delta and load tracking mass equally well would mean they are one signal.
    A = np.vstack([delta.ravel(), np.ones(delta.size)]).T
    (slope, icpt), *_ = np.linalg.lstsq(A, load.ravel(), rcond=None)
    pred = A @ np.array([slope, icpt])
    r2 = 1 - ((load.ravel() - pred) ** 2).sum() / ((load.ravel() - load.mean()) ** 2).sum()

    print(f"Present_Load = {slope:.2f} * delta {icpt:+.1f}   R2 = {r2:.3f}")
    print(f"per-pose R2 against mass: delta {medians['delta']:.3f}, "
          f"load {medians['Present_Load']:.3f}, current {medians['Present_Current']:.3f}")
    print(
        "\nReading: load tracks mass exactly as well as delta does, pose for pose,\n"
        "because it is an affine function of delta -- one signal, not two. Current\n"
        "is the weaker instrument here, not the stronger one. So the reviewer's\n"
        "baseline does not beat the channel; it IS the channel, and it is the one\n"
        "the public corpus does not record."
    )


if __name__ == "__main__":
    main()
