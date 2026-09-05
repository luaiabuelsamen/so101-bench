"""Appendix D statistics: per-pose fits with uncertainty, and the figure.

Supersedes the descriptive pass in scripts/appendix_d_channels.py, which
reported point estimates only. Nothing in the appendix should quote a number
this script does not produce with an interval attached.

Design of the source measurement (blindspot/data/channel_j2.npz): a payload
staircase on one STS3215 (SO-101 shoulder-lift, joint id 2). Twelve hanging
masses spanning 63.7-319 g, each held at five arm poses, logging
Present_Load (addr 60), Present_Current (addr 69), Present_Position (addr 56)
and Goal_Position (addr 42) at static equilibrium. delta = Goal - Present.

Two facts about the design that bound every claim below:

  masses     known from package labelling, not weighed. Systematic error is
             unquantified; the replicate masses below bound the RANDOM part
             only.
  replicates the mass list contains near-duplicates (106.0/106.3, 148.8 twice,
             191.0 twice), which are independent re-measurements and are used
             here as the repeatability estimate.

    python scripts/appendix_d_stats.py [--fig figures/paper/fig_appd.png]
"""

import argparse
from pathlib import Path

import numpy as np

NPZ = "/home/jetson3/projects/research/blindspot/data/channel_j2.npz"
BOOT = 10000
RNG = np.random.default_rng(0)


def ols(x, y):
    A = np.vstack([x, np.ones_like(x)]).T
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ beta
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else np.nan
    n = len(x)
    se_slope = np.nan
    if n > 2 and ((x - x.mean()) ** 2).sum() > 0:
        se_slope = float(np.sqrt(ss_res / (n - 2) / ((x - x.mean()) ** 2).sum()))
    return float(beta[0]), float(beta[1]), r2, se_slope


def boot_r2(x, y, n_boot=BOOT):
    n = len(x)
    out = np.empty(n_boot)
    for b in range(n_boot):
        idx = RNG.integers(0, n, n)
        if len(np.unique(x[idx])) < 3:
            out[b] = np.nan
            continue
        out[b] = ols(x[idx], y[idx])[2]
    out = out[np.isfinite(out)]
    return float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", default=NPZ)
    ap.add_argument("--fig", default=None)
    args = ap.parse_args()

    d = np.load(args.npz, allow_pickle=True)
    o = np.argsort(d["mass"])
    mass = d["mass"][o]
    chan = {
        "delta": (d["goal"] - d["position"])[o],
        "Present_Load": d["load"][o],
        "Present_Current": d["current"][o],
    }
    goal = d["goal"][o]
    n_pose = mass.shape[0] and chan["delta"].shape[1]

    print(f"joint {int(d['joint'])}; {len(mass)} masses {mass.min():.1f}-{mass.max():.1f} g; "
          f"{n_pose} poses\n")

    # --- repeatability from replicate masses -------------------------------
    print("Repeatability from replicate masses (independent re-measurements):")
    rounded = np.round(mass, 0)
    for name, y in chan.items():
        spreads = []
        for v in np.unique(rounded):
            k = np.flatnonzero(rounded == v)
            if len(k) > 1:
                spreads.append(np.abs(y[k[0]] - y[k[1]]))
        if spreads:
            s = np.concatenate(spreads)
            print(f"  {name:<16} mean |difference| between replicates = {s.mean():.2f}")
    print()

    # --- per-pose fits ------------------------------------------------------
    table = {}
    for name, y in chan.items():
        rows = []
        for c in range(n_pose):
            slope, icpt, r2, se = ols(mass, y[:, c])
            lo, hi = boot_r2(mass, y[:, c])
            rows.append(dict(pose=c, goal=float(goal[:, c].mean()), slope=slope,
                             se=se, r2=r2, lo=lo, hi=hi))
        table[name] = rows
        print(f"{name} vs true mass, per pose (95% bootstrap CI on R^2, {BOOT} resamples):")
        for r in rows:
            print(f"   pose {r['pose']} (goal {r['goal']:.0f}): "
                  f"slope {r['slope']:+.4f} +/- {r['se']:.4f} /g   "
                  f"R2 {r['r2']:.3f} [{r['lo']:.3f}, {r['hi']:.3f}]")
        med = np.median([r["r2"] for r in rows])
        print(f"   median R2 {med:.3f}\n")

    # --- delta vs load, pose by pose ---------------------------------------
    dr = [r["r2"] for r in table["delta"]]
    lr = [r["r2"] for r in table["Present_Load"]]
    cr = [r["r2"] for r in table["Present_Current"]]
    same = sum(round(a, 3) == round(b, 3) for a, b in zip(dr, lr))
    print(f"delta vs Present_Load, per-pose R^2 agreement:")
    print(f"   identical to 3 decimals in {same}/{n_pose} poses; "
          f"max |difference| {max(abs(a-b) for a, b in zip(dr, lr)):.3f}")
    print(f"   (that difference is the honest statement, not 'identical')\n")

    # Paired sign test across poses: does delta beat current?
    # NOTE (2026-09-04): this previously printed 2**-n_pose * 2 -- the p-value
    # for a CLEAN SWEEP -- on the same line as the observed win count, with the
    # conditional buried in the string as "if all". At 4/5 that reads as
    # p=0.06 when the exact two-sided value is 0.375. main.tex carried the
    # wrong number. Compute the p-value for the count actually observed.
    from math import comb
    wins = sum(a > b for a, b in zip(dr, cr))
    k = max(wins, n_pose - wins)
    p_one = sum(comb(n_pose, i) for i in range(k, n_pose + 1)) / 2 ** n_pose
    p_two = min(1.0, 2 * p_one)
    print(f"delta beats Present_Current in {wins}/{n_pose} poses "
          f"(exact paired sign test on the OBSERVED count: "
          f"p = {p_two:.3f} two-sided, {p_one:.3f} one-sided)")
    print(f"   for reference, a clean {n_pose}/{n_pose} sweep would give "
          f"p = {2 ** -n_pose * 2:.3f} two-sided -- the smallest value "
          f"reachable at n={n_pose}\n")

    # --- load as an affine function of delta --------------------------------
    x = chan["delta"].ravel()
    y = chan["Present_Load"].ravel()
    slope, icpt, r2, se = ols(x, y)
    lo, hi = boot_r2(x, y, 2000)
    print(f"Present_Load = {slope:.2f} (+/- {se:.2f}) * delta {icpt:+.1f}   "
          f"R2 {r2:.3f} [{lo:.3f}, {hi:.3f}]  n={len(x)}")
    print("   NOTE: pooled over poses, so its residual contains lever-arm variation;")
    print("   it measures that the two are one signal, not the accuracy of the map.")

    if args.fig:
        make_figure(mass, chan, goal, Path(args.fig))


def make_figure(mass, chan, goal, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.2))
    colors = plt.cm.viridis(np.linspace(0.1, 0.85, chan["delta"].shape[1]))

    for ax, (name, y) in zip(axes[:2], list(chan.items())[:2]):
        for c in range(y.shape[1]):
            ax.plot(mass, y[:, c], "o-", ms=3, lw=1, color=colors[c],
                    label=f"pose {c}")
        ax.set_xlabel("true hanging mass (g)")
        ax.set_ylabel({"delta": r"$\delta$ (counts)"}.get(name, name.replace("_", " ")))
        ax.set_title(name.replace("_", r"\_") if False else name, fontsize=9)
    axes[0].legend(fontsize=6, frameon=False)

    ax = axes[2]
    ax.scatter(chan["delta"].ravel(), chan["Present_Load"].ravel(), s=8,
               c="#1b6ca8", alpha=0.6)
    xs = np.linspace(chan["delta"].min(), chan["delta"].max(), 50)
    slope, icpt, r2, _ = ols(chan["delta"].ravel(), chan["Present_Load"].ravel())
    ax.plot(xs, slope * xs + icpt, color="#d95f02", lw=1.4)
    ax.set_xlabel(r"$\delta$ (counts)")
    ax.set_ylabel("Present_Load")
    ax.set_title(f"load = {slope:.2f}$\\delta$ {icpt:+.1f},  $R^2$={r2:.3f}", fontsize=9)

    for a in axes:
        a.annotate("hardware", xy=(0.99, 0.02), xycoords="axes fraction",
                   ha="right", va="bottom", fontsize=7, color="gray")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
