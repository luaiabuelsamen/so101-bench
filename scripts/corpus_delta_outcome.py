"""Does grasp-time delta predict grasp failure across the public corpus?

    python scripts/corpus_delta_outcome.py --root ~/projects/research/proprio-residual/data/raw

THE QUESTION THAT DECIDES THE DIRECTION. If the tracking-error channel at
grasp time carries outcome-relevant information in the wild -- across datasets
nobody instrumented for force -- then the observability thesis applies to the
entire ecosystem retroactively. If the plot is boring, the big version dies
this week, cheaply, as it should.

LABELS WITHOUT LABELS. Public teleop datasets record successes only, so
episode-level success labels do not exist. The label-free outcome proxy:
**regrasps**. A first grasp that fails produces an open->close->open->close
signature in the gripper action trace; a clean grasp closes once and reopens
only at the final release. "First-grasp failure" = two or more sustained
open->close events in one episode. This is conservative (an operator may
regrasp for other reasons) and is measured identically everywhere.

MEASUREMENT. Per episode: find the first sustained gripper close in the
ACTION trace; delta_grip = mean |a[t-1] - s[t]| on the gripper dim (converted
to encoder counts via each dataset's empirically measured quantum, since
normalisation conventions differ) over a short window after the close
completes. Per dataset: ROC-AUC (Mann-Whitney) of delta_grip -> regrasp,
bootstrap CI, kept only when both classes have >= 5 episodes. Pooled: within-
dataset z-scored delta, pooled AUC. Cross-reference: each dataset's channel
resolution (distinct delta levels, from the prior 555-dataset study) against
its AUC -- prediction: low-resolution datasets cannot show coupling.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np
import pandas as pd

MIN_EPISODES = 20
MIN_CLASS = 5
CLOSE_SUSTAIN = 5
WINDOW = (2, 10)          # frames after close-completion for delta_grip


def empirical_quantum(S: np.ndarray) -> float | None:
    qs = []
    for j in range(S.shape[1]):
        u = np.unique(np.round(S[:, j], 9))
        if len(u) < 3:
            continue
        g = np.diff(u)
        g = g[g > 1e-9]
        if len(g):
            qs.append(float(np.percentile(g, 5)))
    return float(np.median(qs)) if qs else None


def load_dataset(root: str, max_files: int = 40):
    files = sorted(glob.glob(os.path.join(root, "data", "**", "*.parquet"),
                             recursive=True))[:max_files]
    parts = []
    for f in files:
        try:
            parts.append(pd.read_parquet(
                f, columns=["observation.state", "action", "episode_index"]))
        except Exception:
            pass
    if not parts:
        return None
    df = pd.concat(parts, ignore_index=True)
    S = np.stack(df["observation.state"].to_numpy()).astype(float)
    A = np.stack(df["action"].to_numpy()).astype(float)
    if S.ndim != 2 or S.shape[1] != 6 or A.shape[1] != 6:
        return None                              # single-arm SO-100/101 only
    q = empirical_quantum(S)
    if q is None or not (500.0 <= 200.0 / q <= 5000.0):
        return None
    return S / q, A / q, df["episode_index"].to_numpy()


def close_events(grip_a: np.ndarray):
    """Sustained open->close transitions in the gripper ACTION trace."""
    lo, hi = np.percentile(grip_a, 1), np.percentile(grip_a, 99)
    if hi - lo < 4:                              # gripper never really moves
        return None, None
    mid = (lo + hi) / 2.0
    closed = grip_a < mid                        # SO-101: smaller = more closed
    # sustained-state filter
    events = []
    i, n = 0, len(closed)
    state = closed[0]
    run_start = 0
    runs = []
    for i in range(1, n + 1):
        if i == n or closed[i] != state:
            runs.append((state, run_start, i))
            if i < n:
                state = closed[i]
                run_start = i
    runs = [(s, a, b) for s, a, b in runs if b - a >= CLOSE_SUSTAIN]
    for k in range(1, len(runs)):
        if runs[k][0] and not runs[k - 1][0]:
            events.append(runs[k][1])            # frame where a close begins
    return events, mid


def analyse_dataset(root: str):
    got = load_dataset(root)
    if got is None:
        return None
    S, A, ep = got
    rows = []
    for e in np.unique(ep):
        m = ep == e
        s, a = S[m], A[m]
        if len(s) < 30:
            continue
        ev, mid = close_events(a[:, 5])
        if ev is None or len(ev) == 0:
            continue
        t0 = ev[0]
        j0, j1 = t0 + WINDOW[0], min(t0 + WINDOW[1], len(s) - 1)
        if j1 <= j0 + 2:
            continue
        delta = np.abs(a[j0 - 1:j1 - 1, 5] - s[j0:j1, 5])
        rows.append(dict(episode=int(e), delta_grip=float(delta.mean()),
                         regrasp=bool(len(ev) >= 2), n_closes=int(len(ev))))
    if len(rows) < MIN_EPISODES:
        return None
    d = np.array([r["delta_grip"] for r in rows])
    y = np.array([r["regrasp"] for r in rows])
    if y.sum() < MIN_CLASS or (~y).sum() < MIN_CLASS:
        return dict(name=os.path.basename(root), episodes=len(rows),
                    regrasp_rate=float(y.mean()), auc=None)
    # Mann-Whitney AUC
    order = np.argsort(np.argsort(d))
    auc = (order[y].sum() - y.sum() * (y.sum() - 1) / 2) / (y.sum() * (~y).sum())
    boots = []
    rng = np.random.default_rng(0)
    for _ in range(1000):
        idx = rng.integers(0, len(d), len(d))
        yy, dd = y[idx], d[idx]
        if yy.sum() < 2 or (~yy).sum() < 2:
            continue
        o = np.argsort(np.argsort(dd))
        boots.append((o[yy].sum() - yy.sum() * (yy.sum() - 1) / 2)
                     / (yy.sum() * (~yy).sum()))
    lo, hi = (np.percentile(boots, 2.5), np.percentile(boots, 97.5)) if boots else (None, None)
    return dict(name=os.path.basename(root), episodes=len(rows),
                regrasp_rate=float(y.mean()), auc=float(auc),
                auc_ci=[float(lo), float(hi)],
                delta_z=list((d - d.mean()) / (d.std() + 1e-9)),
                regrasp=list(map(bool, y)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", default=os.path.expanduser(
        "~/projects/research/proprio-residual/data/raw"))
    ap.add_argument("--limit", type=int, default=200)
    ap.add_argument("--json", default="results/corpus_delta_outcome.json")
    args = ap.parse_args()

    roots = sorted(glob.glob(os.path.join(os.path.expanduser(args.root), "*")))[: args.limit]
    out, pooled_d, pooled_y = [], [], []
    print(f"{'dataset':38s} {'eps':>5} {'regrasp%':>9} {'AUC':>6} {'95% CI':>14}")
    for r in roots:
        try:
            res = analyse_dataset(r)
        except Exception:
            continue
        if res is None:
            continue
        if res.get("auc") is not None:
            pooled_d += res["delta_z"]
            pooled_y += res["regrasp"]
            print(f"{res['name'][:38]:38s} {res['episodes']:5d} "
                  f"{100 * res['regrasp_rate']:8.0f}% {res['auc']:6.2f} "
                  f"[{res['auc_ci'][0]:.2f},{res['auc_ci'][1]:.2f}]")
        out.append({k: v for k, v in res.items() if k not in ("delta_z", "regrasp")})

    if pooled_d:
        d = np.array(pooled_d)
        y = np.array(pooled_y)
        o = np.argsort(np.argsort(d))
        auc = (o[y].sum() - y.sum() * (y.sum() - 1) / 2) / (y.sum() * (~y).sum())
        aucs = [r["auc"] for r in out if r.get("auc") is not None]
        print(f"\npooled (within-dataset z-scored): AUC {auc:.3f} over "
              f"{len(d)} episodes, {int(y.sum())} regrasps, "
              f"{len(aucs)} datasets with both classes")
        print(f"per-dataset AUC: median {np.median(aucs):.2f}, "
              f"{sum(a > 0.5 for a in aucs)}/{len(aucs)} above chance")
    Path(args.json).parent.mkdir(exist_ok=True)
    with open(args.json, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
