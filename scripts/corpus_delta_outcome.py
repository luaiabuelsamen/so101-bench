"""Phase 3 Task 1: delta at seat vs episode outcome across the public corpus.

    python scripts/corpus_delta_outcome.py                 # local datasets
    python scripts/corpus_delta_outcome.py --fetch 40      # top up to 20 qualifiers

ONE SCRIPT, ONE FIGURE, THEN STOP (docs/phase3.md). All operational
definitions are frozen in docs/phase3.md and were written before the run:

- close events: hysteresis on the gripper ACTION trace (close < 30% of
  range sustained 5 frames, reopen > 60% sustained 5 frames); datasets whose
  MEDIAN episode closes more than once are multi-grasp by design and excluded;
- seat frame T: offline stall -- first frame after close onset where the
  measured jaw moves < 1 count for 4 consecutive frames; seated flag =
  command-measured gap >= 2 counts at T;
- delta at seat: mean of (a[t-1] - s[t]) on the gripper dim over [T, T+8],
  encoder counts (quantum per repo from the 555-dataset study, empirical
  fallback);
- success proxy (frozen, not iterated): no reopen between T and the final
  10% of the episode AND transport (max_t>T ||s[t,:5]-s[T,:5]||_2 >= 60
  counts);
- PRE-REGISTERED: successes at HIGHER delta; Cohen's d > 0.5 in >= 10/20
  datasets; a negative is written at full strength and Phase 3 continues.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

MIN_EPISODES = 20
MIN_CLASS = 5
CLOSE_SUSTAIN = 5
STAB_FRAMES = 4            # measured-jaw stillness run defining T
STAB_COUNT = 1.0           # counts of motion still treated as still
SEAT_GAP = 2.0             # command-measured gap at T that means "seated"
DELTA_WIN = 8              # frames of delta averaged from T
TRANSPORT_COUNTS = 60.0    # arm displacement (5 joint dims, L2) for "moves"
REOPEN_TAIL = 0.10         # reopen inside the final 10% doesn't count


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


def load_dataset(root: str, q_known: float | None = None, max_files: int = 40):
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
    # convert whatever normalisation the dataset uses to encoder counts via
    # the channel quantum; gate on implied joint range, not assumed units
    q = q_known or empirical_quantum(S)
    if q is None or q <= 0:
        return None
    rng_counts = float(np.median((S.max(0) - S.min(0)) / q))
    if not (100.0 <= rng_counts <= 8000.0):
        return None
    return S / q, A / q, df["episode_index"].to_numpy()


def close_events(grip_a: np.ndarray):
    """Hysteresis open->close events in the gripper ACTION trace (frozen)."""
    lo, hi = np.percentile(grip_a, 1), np.percentile(grip_a, 99)
    if hi - lo < 4:                              # gripper never really moves
        return None, None, None
    t_close = lo + 0.30 * (hi - lo)              # SO-101: smaller = more closed
    t_open = lo + 0.60 * (hi - lo)
    events, reopens = [], []
    state = "open" if grip_a[0] >= t_close else "closed"
    cand_start, cand_n = None, 0
    for i, g in enumerate(grip_a):
        if state == "open":
            if g <= t_close:
                if cand_start is None:
                    cand_start = i
                cand_n += 1
                if cand_n >= CLOSE_SUSTAIN:
                    state, cand_start, cand_n = "closed", None, 0
                    events.append(i - CLOSE_SUSTAIN + 1)
            else:
                cand_start, cand_n = None, 0
        else:
            if g >= t_open:
                cand_n += 1
                if cand_n >= CLOSE_SUSTAIN:
                    state, cand_n = "open", 0
                    reopens.append(i - CLOSE_SUSTAIN + 1)
            else:
                cand_n = 0
    return events, reopens, t_close


def episode_row(s: np.ndarray, a: np.ndarray):
    """One frozen measurement per episode; None if no close event."""
    ev, reopens, _ = close_events(a[:, 5])
    if ev is None or len(ev) == 0:
        return None
    c = ev[0]
    n = len(s)
    # stabilization frame T: measured jaw still for STAB_FRAMES -- but only
    # after closing has begun (measured moved >= 3 counts from close onset,
    # or command already >= SEAT_GAP beyond measured), else the pre-close
    # stillness of an open jaw satisfies the test at c+1
    T = None
    moving_seen = False
    for t in range(c + 1, min(c + 120, n - DELTA_WIN - 1)):
        if not moving_seen:
            if abs(s[t, 5] - s[c, 5]) >= 3.0 or s[t, 5] - a[t, 5] >= SEAT_GAP:
                moving_seen = True
            else:
                continue
        w = s[max(t - STAB_FRAMES, 0): t + 1, 5]
        if len(w) > STAB_FRAMES and np.abs(np.diff(w)).max() < STAB_COUNT:
            T = t
            break
    if T is None:
        return None
    j1 = min(T + DELTA_WIN, n - 1)
    # magnitude: seated grasps hold the jaw ABOVE the (deeper) command, a
    # clean miss tracks the command to ~0 gap; unsigned tracking error is
    # the force reading
    delta = float(np.mean(np.abs(a[T - 1: j1 - 1, 5] - s[T: j1, 5])))
    seated = bool(s[T, 5] - a[T, 5] >= SEAT_GAP)   # jaw held above command
    reopen_early = any(T < r < int(n * (1.0 - REOPEN_TAIL)) for r in reopens)
    transport = float(np.max(np.linalg.norm(s[T:, :5] - s[T, :5], axis=1)))
    success = (not reopen_early) and transport >= TRANSPORT_COUNTS
    return dict(delta=delta, seated=seated, success=bool(success),
                n_closes=int(len(ev)), transport=transport)


def cohen_d(x: np.ndarray, y: np.ndarray) -> float:
    nx, ny = len(x), len(y)
    sp = np.sqrt(((nx - 1) * x.std(ddof=1) ** 2 + (ny - 1) * y.std(ddof=1) ** 2)
                 / max(nx + ny - 2, 1))
    return float((x.mean() - y.mean()) / (sp + 1e-9))


def analyse_dataset(root: str, q_known: float | None = None):
    got = load_dataset(root, q_known)
    if got is None:
        return None
    S, A, ep = got
    rows = []
    for e in np.unique(ep):
        m = ep == e
        if m.sum() < 30:
            continue
        r = episode_row(S[m], A[m])
        if r is not None:
            rows.append(r)
    if len(rows) < MIN_EPISODES:
        return None
    if np.median([r["n_closes"] for r in rows]) > 1:
        return dict(name=os.path.basename(root), episodes=len(rows),
                    multigrasp_task=True)
    d_all = np.array([r["delta"] for r in rows])
    y = np.array([r["success"] for r in rows])
    base = dict(name=os.path.basename(root), episodes=len(rows),
                success_rate=float(y.mean()),
                seated_rate=float(np.mean([r["seated"] for r in rows])))
    if y.sum() < MIN_CLASS or (~y).sum() < MIN_CLASS:
        return base
    d = cohen_d(d_all[y], d_all[~y])
    rng = np.random.default_rng(0)
    boots = []
    for _ in range(2000):
        i = rng.integers(0, len(y), len(y))
        if 1 < y[i].sum() < len(y) - 1:
            boots.append(cohen_d(d_all[i][y[i]], d_all[i][~y[i]]))
    lo, hi = np.percentile(boots, [2.5, 97.5]) if boots else (np.nan, np.nan)
    return dict(**base, cohen_d=float(d), d_ci=[float(lo), float(hi)],
                delta_succ_median=float(np.median(d_all[y])),
                delta_fail_median=float(np.median(d_all[~y])),
                delta_z=list((d_all - d_all.mean()) / (d_all.std() + 1e-9)),
                success=list(map(bool, y)))


def fetch_more(raw_root: str, quanta: dict[str, float], have: set[str],
               want: int, frames: dict[str, int] | None = None,
               free_gb_floor: float = 3.0):
    """Download parquet+meta only for candidate repos, biggest first.

    Candidates are ordered by recorded frame count (statistical power),
    chosen before any outcome is seen -- the MIN_EPISODES/MIN_CLASS gates
    eat small datasets, so alphabetical order wastes downloads.
    """
    from huggingface_hub import snapshot_download
    frames = frames or {}
    cands = sorted(quanta.keys() - have,
                   key=lambda n: -frames.get(n, 0))
    got = 0
    for name in cands:
        if got >= want:
            break
        if shutil.disk_usage(raw_root).free / 1e9 < free_gb_floor:
            print("disk floor reached, stopping fetch")
            break
        repo = name.replace("__", "/", 1)
        tgt = os.path.join(raw_root, name)
        try:
            snapshot_download(repo, repo_type="dataset", local_dir=tgt,
                              allow_patterns=["data/**", "meta/**"])
            got += 1
            print(f"fetched {repo}")
        except Exception as e:
            print(f"fetch failed {repo}: {type(e).__name__}")
            shutil.rmtree(tgt, ignore_errors=True)
    return got


def figure(rows, out_png: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = sorted(rows, key=lambda r: r["cohen_d"])
    fig, ax = plt.subplots(figsize=(8, 0.42 * len(rows) + 1.8))
    XLIM = 4.0                        # display clip; stereotyped-success
    for k, r in enumerate(rows):      # datasets have |d| in the tens
        lo, hi = r["d_ci"]
        d = r["cohen_d"]
        if d < -XLIM:
            ax.scatter([-XLIM * 0.97], [k], s=45, marker="<",
                       color="#2b7bba", zorder=3)
            ax.annotate(f"d={d:.1f}", (-XLIM * 0.94, k), fontsize=7,
                        va="center")
        else:
            ax.plot([max(lo, -XLIM), min(hi, XLIM)], [k, k],
                    color="#2b7bba", lw=1.5)
            ax.scatter([d], [k], s=18 + r["episodes"] * 0.6,
                       color="#2b7bba", zorder=3)
    ax.axvline(0, color="gray", lw=0.8)
    ax.axvline(0.5, color="green", ls="--", lw=0.8,
               label="pre-registered d = 0.5")
    ax.set_xlim(-XLIM, XLIM)
    ax.set_yticks(range(len(rows)),
                  [r["name"][:30] for r in rows], fontsize=7)
    ax.set_xlabel("Cohen's d: delta-at-seat, success vs failure (bootstrap 95% CI)")
    hits = sum(r["cohen_d"] > 0.5 for r in rows)
    ax.set_title(f"delta at seat vs outcome: pre-registered d>0.5 in "
                 f"{hits}/{len(rows)} datasets", fontsize=10)
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=160)
    print(f"wrote {out_png}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", default=os.path.expanduser(
        "~/projects/research/proprio-residual/data/raw"))
    ap.add_argument("--quanta-jsonl", default=os.path.expanduser(
        "~/projects/research/corpus_scale/results.jsonl"),
        help="per-repo quantum from the prior 555-dataset resolution study")
    ap.add_argument("--fetch", type=int, default=0,
                    help="download up to N additional candidate datasets")
    ap.add_argument("--limit", type=int, default=400)
    ap.add_argument("--json", default="results/corpus_delta_outcome.json")
    ap.add_argument("--fig", default="figures/paper/fig1_corpus.png")
    args = ap.parse_args()

    quanta: dict[str, float] = {}
    frames: dict[str, int] = {}
    if os.path.exists(args.quanta_jsonl):
        for line in open(args.quanta_jsonl):
            try:
                r = json.loads(line)
            except Exception:
                continue
            if r.get("status") == "success" and r.get("quantum"):
                name = r["repo_id"].replace("/", "__")
                quanta[name] = float(r["quantum"])
                frames[name] = int(r.get("n_frames") or 0)

    root = os.path.expanduser(args.root)
    if args.fetch:
        have = {os.path.basename(p) for p in glob.glob(os.path.join(root, "*"))}
        fetch_more(root, quanta, have, args.fetch, frames)

    roots = sorted(glob.glob(os.path.join(root, "*")))[: args.limit]
    out, scored = [], []
    pooled_z, pooled_y = [], []
    print(f"{'dataset':38s} {'eps':>5} {'succ%':>6} {'seat%':>6} "
          f"{'d':>6} {'95% CI':>14}")
    for r in roots:
        try:
            res = analyse_dataset(r, quanta.get(os.path.basename(r)))
        except Exception:
            continue
        if res is None:
            continue
        if res.get("cohen_d") is not None:
            scored.append(res)
            pooled_z += res["delta_z"]
            pooled_y += res["success"]
            print(f"{res['name'][:38]:38s} {res['episodes']:5d} "
                  f"{100 * res['success_rate']:5.0f}% "
                  f"{100 * res['seated_rate']:5.0f}% {res['cohen_d']:6.2f} "
                  f"[{res['d_ci'][0]:5.2f},{res['d_ci'][1]:5.2f}]")
        out.append({k: v for k, v in res.items()
                    if k not in ("delta_z", "success")})

    if scored:
        z = np.array(pooled_z)
        yy = np.array(pooled_y)
        pooled_d = cohen_d(z[yy], z[~yy])
        hits = sum(r["cohen_d"] > 0.5 for r in scored)
        print(f"\n{len(scored)} scored datasets; pre-registered d>0.5 in "
              f"{hits}/{len(scored)}; pooled z-scored d = {pooled_d:.2f} "
              f"({len(z)} episodes, {int(yy.sum())} successes)")
    Path(args.json).parent.mkdir(exist_ok=True)
    with open(args.json, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"wrote {args.json}")
    if len(scored) >= 3:
        figure(scored, Path(args.fig))


if __name__ == "__main__":
    main()
