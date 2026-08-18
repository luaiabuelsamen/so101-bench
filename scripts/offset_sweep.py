"""Roadmap C.1: command-to-state offset sweep over the downloaded corpus.

    python scripts/offset_sweep.py

For each scored corpus dataset, sweep the alignment k in
delta_k[t] = a[t-k] - s[t], k in 0..5, and report:

  (a) per-dataset latency estimate k* = argmin over k of the median |delta_k|
      on free-motion (jaw-open) frames -- at the true alignment the command
      and the position agree when nothing resists;
  (b) sensitivity of the Sec 4.3 result: Cohen's d of delta_k-at-seat vs the
      frozen success proxy, recomputed at every k -- the range bounds how
      much the corpus numbers depend on the k=1 convention the protocol
      froze.

CPU only; no GPU, no downloads (uses the datasets already on disk).
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from corpus_delta_outcome import (  # noqa: E402
    close_events, cohen_d, load_dataset, DELTA_WIN, SEAT_GAP, STAB_COUNT,
    STAB_FRAMES, REOPEN_TAIL, TRANSPORT_COUNTS,
)

RAW = os.path.expanduser("~/projects/research/proprio-residual/data/raw")
KS = range(0, 6)


def episode_row_k(s: np.ndarray, a: np.ndarray, k: int):
    """The frozen Sec-4.3 measurement with the alignment offset swept."""
    ev, reopens, _ = close_events(a[:, 5])
    if ev is None or len(ev) == 0:
        return None
    c = ev[0]
    n = len(s)
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
    if T is None or T - k < 0:
        return None
    j1 = min(T + DELTA_WIN, n - 1)
    if j1 - k < 0 or T - k < 0:
        return None
    delta = float(np.mean(np.abs(a[T - k: j1 - k, 5] - s[T: j1, 5])))
    reopen_early = any(T < r < int(n * (1.0 - REOPEN_TAIL)) for r in reopens)
    transport = float(np.max(np.linalg.norm(s[T:, :5] - s[T, :5], axis=1)))
    success = (not reopen_early) and transport >= TRANSPORT_COUNTS
    return delta, bool(success)


def main():
    scored = [r["name"] for r in
              json.load(open("results/corpus_delta_outcome.json"))
              if r.get("cohen_d") is not None]
    out = []
    print(f"{'dataset':38s} {'k*':>3} {'free|δ| by k':>28}  d by k")
    for name in scored:
        got = load_dataset(os.path.join(RAW, name))
        if got is None:
            continue
        S, A, ep = got
        # (a) free-motion |delta_k|: jaw-open frames, per k
        open_thr = np.quantile(A[:, 5], 0.60)
        free = A[:, 5] > open_thr
        free_med = []
        for k in KS:
            if k == 0:
                d = np.abs(A[:, 5] - S[:, 5])[free]
            else:
                m = free[k:]
                d = np.abs(A[:-k, 5] - S[k:, 5])[m]
            free_med.append(float(np.median(d)))
        k_star = int(np.argmin(free_med))
        # (b) Cohen's d of delta_k-at-seat per k
        ds = []
        for k in KS:
            deltas, ys = [], []
            for e in np.unique(ep):
                m = ep == e
                if m.sum() < 30:
                    continue
                r = episode_row_k(S[m], A[m], k)
                if r is not None:
                    deltas.append(r[0])
                    ys.append(r[1])
            deltas, ys = np.array(deltas), np.array(ys, bool)
            if len(ys) >= 20 and 5 <= ys.sum() <= len(ys) - 5:
                sp = np.sqrt(((ys.sum() - 1) * deltas[ys].std(ddof=1) ** 2
                              + ((~ys).sum() - 1) * deltas[~ys].std(ddof=1) ** 2)
                             / max(len(ys) - 2, 1))
                ds.append(float((deltas[ys].mean() - deltas[~ys].mean())
                                / (sp + 1e-9)))
            else:
                ds.append(None)
        fm = " ".join(f"{v:4.1f}" for v in free_med)
        dd = " ".join("  -- " if d is None else f"{d:+5.2f}" for d in ds)
        print(f"{name[:38]:38s} {k_star:3d} [{fm}]  [{dd}]")
        out.append(dict(name=name, k_star=k_star, free_median_by_k=free_med,
                        cohen_d_by_k=ds))

    ks = [r["k_star"] for r in out]
    print(f"\nk* distribution: {dict((k, ks.count(k)) for k in sorted(set(ks)))}")
    spans = [max(x for x in r["cohen_d_by_k"] if x is not None)
             - min(x for x in r["cohen_d_by_k"] if x is not None)
             for r in out if any(x is not None for x in r["cohen_d_by_k"])]
    print(f"d sensitivity to k: median span {np.median(spans):.2f}, "
          f"max {max(spans):.2f} across {len(spans)} datasets")
    with open("results/offset_sweep.json", "w") as fh:
        json.dump(out, fh, indent=2, default=float)
    print("wrote results/offset_sweep.json")


if __name__ == "__main__":
    main()
