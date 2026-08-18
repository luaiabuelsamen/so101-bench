"""Roadmap E.1 / review optional: the free-motion compensator ladder, offline.

    python scripts/compensator_ladder.py

Question: is the paper's velocity-only lag model (delta ~ K*r) leaving
compensation on the table? Fit increasingly rich free-motion predictors of
delta on the TRAIN split of contact-free (jaw-open) frames of demos_v3 and
score held-out RMSE on the TEST split, per joint:

  M0  zero predictor (raw delta baseline: no compensation)
  M1  K*r                      (the paper's model: scalar through origin)
  M2  K*r + b                  (adds an intercept)
  M3  linear in [r, |r|, sign(r), dr]   (direction + accel terms)
  M4  FIR over r[t-1..t-4]     (short causal filter)

Split is by EPISODE (first 80% train, last 20% test) so temporal leakage
cannot flatter the richer models. If M1 ~= M3/M4, minimalism is validated;
if the richer model wins clearly, arm E' is worth training (roadmap E).
CPU only.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from train_act import FastChunkDataset, load_dataset  # noqa: E402


def fit_eval(Xtr, ytr, Xte, yte):
    w, *_ = np.linalg.lstsq(Xtr, ytr, rcond=None)
    return float(np.sqrt(np.mean((yte - Xte @ w) ** 2)))


def main():
    ds = load_dataset("data/demos_v3")
    f = FastChunkDataset(ds, "excess")          # builds A_prev, rate, k_hat
    ep = np.cumsum(f.first)                     # episode id per frame
    n_ep = ep.max()
    tr = ep <= int(n_ep * 0.8)
    te = ~tr

    delta = np.where(~f.first[:, None], f.A_prev - f.S, 0.0)
    r = f.rate
    # r[t-2..t-4] within episode for the FIR model
    def lag(x, k):
        out = np.zeros_like(x)
        ok = np.zeros(len(x), bool)
        ok[k:] = ep[k:] == ep[:-k]
        out[ok] = x[np.flatnonzero(ok) - k]
        return out
    r2, r3, r4 = lag(r, 1), lag(r, 2), lag(r, 3)

    jaw_open = f.A[:, 5] > np.quantile(f.A[:, 5], 0.30)
    results = {}
    print(f"{'joint':>6} {'M0 raw':>8} {'M1 K*r':>8} {'M2 +b':>8} "
          f"{'M3 rich':>8} {'M4 FIR':>8}   (held-out free-motion RMSE, counts)")
    for j in range(6):
        m_tr = jaw_open & tr
        m_te = jaw_open & te
        y_tr, y_te = delta[m_tr, j], delta[m_te, j]
        rj_tr, rj_te = r[m_tr, j], r[m_te, j]
        rows = {}
        rows["M0"] = float(np.sqrt(np.mean(y_te ** 2)))
        rows["M1"] = fit_eval(rj_tr[:, None], y_tr, rj_te[:, None], y_te)
        rows["M2"] = fit_eval(np.c_[rj_tr, np.ones_like(rj_tr)], y_tr,
                              np.c_[rj_te, np.ones_like(rj_te)], y_te)
        X3tr = np.c_[rj_tr, np.abs(rj_tr), np.sign(rj_tr),
                     rj_tr - r2[m_tr, j]]
        X3te = np.c_[rj_te, np.abs(rj_te), np.sign(rj_te),
                     rj_te - r2[m_te, j]]
        rows["M3"] = fit_eval(X3tr, y_tr, X3te, y_te)
        X4tr = np.c_[rj_tr, r2[m_tr, j], r3[m_tr, j], r4[m_tr, j]]
        X4te = np.c_[rj_te, r2[m_te, j], r3[m_te, j], r4[m_te, j]]
        rows["M4"] = fit_eval(X4tr, y_tr, X4te, y_te)
        results[j] = rows
        print(f"{j:6d} " + " ".join(f"{rows[m]:8.2f}"
                                    for m in ("M0", "M1", "M2", "M3", "M4")))

    jaw = results[5]
    best = min(("M1", "M2", "M3", "M4"), key=lambda m: jaw[m])
    gain_vs_m1 = (jaw["M1"] - jaw[best]) / jaw["M1"] * 100
    print(f"\njaw joint: best model {best}; improvement over M1 (paper's "
          f"model): {gain_vs_m1:.1f}% of residual RMSE")
    with open("results/compensator_ladder.json", "w") as fh:
        json.dump(results, fh, indent=2)
    print("wrote results/compensator_ladder.json")


if __name__ == "__main__":
    main()
