"""Pre-registered analysis of the B2 and G cells.

WRITTEN BEFORE THE THIRD SEED EXISTED. At authoring time two B2 seeds had
completed and no G seed had. The point of writing it now is that the comparison,
the interval method and the decision rule are fixed before the data can
influence them; running it later then has no degrees of freedom left.

Decision rule, frozen in docs/task2_arms.md before the cells launched:
claim a difference only at >= 15 points. sigma_seed on this grid is ~10 and
E's own five seeds span 17-36, so smaller gaps are not resolvable at n=3.

Readings, also frozen before the cells ran:

  E >= B2   strong evidence for the free-motion subtraction. B2 is
            information-matched to E -- both see s[t], a[t-1], a[t-2] -- so a
            gap cannot be the extra action frame and must be the form.
  E <  B2   AMBIGUOUS, and must not be reported as a clean negative. B2 is
            18-dim against E's 12, so raw two-frame access winning and extra
            observation width helping are not separable by this cell.

  E vs G    G is [s[t], s[t-8]], width-matched to E at 12 dims but carrying
            position history instead of the channel. E > G is evidence the
            channel is not substitutable by joint history. G bounds the
            long-context alternative of zeng2026revisiting; it does not
            settle it, since G is one frame at lag 8 and their effect is
            T_o = 12-20.

Intervals are the paper's two-level bootstrap: resample seeds, then redraw each
resampled seed's success count from Binomial(100, p_hat). Seed-only resampling
understates width by 2.2-2.6 points on this grid and would overstate precision.

    python scripts/analyze_b2_g.py
"""

import glob
import json
import statistics as st
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
EPISODES = 100
CLAIM_THRESHOLD = 15.0
RNG = np.random.default_rng(20260904)

ARMS = {
    "A base": "grid_A_base_s*",
    "B hist": "grid_B_base_hist_s*",
    "C delta": "grid_C_delta_s*",
    "E excess": "grid_E_excess_s*",
    "B2 hist2": "grid_H_base_hist2_s*",
    "G poshist": "grid_G_ghist_s*",
}


def seeds(pattern):
    out = []
    for f in sorted(glob.glob(str(REPO / "results" / f"{pattern}.json"))):
        for r in json.loads(Path(f).read_text())["results"]:
            if r["crush"] == -1.0:
                out.append(r["success"])
    return np.array(out, float)


def boot_ci(x, y, n=40000):
    """Two-level: resample seeds, then the per-seed episode binomial."""
    d = np.empty(n)
    for i in range(n):
        xs = RNG.choice(x, len(x), replace=True)
        ys = RNG.choice(y, len(y), replace=True)
        xs = RNG.binomial(EPISODES, np.clip(xs / EPISODES, 0, 1))
        ys = RNG.binomial(EPISODES, np.clip(ys / EPISODES, 0, 1))
        d[i] = xs.mean() - ys.mean()
    return float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))


def welch(x, y):
    if len(x) < 2 or len(y) < 2:
        return float("nan")
    vx, vy = st.variance(x), st.variance(y)
    denom = (vx / len(x) + vy / len(y)) ** 0.5
    return (st.mean(x) - st.mean(y)) / denom if denom else float("nan")


def main():
    data = {k: seeds(p) for k, p in ARMS.items()}
    print(f"{'arm':<12}{'n':>3}{'seeds':>22}{'mean':>8}{'sd':>7}")
    for k, v in data.items():
        if len(v):
            sd = st.stdev(v) if len(v) > 1 else 0.0
            print(f"{k:<12}{len(v):>3}{str([int(x) for x in v]):>22}{v.mean():8.1f}{sd:7.1f}")
        else:
            print(f"{k:<12}{0:>3}{'(not run)':>22}")
    print()

    incomplete = [k for k, v in data.items() if 0 < len(v) < 3]
    if incomplete:
        print(f"INCOMPLETE, not interpreting: {', '.join(incomplete)} have fewer than 3 seeds.")
        print("The frozen rule needs 3 seeds. Re-run when the cells finish.\n")

    pairs = [
        ("E excess", "B2 hist2", "isolates the subtraction: information-matched"),
        ("E excess", "G poshist", "channel vs position history: width-matched"),
        ("E excess", "B hist", "the paper's declared primary: confounded"),
        ("B2 hist2", "B hist", "value of the second action frame alone"),
        ("G poshist", "B hist", "position history vs one action frame"),
    ]
    print(f"{'comparison':<26}{'diff':>7}{'t':>7}   {'95% CI':>18}  verdict")
    for a, b in [(p[0], p[1]) for p in pairs]:
        x, y = data.get(a), data.get(b)
        if x is None or y is None or len(x) < 3 or len(y) < 3:
            print(f"{a+' - '+b:<26}{'':>7}{'':>7}   {'':>18}  not yet complete")
            continue
        d = x.mean() - y.mean()
        lo, hi = boot_ci(x, y)
        t = welch(x, y)
        if abs(d) >= CLAIM_THRESHOLD and lo * hi > 0:
            v = "resolved"
        elif lo * hi > 0:
            v = "interval excludes 0 but below the 15-point rule"
        else:
            v = "unresolved"
        print(f"{a+' - '+b:<26}{d:+7.1f}{t:+7.2f}   [{lo:+6.1f},{hi:+6.1f}]  {v}")

    print()
    E, B2 = data.get("E excess"), data.get("B2 hist2")
    if E is not None and B2 is not None and len(B2) >= 3:
        d = E.mean() - B2.mean()
        if d >= 0:
            print("E >= B2: reading is the frozen one -- evidence for the subtraction,")
            print("since B2 is information-matched and a gap cannot be the extra frame.")
        else:
            print("E < B2: AMBIGUOUS by the frozen reading. B2 is 18-dim against E's 12,")
            print("so raw two-frame access winning and width helping are not separable")
            print("here. Do not report this as a clean negative on the subtraction.")


if __name__ == "__main__":
    main()
