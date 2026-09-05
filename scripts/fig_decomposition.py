"""Figure: the declared primary decomposes into two unresolved halves.

    python scripts/fig_decomposition.py

Left  -- the headline E − B walked as two steps through C, the arm that shares
         B's raw information and E's representation.
Right -- forest plot of every pairwise comparison the grid supports, marked by
         whether the two arms observe the SAME raw information. Only C−B and
         D−A are matched; E−B is not, because E consumes a[t−2] through the
         commanded rate (train_act.py:118-131,190).

Reads results/grid_{L}_{arm}_s{seed}.json. CPU only, no checkpoints.
See docs/findings.md (2026-09-03) for the audit this figure reports.

CI METHOD -- two-level percentile bootstrap, and it matters. Outer level
resamples TRAINING SEEDS with replacement, independently per arm (arms at the
same seed index are independent training runs). Inner level redraws each
resampled seed's success count from Binomial(100, p_hat), i.e. the evaluation
sampling error at n=100 episodes/seed. 40k resamples, fixed RNG seed 0, so the
figure is deterministic.

Seed-only resampling is NOT enough: it omits the eval binomial term and gives
intervals ~2.4 points too narrow (E-B [+4.0, +21.2] instead of [+2.8, +22.6]).
The two-level form reproduces every interval in main.tex Table 1 to within 0.2
of both endpoints across E-A, E-B, C-A and C-B, which is how the paper's
undocumented method was recovered -- it was not written down in any committed
script. Now it is. check_paper_numbers.py can verify the published CIs against
this, not just the point estimates.
"""

from __future__ import annotations

import glob
import json
import re
from pathlib import Path

import numpy as np

ARMS = {"A": "base", "B": "base_hist", "C": "delta",
        "D": "resid", "E": "excess", "F": "token"}
BOOT = 40000
N_EVAL = 100          # episodes per seed; the inner bootstrap level
RNG = np.random.default_rng(0)


def load():
    cells: dict[str, dict[int, int]] = {}
    for p in glob.glob("results/grid_[A-F]_*_s[0-9].json"):
        m = re.match(r"results/grid_([A-F])_(.+)_s(\d)\.json", p)
        if not m:
            continue
        d = json.load(open(p))
        row = next(r for r in d["results"] if r["crush"] == -1)
        cells.setdefault(m.group(1), {})[int(m.group(3))] = row["success"]
    return {k: np.array([v[s] for s in sorted(v)], float) for k, v in cells.items()}


def boot_ci(x, y):
    """95% two-level percentile CI on mean(x) - mean(y).

    Outer: resample training seeds. Inner: redraw each seed's success count
    from Binomial(N_EVAL, p_hat). See the module docstring on why the inner
    level is required to match the published intervals.
    """
    d = np.empty(BOOT)
    for i in range(BOOT):
        xs = RNG.choice(x, x.size)
        ys = RNG.choice(y, y.size)
        xs = RNG.binomial(N_EVAL, np.clip(xs, 0, N_EVAL) / N_EVAL).astype(float)
        ys = RNG.binomial(N_EVAL, np.clip(ys, 0, N_EVAL) / N_EVAL).astype(float)
        d[i] = xs.mean() - ys.mean()
    return (float(x.mean() - y.mean()),
            float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5)))


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    v = load()
    n = max(len(a) for a in v.values())
    res = 15.0 * (3.0 / n) ** 0.5          # grid's own resolution rule

    fig, (axl, axr) = plt.subplots(1, 2, figsize=(11.5, 5.0),
                                   gridspec_kw={"width_ratios": [1, 1.3]})

    # ---- left: the headline walked through C -------------------------------
    mB, mC, mE = v["B"].mean(), v["C"].mean(), v["E"].mean()
    axl.bar(0, mB, 0.55, color="#9ecae1", zorder=2)
    axl.bar(1, mC, 0.55, color="#4292c6", zorder=2)
    axl.bar(2, mE, 0.55, color="#1a4f78", zorder=2)
    for k, arm in enumerate("BCE"):
        axl.scatter([k] * v[arm].size, v[arm], s=22, color="#08306b",
                    zorder=4, alpha=.75)
    tops = [v[a].max() for a in "BCE"]
    for k, (m, t) in enumerate(zip((mB, mC, mE), tops)):
        axl.annotate(f"{m:.1f}", (k, t + 1.4), ha="center", fontsize=9,
                     fontweight="bold")
    ybar = max(tops) + 7.0            # both step arrows on one clear line
    for x0, x1, txt in ((0, 1, "C−B  +8.0\nsame information,\ndifferent form\n(unresolved)"),
                        (1, 2, "E−C  +4.8\nadds a[t−2]\n(unresolved)")):
        axl.annotate("", (x1 - .04, ybar), (x0 + .04, ybar),
                     arrowprops=dict(arrowstyle="->", color="#444", lw=1.1))
        axl.annotate(txt, ((x0 + x1) / 2, ybar + 0.7), ha="center",
                     fontsize=7.5, color="#444")
    axl.set_xticks(range(3), ["B\n[s, a[t−1]]", "C\n[s, δ]", "E\n[s, δ−K̂r]"],
                   fontsize=8)
    axl.set_ylabel("placement success (%)")
    axl.set_title("the declared primary, walked through C", fontsize=9.5)
    axl.set_ylim(0, ybar + 9)
    axl.annotate("simulation", (0.99, 0.02), xycoords="axes fraction",
                 ha="right", va="bottom", fontsize=8, color="gray")

    # ---- right: forest plot, matched vs confounded --------------------------
    # kind: "matched" = same raw information, so a genuine FORM comparison.
    # "confounded" = presented as a form comparison but content differs too.
    # "content" = deliberately a content comparison; mismatch is the point.
    rows = [
        ("E − A", "E", "A", "content",    "channel vs none"),
        ("E − B", "E", "B", "confounded", "DECLARED PRIMARY · E also sees a[t−2]"),
        ("C − B", "C", "B", "matched",    "form, at matched information"),
        ("E − C", "E", "C", "confounded", "the a[t−2] frame alone"),
        ("D − A", "D", "A", "matched",    "form, at matched information"),
    ]
    STYLE = {"matched":    ("#1a4f78", "o"),
             "confounded": ("#c0392b", "D"),
             "content":    ("#7f7f7f", "s")}
    ys = np.arange(len(rows))[::-1]
    for y, (lab, a, b, kind, note) in zip(ys, rows):
        m, lo, hi = boot_ci(v[a], v[b])
        col, mk = STYLE[kind]
        axr.plot([lo, hi], [y, y], color=col, lw=2.2, solid_capstyle="round")
        axr.scatter([m], [y], s=52, color=col, zorder=4, marker=mk)
        axr.annotate(f"{m:+.1f}  [{lo:+.1f}, {hi:+.1f}]", (hi + 1.2, y),
                     va="center", fontsize=8)
        axr.annotate(note, (-38, y + 0.20), va="bottom", ha="left",
                     fontsize=7, color="#555")
    axr.axvline(0, color="black", lw=0.9)
    axr.axvspan(-res, res, color="gray", alpha=0.12)
    axr.set_yticks(ys, [r[0] for r in rows], fontsize=9)
    axr.set_xlabel("difference in placement success (points), 95% bootstrap CI")
    axr.set_xlim(-40, 52)
    axr.set_ylim(-0.75, len(rows) - 0.35)
    axr.set_title("every comparison the grid supports", fontsize=9.5)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    axr.legend(handles=[
        Line2D([], [], color="#1a4f78", marker="o", ls="none",
               label="form comparison, information matched"),
        Line2D([], [], color="#c0392b", marker="D", ls="none",
               label="form comparison, information NOT matched"),
        Line2D([], [], color="#7f7f7f", marker="s", ls="none",
               label="content comparison (mismatch is the point)"),
        Patch(facecolor="gray", alpha=0.12,
              label=f"below the grid's resolution (±{res:.0f} pts, n=100×{n})"),
    ], fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=2,
        frameon=False)
    axr.annotate("simulation", (0.99, 0.02), xycoords="axes fraction",
                 ha="right", va="bottom", fontsize=8, color="gray")

    fig.tight_layout()
    out = Path("figures/paper/fig_decomposition.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160)
    print(f"wrote {out}")
    for lab, a, b, kind, _ in rows:
        m, lo, hi = boot_ci(v[a], v[b])
        print(f"  {lab}: {m:+.1f} [{lo:+.1f}, {hi:+.1f}]  {kind}")


if __name__ == "__main__":
    main()
