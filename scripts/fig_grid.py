"""Figure 2: the Task 2 observation-design grid (paper's central figure).

    python scripts/fig_grid.py

Per-arm seed dots + mean bar at n=100/seed, with the measured resolution
band (sigma_seed ~ 10 -> differences under ~15 points are not claimable).
Reads results/grid_{L}_{arm}_s{seed}.json; guarded column (Lg) plotted
separately when complete.
"""

from __future__ import annotations

import glob
import json
import re
from pathlib import Path

import numpy as np

ORDER = ["A", "B", "C", "D", "E", "F"]
LABELS = {
    "A": "A base\ns[t]",
    "B": "B +a[t−1]\n(raw history)",
    "C": "C +δ\n(tracking error)",
    "D": "D δ as aux\ntarget only",
    "E": "E δ − lag\nbaseline",
    "F": "F seat\nevent token",
}


def load(pattern: str):
    cells: dict[str, dict[int, int]] = {}
    for f in glob.glob(f"results/{pattern}"):
        m = re.match(r".*grid_([A-F])_([a-z_0-9]+)_s(\d)\.json", f)
        if not m:
            continue
        d = json.load(open(f))
        if "results" not in d:
            continue
        cells.setdefault(m.group(1), {})[int(m.group(3))] = d["results"][0]["success"]
    return cells


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cells = load("grid_[A-F]_*.json")
    fig, ax = plt.subplots(figsize=(8, 4.2))
    xs, means = [], []
    for k, letter in enumerate(ORDER):
        seeds = cells.get(letter, {})
        vals = [seeds[s] for s in sorted(seeds)]
        if not vals:
            continue
        ax.scatter([k] * len(vals), vals, s=28, color="#2b7bba", zorder=3)
        mean = float(np.mean(vals))
        ax.hlines(mean, k - 0.28, k + 0.28, color="#1a4f78", lw=2.5)
        ax.annotate(f"{mean:.1f}", (k + 0.32, mean), fontsize=9, va="center")
        xs.append(k)
        means.append(mean)
    base_mean = means[0] if means else 0.0
    n_seeds = max(len(v) for v in cells.values())
    res = 15.0 * (3.0 / n_seeds) ** 0.5
    ax.axhspan(base_mean, base_mean + res, color="gray", alpha=0.12,
               label=f"below resolution vs base (σ_seed≈10, n=100×{n_seeds})")
    ax.set_xticks(range(len(ORDER)), [LABELS[o] for o in ORDER], fontsize=8)
    ax.set_ylabel("placement success (%)")
    ax.set_ylim(-2, max(45, max(means) + 8))
    ax.legend(fontsize=8, loc="upper left")
    ax.annotate("simulation", xy=(0.99, 0.02), xycoords="axes fraction",
                ha="right", va="bottom", fontsize=8, color="gray")

    def bracket(i, j, y, text):
        ax.plot([i, i, j, j], [y - 1, y, y, y - 1], color="black", lw=0.8)
        ax.annotate(text, ((i + j) / 2, y + 0.6), ha="center", fontsize=8)

    idx = {L: k for k, L in enumerate(ORDER)}
    top = max(means) + 4
    bracket(idx["A"], idx["D"], top, "D − A = +0.4 [−6.2, +6.4]")
    bracket(idx["B"], idx["E"], top + 5.5, "E − B = +12.8 [+2.8, +22.6]")
    ax.set_ylim(-2, top + 10)
    fig.tight_layout()
    out = Path("figures/paper/fig2_grid.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
