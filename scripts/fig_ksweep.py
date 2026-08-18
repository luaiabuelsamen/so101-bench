"""Timing-robustness figure: alignment k vs free-motion residual and effect.

    python scripts/fig_ksweep.py

Left: per-dataset median free-motion |delta_k| (normalized to its k=0
value) -- the minimum estimates each dataset's command-to-state alignment;
corpus consensus k* ~ 3-4 recorded frames. Right: per-dataset Cohen's d of
delta_k-at-seat vs the frozen proxy, across the same sweep -- the Sec 4.3
conclusion is insensitive to the alignment convention. Data:
results/offset_sweep.json (roadmap C.1).
"""

import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = json.load(open("results/offset_sweep.json"))
ks = np.arange(6)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.6))
for r in rows:
    fm = np.array(r["free_median_by_k"], float)
    if fm[0] <= 0:
        continue
    ax1.plot(ks, fm / fm[0], color="#1b6ca8", alpha=0.35, lw=1.0)
med = np.median([np.array(r["free_median_by_k"]) / max(r["free_median_by_k"][0], 1e-9)
                 for r in rows], axis=0)
ax1.plot(ks, med, color="#1b6ca8", lw=2.5, label="median dataset")
ax1.axvspan(2.5, 4.5, color="#1b6ca8", alpha=0.08)
ax1.annotate("corpus consensus\nk* ≈ 3–4", (3.5, ax1.get_ylim()[1]*0.92),
             ha="center", fontsize=8, color="#14568a")
ax1.set_xlabel("alignment k (recorded frames)")
ax1.set_ylabel("free-motion |δₖ| (relative to k=0)")
ax1.set_title("Latency shows up as the residual minimum", fontsize=10)
ax1.legend(fontsize=8)

for r in rows:
    d = [x for x in r["cohen_d_by_k"]]
    if any(x is None for x in d):
        continue
    d = np.clip(np.array(d, float), -3, 3)
    ax2.plot(ks, d, color="#666666", alpha=0.45, lw=1.0)
ax2.axhline(0, color="#999999", lw=0.7)
ax2.set_ylim(-3.2, 3.2)
ax2.set_xlabel("alignment k (recorded frames)")
ax2.set_ylabel("Cohen's d of δₖ-at-seat (clipped ±3)")
ax2.set_title("The corpus conclusion does not care (median span 0.12)",
              fontsize=10)
fig.tight_layout()
out = Path("figures/paper/fig_ksweep.png")
fig.savefig(out, dpi=200)
print(f"wrote {out}")
