"""Figure: the mechanism — one servo, one grasp, the subtraction made visible.

    python scripts/fig_mechanism.py

Goal position vs measured position on the jaw servo through a scripted
grasp; their difference (delta) below on the same time axis; first contact
and the detector's seat event marked across both panels. Sim trace now;
the bench session swaps in the same plot from the real arm.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from so101_bench import DemoEnv, ExpertConfig  # noqa: E402
from so101_bench.guard import JawGuard  # noqa: E402


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    env = DemoEnv(seed=7, cameras=(), image_size=96,
                  expert=ExpertConfig(grip_mode="force"))
    result, rec = env.rollout(record=True)
    S = np.stack(rec.states)          # counts, stride-2 samples (15 Hz)
    A = np.stack(rec.actions)
    F = np.asarray(rec.forces)
    env.close()

    goal, meas = A[:, 5], S[:, 5]
    delta = np.empty_like(goal)
    delta[0], delta[1:] = 0.0, A[:-1, 5] - S[1:, 5]
    contact = int(np.flatnonzero(F > 0.5)[0])

    g = JawGuard(max_close_rate=1e9)
    seat = None
    for t in range(len(S)):
        g(float(meas[t]), float(goal[t - 1] if t else meas[0]))
        if g.seat_jaw is not None:
            seat = t
            break

    lo = max(contact - 45, 0)
    hi = min(contact + 60, len(S) - 1)
    tt = (np.arange(lo, hi) - contact) / 15.0     # seconds, 0 = contact

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(6.5, 4.2), sharex=True,
        gridspec_kw=dict(height_ratios=[2, 1.2], hspace=0.08))
    ax1.plot(tt, goal[lo:hi], color="#d95f02", lw=1.6, label="commanded (Goal_Position)")
    ax1.plot(tt, meas[lo:hi], color="#1b6ca8", lw=1.6, label="measured (Present_Position)")
    ax1.set_ylabel("jaw position (counts)")
    ax1.legend(fontsize=8, loc="upper right")

    ax2.plot(tt, np.abs(delta[lo:hi]), color="#444444", lw=1.6)
    ax2.fill_between(tt, 0, np.abs(delta[lo:hi]), color="#444444", alpha=0.15)
    ax2.set_ylabel("|δ| (counts)")
    ax2.set_xlabel("time from first contact (s)")
    ax2.annotate("≈ 2 N per count", (tt[-1], np.abs(delta[lo:hi]).max() * 0.85),
                 ha="right", fontsize=8, color="#444444")

    for ax in (ax1, ax2):
        ax.axvline(0, color="green", ls="--", lw=1.0)
        if seat is not None and lo < seat < hi:
            ax.axvline((seat - contact) / 15.0, color="purple", ls=":", lw=1.2)
    ymax = goal[lo:hi].max()
    ax1.annotate("first contact", (0, ymax * 0.62), fontsize=8,
                 color="green", ha="left", xytext=(4, 0), textcoords="offset points")
    if seat is not None:
        ax1.annotate("seat detected",
                     ((seat - contact) / 15.0, ymax * 0.40), fontsize=8,
                     color="purple", ha="left", xytext=(4, 0),
                     textcoords="offset points")
    k = int(len(tt) * 0.12)
    ax1.annotate("free close:\nservo tracks command", (tt[k], meas[lo + k]),
                 fontsize=8, color="#1b6ca8", ha="left",
                 xytext=(10, -26), textcoords="offset points")
    ax1.annotate("jaw held by object;\ncommand deepens → gap = force",
                 (tt[-1] * 0.55, meas[hi - 1]), fontsize=8, color="#1b6ca8",
                 ha="left", xytext=(0, 22), textcoords="offset points")

    ax1.set_title("The observation the log already contains: "
                  "δ = command − measured", fontsize=10)
    fig.tight_layout()
    out = Path("figures/paper/fig_mech.png")
    fig.savefig(out, dpi=200)
    print(f"wrote {out}  (contact@{contact}, seat@{seat}, "
          f"peak |δ| {np.abs(delta).max():.0f} counts, placed={result.placed})")


if __name__ == "__main__":
    main()
