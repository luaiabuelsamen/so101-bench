"""Analyse the crush-tier evaluation. Written before any result was read.

The grid scored every cell with the crush limit disabled, so success there needs
only contact DETECTION -- grip hard enough, and harder is never punished. Force
MAGNITUDE, which is what separates the tracking-error channel from generic
temporal context, cannot pay. Arm G carries almost no magnitude (force R^2 0.008
against delta's 0.749) yet matches the raw-channel arm at crush -1.

PREDICTION, frozen in eval_crush_tier.py before the sweep ran: at a crush tier E
beats G, because regulating grip into a band needs magnitude and G has none. If G
holds up under crush, the channel's distinguishing property does not pay even
where it should, and that is a finding against the paper.

READING RULES, fixed here before the numbers are seen:

1. The comparison is badly unbalanced: three G seeds against ONE E seed. A
   difference cannot be attributed to the arm rather than to E's seed. Treat any
   E-vs-G gap as directional only, and say so wherever it is reported.
2. Success rate is not the only outcome. At a crush tier a policy can fail by
   crushing or by dropping, and those are different failures: crushing is
   over-force, dropping is under-force. An arm carrying magnitude should crush
   LESS at a given success rate. The crush count is therefore the more
   diagnostic column and is reported alongside success.
3. If G's success collapses at crush while E's holds, that supports the
   prediction. If BOTH collapse, the tier is simply too tight to discriminate and
   the cell says nothing -- that is a null about the experiment, not about the
   channel.
4. No claim of resolution at n=1 for E. This is a screen, not a test.

    python scripts/analyze_crush_tier.py
"""

import glob
import json
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
EPISODES = 50


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * ((p * (1 - p) / n + z * z / (4 * n * n)) ** 0.5) / d
    return (max(0.0, c - h), min(1.0, c + h))


def main():
    rows = []
    for f in sorted(glob.glob(str(REPO / "results" / "crush_*.json"))):
        rows += json.loads(Path(f).read_text())
    if not rows:
        raise SystemExit("no crush_*.json yet")

    tiers = sorted({r["crush"] for r in rows}, reverse=True)
    arms = sorted({r["arm"] for r in rows})
    print(f"{'arm':>13}" + "".join(f"{('crush ' + str(int(t))):>26}" for t in tiers))
    print(f"{'':>13}" + "".join(f"{'success':>12}{'crush':>7}{'drop':>7}" for _ in tiers))
    for a in arms:
        line = f"{a:>13}"
        for t in tiers:
            m = [r for r in rows if r["arm"] == a and r["crush"] == t]
            if not m:
                line += f"{'--':>26}"
                continue
            r = m[0]
            lo, hi = wilson(r["success"], EPISODES)
            line += f"{r['success']:>4}/{EPISODES} [{lo:.2f},{hi:.2f}]"
            line += f"{r['crushed']:>7}{r['dropped']:>7}"
        print(line)

    # G is the only arm with more than one seed; pool it and compare directionally
    print()
    for t in tiers:
        g = [r for r in rows if r["arm"].startswith("G ") and r["crush"] == t]
        e = [r for r in rows if r["arm"].startswith("E ") and r["crush"] == t]
        b = [r for r in rows if r["arm"].startswith("B2") and r["crush"] == t]
        if not g or not e:
            continue
        gk = sum(r["success"] for r in g)
        gn = EPISODES * len(g)
        glo, ghi = wilson(gk, gn)
        ek, elo_hi = e[0]["success"], wilson(e[0]["success"], EPISODES)
        print(f"crush {int(t)}:")
        print(f"   G pooled {gk}/{gn} = {100*gk/gn:.1f}%  [{glo:.2f},{ghi:.2f}]   "
              f"crushes {sum(r['crushed'] for r in g)}/{gn}")
        print(f"   E        {ek}/{EPISODES} = {100*ek/EPISODES:.1f}%  "
              f"[{elo_hi[0]:.2f},{elo_hi[1]:.2f}]   crushes {e[0]['crushed']}/{EPISODES}")
        if b:
            print(f"   B2       {b[0]['success']}/{EPISODES} = "
                  f"{100*b[0]['success']/EPISODES:.1f}%   crushes {b[0]['crushed']}/{EPISODES}")
        if gk == 0 and ek == 0:
            print("   BOTH ZERO -- tier too tight to discriminate; says nothing"
                  " about the channel (rule 3).")
        else:
            print("   directional only: one E seed against three G seeds (rule 1).")
    print("\nCrush counts are the diagnostic column: over-force and under-force are")
    print("different failures, and an arm carrying magnitude should crush less.")


if __name__ == "__main__":
    main()
