"""Guard Pareto accounting (roadmap docs/roadmap.md:219-220).

Reports crushes, successes, deletions, false interventions and peak force as
one table, from the paired guarded/unguarded runs already in results/.

Definitions, grounded in eval_guard.py and paper/main.tex:sec:guard:
  success    placed = picked and block_in_box and not crushed
  crush      scene.crushed
  drop       picked and not placed and not crushed
  deletion   a success present unguarded that the guard removes -- the paper's
             "it also deletes successes one policy earned by over-gripping"
  false
  intervention
             a deletion in the crush = -1 tier, where crushing is impossible and
             so the guard had nothing to prevent

Deletions are a net count per paired cell, not an episode-level match: the runs
are paired by seed, but the stored rows carry only aggregate counts, so a cell
where the guard deletes one success and earns another nets to zero here. Treat
the deletion column as a lower bound.

Usage:
    python scripts/guard_pareto.py [--file results/guard_ab.json] [--tex]
"""

import argparse
import json
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def load(path):
    rows = json.loads(Path(path).read_text())
    keyed = {}
    for r in rows:
        keyed[(r["arm"], r["crush"], bool(r["guarded"]))] = r
    return rows, keyed


def build(rows, keyed):
    """One record per (arm, crush tier) pairing the guarded and unguarded run."""
    out = []
    seen = []
    for r in rows:
        k = (r["arm"], r["crush"])
        if k not in seen:
            seen.append(k)
    for arm, crush in seen:
        u = keyed.get((arm, crush, False))
        g = keyed.get((arm, crush, True))
        if u is None or g is None:
            continue
        crushable = crush > 0
        deletions = max(0, u["success"] - g["success"])
        out.append(
            dict(
                arm=arm,
                crush=crush,
                crushable=crushable,
                episodes=u["episodes"],
                succ_u=u["success"],
                succ_g=g["success"],
                crush_u=u["crushed"],
                crush_g=g["crushed"],
                drop_u=u["dropped"],
                drop_g=g["dropped"],
                pf_u=u["peak_force_median"],
                pf_g=g["peak_force_median"],
                deletions=deletions,
                false_interventions=0 if crushable else deletions,
            )
        )
    return out


def totals(recs):
    """Crush-relevant totals: the tiers where a crush is actually possible."""
    live = [r for r in recs if r["crushable"]]
    return dict(
        episodes=sum(r["episodes"] for r in live),
        crush_u=sum(r["crush_u"] for r in live),
        crush_g=sum(r["crush_g"] for r in live),
        succ_u=sum(r["succ_u"] for r in live),
        succ_g=sum(r["succ_g"] for r in live),
        deletions=sum(r["deletions"] for r in live),
        false_interventions=sum(r["false_interventions"] for r in recs),
        pf_u_lo=min((r["pf_u"] for r in live), default=0.0),
        pf_u_hi=max((r["pf_u"] for r in live), default=0.0),
        pf_g_lo=min((r["pf_g"] for r in live), default=0.0),
        pf_g_hi=max((r["pf_g"] for r in live), default=0.0),
    )


def fmt_text(recs, t):
    w = f"{'arm':>10} {'crush':>7} {'succ u>g':>10} {'crush u>g':>11} {'drop u>g':>10} {'peakF u>g':>16} {'del':>4} {'false':>6}"
    lines = [w, "-" * len(w)]
    for r in recs:
        tier = "none" if not r["crushable"] else f"{r['crush']:.0f}N"
        lines.append(
            f"{r['arm']:>10} {tier:>7} "
            f"{r['succ_u']:4d} > {r['succ_g']:<3d} "
            f"{r['crush_u']:5d} > {r['crush_g']:<3d} "
            f"{r['drop_u']:4d} > {r['drop_g']:<3d} "
            f"{r['pf_u']:6.1f} > {r['pf_g']:<6.1f} "
            f"{r['deletions']:4d} {r['false_interventions']:6d}"
        )
    red = (1 - t["crush_g"] / t["crush_u"]) * 100 if t["crush_u"] else float("nan")
    lines += [
        "-" * len(w),
        f"crush-capable tiers only: {t['crush_u']} -> {t['crush_g']} crushes "
        f"of {t['episodes']} episodes ({red:.0f}% reduction)",
        f"peak force median range: {t['pf_u_lo']:.0f}-{t['pf_u_hi']:.0f} N -> "
        f"{t['pf_g_lo']:.0f}-{t['pf_g_hi']:.0f} N",
        f"successes: {t['succ_u']} -> {t['succ_g']}  (deletions {t['deletions']}, "
        f"false interventions {t['false_interventions']} in the no-crush tier)",
    ]
    return "\n".join(lines)


def fmt_tex(recs, t):
    lines = [
        r"\begin{tabular}{llrrrrrr}",
        r"\toprule",
        r"arm & crush & \multicolumn{2}{c}{success} & \multicolumn{2}{c}{crushes} "
        r"& \multicolumn{2}{c}{peak force (N)} \\",
        r"\cmidrule(lr){3-4}\cmidrule(lr){5-6}\cmidrule(lr){7-8}",
        r" & & off & on & off & on & off & on \\",
        r"\midrule",
    ]
    for r in recs:
        tier = "none" if not r["crushable"] else f"{r['crush']:.0f}\\,N"
        lines.append(
            f"{r['arm'].replace('_', chr(92) + '_')} & {tier} & {r['succ_u']} & {r['succ_g']} & "
            f"{r['crush_u']} & {r['crush_g']} & {r['pf_u']:.1f} & {r['pf_g']:.1f} \\\\"
        )
    red = (1 - t["crush_g"] / t["crush_u"]) * 100 if t["crush_u"] else float("nan")
    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        f"% crush-capable tiers: {t['crush_u']}->{t['crush_g']} of {t['episodes']} ({red:.0f}\\%)",
    ]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default=str(REPO / "results" / "guard_ab.json"))
    ap.add_argument("--tex", action="store_true", help="emit a LaTeX tabular")
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    rows, keyed = load(args.file)
    recs = build(rows, keyed)
    t = totals(recs)

    print(f"# {Path(args.file).name}\n")
    print(fmt_tex(recs, t) if args.tex else fmt_text(recs, t))

    if args.json_out:
        Path(args.json_out).write_text(json.dumps(dict(rows=recs, totals=t), indent=2))
        print(f"\nwrote {args.json_out}")


if __name__ == "__main__":
    main()
