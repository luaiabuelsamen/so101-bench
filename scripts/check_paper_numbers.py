"""Check the paper's load-bearing numbers against the frozen result files.

Twice in one day the paper stated a number the data did not support: the guard
paragraph said "one policy" where the result was every policy, and a crush count
over 180 episodes was quoted beside a force range over 270. Both were recorded in
docs/findings.md and neither reached main.tex. The failure mode is the paper
drifting from data that a script already reproduces, so this is that script.

It recomputes each claim from results/ and asserts the value appears in the
paper. It is deliberately literal: it does not parse LaTeX, it checks that the
rendered number is present, which is enough to catch a stale edit and cheap
enough to run on every build.

    python scripts/check_paper_numbers.py          # exits non-zero on drift
"""

import glob
import json
import re
import statistics as st
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TEX = REPO / "paper" / "main.tex"


def grid_means():
    out = {}
    for pat, name in [("grid_A_base_s*", "A"), ("grid_B_base_hist_s*", "B"),
                      ("grid_C_delta_s*", "C"), ("grid_D_resid_s*", "D"),
                      ("grid_E_excess_s*", "E"), ("grid_F_token_s*", "F")]:
        vals = []
        for f in sorted(glob.glob(str(REPO / "results" / f"{pat}.json"))):
            for r in json.loads(Path(f).read_text())["results"]:
                if r["crush"] == -1.0:
                    vals.append(r["success"])
        if vals:
            out[name] = st.mean(vals)
    return out


def guard_totals():
    rows = json.loads((REPO / "results" / "guard_ab.json").read_text())
    keyed = {(r["arm"], r["crush"], bool(r["guarded"])): r for r in rows}
    live = [r for r in rows if r["crush"] > 0]
    t = dict(
        crush_u=sum(r["crushed"] for r in live if not r["guarded"]),
        crush_g=sum(r["crushed"] for r in live if r["guarded"]),
        eps_live=sum(r["episodes"] for r in live if not r["guarded"]),
        succ_u=sum(r["success"] for r in rows if not r["guarded"]),
        succ_g=sum(r["success"] for r in rows if r["guarded"]),
        eps_all=sum(r["episodes"] for r in rows if not r["guarded"]),
        zero_cells=sum(1 for r in rows if r["guarded"] and r["success"] == 0),
        n_cells=sum(1 for r in rows if r["guarded"]),
    )
    pf_live_u = [r["peak_force_median"] for r in live if not r["guarded"]]
    pf_live_g = [r["peak_force_median"] for r in live if r["guarded"]]
    pf_all_u = [r["peak_force_median"] for r in rows if not r["guarded"]]
    t["pf_live"] = (min(pf_live_u), max(pf_live_u), min(pf_live_g), max(pf_live_g))
    t["pf_all_u"] = (min(pf_all_u), max(pf_all_u))
    _ = keyed
    return t


def main():
    tex = TEX.read_text()
    # collapse whitespace so a claim split across lines still matches
    flat = re.sub(r"\s+", " ", tex)
    fails, checks = [], 0

    def want(label, needle, note=""):
        nonlocal checks
        checks += 1
        if needle not in flat:
            fails.append(f"{label}: {needle!r} not in main.tex {note}")

    g = grid_means()
    for arm, v in g.items():
        pass  # means are quoted selectively; the pairs below are what the paper claims

    if {"E", "A", "B"} <= g.keys():
        want("E mean", f"{g['E']:.1f}\\%")
        want("A mean", f"{g['A']:.1f}\\%")
        want("B mean", f"{g['B']:.1f}\\%")
    if {"E", "C", "B"} <= g.keys():
        eb, ec, cb = g["E"] - g["B"], g["E"] - g["C"], g["C"] - g["B"]
        want("E-B decomposition", f"+{eb:.1f}")
        want("E-C component", f"+{ec:.1f}")
        want("C-B component", f"+{cb:.1f}", "(the clean representation test)")

    t = guard_totals()
    want("crush reduction", f"{t['crush_u']} to {t['crush_g']} of {t['eps_live']}")
    want("guarded successes", f"{t['succ_g']} of {t['eps_all']}")
    want("unguarded successes", f"{t['succ_u']}")
    lo_u, hi_u, lo_g, hi_g = t["pf_live"]
    want("peak force, crush tiers",
         f"{lo_u:.0f}--{hi_u:.0f}\\,N to {lo_g:.0f}--{hi_g:.0f}\\,N",
         "(must be the 180-episode range, not the 270-episode one)")

    print(f"checked {checks} claims against results/")
    if fails:
        print("\nDRIFT — the paper does not match the data:")
        for f in fails:
            print(f"  {f}")
        return 1
    print("all load-bearing numbers match the frozen results")
    return 0


if __name__ == "__main__":
    sys.exit(main())
