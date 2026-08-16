"""Tuned-controller tournament: each grip controller at its best setpoint.

    python scripts/tournament.py --episodes 20 --json results/tournament.json

WHY SETPOINTS ARE SWEPT. An oracle regulator can always mimic the clamp by
targeting 300 N, so a correctly tuned oracle can never lose to it -- more
information cannot hurt a controller that is allowed to use it. An earlier
comparison here reported exactly that impossibility (oracle 18/30 against clamp
23/30 on the rigid task) because the oracle's setpoint was fixed at 15 N, too
weak to survive transport: it measured one bad tuning choice, not the value of
information. Every controller therefore gets its one knob swept per condition,
and the comparison of record is best-vs-best.

WHAT THE CLAMP'S KNOB CANNOT DO. The clamp's angle is chosen *before* the
episode, so one angle must serve every randomised block. Aperture changes
~51 mm per radian and the block's short side spans 15.2-19.0 mm under the
default randomisation, so a fixed angle that just grips the largest block
penetrates ~24 counts further into the smallest -- a spread of roughly 58 N
that no amount of tuning removes. The regulators measure the object each
episode; that, precisely, is what the information buys. The tournament makes
this quantitative instead of asserted.
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from so101_bench import DemoEnv, ExpertConfig

CLAMP_ANGLES = [-0.20, -0.10, -0.05, 0.00, 0.03]
FORCE_TARGETS = [10.0, 20.0, 30.0, 45.0]
ORACLE_TARGETS = [25.0, 45.0, 70.0, 100.0]


def run_cell(crush, mode, setpoint, episodes, seed):
    if mode == "clamp":
        cfg = ExpertConfig(grip_mode="clamp", clamp_jaw_angle=setpoint)
    elif mode == "force":
        cfg = ExpertConfig(grip_mode="force", grip_target_counts=setpoint)
    else:
        cfg = ExpertConfig(grip_mode="oracle", grip_target_newtons=setpoint)
    env = DemoEnv(seed=seed, cameras=(), crush_newtons=crush, expert=cfg)
    results = env.evaluate(episodes)
    peak = np.array([r.peak_grip_n for r in results])
    return {
        "crush_n": crush,
        "grip_mode": mode,
        "setpoint": setpoint,
        "episodes": episodes,
        "picked": int(sum(r.picked for r in results)),
        "placed": int(sum(r.placed for r in results)),
        "crushed": int(sum(r.crushed for r in results)),
        "peak_grip_median_n": float(np.median(peak)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--crush", type=float, nargs="*", default=[-1.0, 120.0, 100.0])
    parser.add_argument("--json", type=str, default=None)
    args = parser.parse_args()

    sweeps = [
        ("clamp", CLAMP_ANGLES),
        ("force", FORCE_TARGETS),
        ("oracle", ORACLE_TARGETS),
    ]
    rows, best = [], {}
    for crush_arg in args.crush:
        crush = None if crush_arg < 0 else crush_arg
        label = "none" if crush is None else f"{crush:.0f} N"
        print(f"--- crush limit {label}")
        for mode, setpoints in sweeps:
            cells = []
            for sp in setpoints:
                row = run_cell(crush, mode, sp, args.episodes, args.seed)
                rows.append(row)
                cells.append(row)
                print(
                    f"    {mode:>6} @ {sp:7.2f}: placed {row['placed']:2d}/{args.episodes}"
                    f"  crushed {row['crushed']:2d}  peak {row['peak_grip_median_n']:6.1f} N",
                    flush=True,
                )
            top = max(cells, key=lambda r: r["placed"])
            best[(label, mode)] = top
            print(f"    {mode:>6} BEST: {top['placed']}/{args.episodes} at {top['setpoint']}")
        print()

    print("=== tournament (best setpoint per controller) ===")
    print(f"{'crush':>8} {'clamp':>10} {'force':>10} {'oracle':>10}")
    for crush_arg in args.crush:
        label = "none" if crush_arg < 0 else f"{crush_arg:.0f} N"
        line = " ".join(
            f"{best[(label, m)]['placed']:6d}/{args.episodes}"
            for m in ("clamp", "force", "oracle")
        )
        print(f"{label:>8} {line}")

    if args.json:
        with open(args.json, "w") as fh:
            json.dump(rows, fh, indent=2)
        print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
