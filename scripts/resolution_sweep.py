"""Does force-channel resolution decide task success, and where is the cliff?

    python scripts/resolution_sweep.py --episodes 30 --json results.json

The environment supplies a grip *window*: the block must be held firmly enough
to survive transport and gently enough not to exceed ``--crush``. The
force-regulated expert commands grip as an integer overshoot in encoder counts
past the detected contact point, so coarsening the observation quantum directly
coarsens the force it can command.

PREREGISTERED PREDICTION. Grip force changes by about 2.4 N per encoder count
(measured; see ``docs/findings.md``), and the expert's typical peak grip is
~91 N. The usable margin at a crush limit ``C`` is therefore ``C - 91`` newtons,
and the quantum at which control fails should be

    q_cliff ~= (C - 91) / 2.4      counts

So the cliff must MOVE with the crush limit: tighter limits fail at finer
quanta. A resolution effect that appears at every crush limit alike is not a
resolution effect -- it is an artifact of coarsening an input.

Reference rows are run at every crush limit:

* ``clamp``  -- no force feedback; expected to fail whenever a limit exists
* ``oracle`` -- true contact force; the upper bound the proxy is measured against
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from so101_bench import DemoEnv, ExpertConfig

#: Measured grip response, used only to state the prediction. See findings.
NEWTONS_PER_COUNT = 2.4
TYPICAL_PEAK_N = 91.0


#: Grip setpoints. ``force`` regulates the quantised tracking error to this many
#: counts (~45 N at 2.4 N/count); ``oracle`` regulates the true force to the same
#: nominal grip, so the two controllers aim at the same physical target and
#: differ only in what they are able to measure.
FORCE_SETPOINT_COUNTS = 20.0
ORACLE_SETPOINT_N = 45.0


def cell(crush: float | None, quantum: float, mode: str, episodes: int, seed: int) -> dict:
    kwargs: dict[str, float] = {}
    if mode == "force":
        kwargs = {"grip_target_counts": FORCE_SETPOINT_COUNTS}
    elif mode == "oracle":
        kwargs = {"grip_target_newtons": ORACLE_SETPOINT_N}
    env = DemoEnv(
        seed=seed,
        cameras=(),
        crush_newtons=crush,
        obs_quantum=quantum,
        expert=ExpertConfig(grip_mode=mode, **kwargs),
    )
    results = env.evaluate(episodes)
    peak = np.array([r.peak_grip_n for r in results])
    return {
        "crush_n": crush,
        "obs_quantum": quantum,
        "grip_mode": mode,
        "episodes": episodes,
        "picked": int(sum(r.picked for r in results)),
        "placed": int(sum(r.placed for r in results)),
        "crushed": int(sum(r.crushed for r in results)),
        "peak_grip_median_n": float(np.median(peak)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--crush", type=float, nargs="*", default=[100.0, 120.0, 160.0])
    parser.add_argument("--quanta", type=float, nargs="*", default=[1.0, 4.0, 8.0, 16.0, 32.0])
    parser.add_argument("--json", type=str, default=None)
    args = parser.parse_args()

    rows: list[dict] = []
    print(
        f"predicted cliff = (crush - {TYPICAL_PEAK_N:.0f})"
        f" / {NEWTONS_PER_COUNT:.1f} counts\n"
    )
    for crush in args.crush:
        predicted = (crush - TYPICAL_PEAK_N) / NEWTONS_PER_COUNT
        print(f"--- crush {crush:.0f} N   predicted cliff ~{predicted:.0f} counts")
        header = " ".join(f"{q:>7.2f}" for q in args.quanta)
        print(f"    {'placed/30 by quantum':<24}{header}")
        placed = []
        for quantum in args.quanta:
            row = cell(crush, quantum, "force", args.episodes, args.seed)
            rows.append(row)
            placed.append(row["placed"])
        print(f"    {'force (delta)':<24}" + " ".join(f"{p:>7d}" for p in placed), flush=True)

        refs = []
        for mode in ("clamp", "oracle"):
            row = cell(crush, 1.0, mode, args.episodes, args.seed)
            rows.append(row)
            refs.append((mode, row["placed"]))
        print("    " + "  ".join(f"{m}: {p}/{args.episodes}" for m, p in refs), flush=True)
        print()

    if args.json:
        with open(args.json, "w") as fh:
            json.dump(rows, fh, indent=2)
        print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
