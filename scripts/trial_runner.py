"""Run repeated real-arm policy trials and record a success rate.

One rollout is an anecdote. This runs a paired sequence -- alternating arms from
the same reset scene -- and writes an outcome file you can compute a rate from.

Pairing matters more than trial count here. Alternating two policies over the
same physical resets removes the between-reset variation that would otherwise
swamp the comparison, which is the same reason the simulated grid evaluates on
paired episodes.

You reset the object between trials; the script waits for you. The outcome is
recorded from your keypress, because nothing on this bench can see whether the
adapter ended up in the tape roll.

    python scripts/trial_runner.py --trials 10 --arms base,excess

Outcomes land in results/real_trials.json, appended, so runs accumulate across
sessions rather than overwriting.
"""

import argparse
import json
import subprocess
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
# Always the venv interpreter, never sys.executable: this script is convenient
# to launch with the system python, which has NumPy 2.x and cannot import the
# cv2/torch build the rest of the pipeline uses.
PY = "/home/jetson3/projects/clean_env/venv/bin/python"
CKPT = {"base": "checkpoints/real50_base_s0",
        "excess": "checkpoints/real50_excess_s0",
        "ghist": "checkpoints/real50_ghist_s0",
        "base_trim": "checkpoints/real50_base_trim"}


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * ((p * (1 - p) / n + z * z / (4 * n * n)) ** 0.5) / d
    return (max(0.0, c - h), min(1.0, c + h))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10, help="total rollouts")
    ap.add_argument("--arms", default="base", help="comma-separated, alternated")
    ap.add_argument("--jumpstart", type=int, default=70)
    ap.add_argument("--max-steps", type=int, default=400)
    ap.add_argument("--root", default="data/real/pickplace_real_v0")
    ap.add_argument("--out", default="results/real_trials.json")
    args = ap.parse_args()

    arms = [a.strip() for a in args.arms.split(",")]
    for a in arms:
        if not (REPO / CKPT[a]).exists():
            raise SystemExit(f"missing checkpoint for {a}: {CKPT[a]}")

    out = REPO / args.out
    records = json.loads(out.read_text()) if out.exists() else []
    session = datetime.now().isoformat(timespec="seconds")
    traj_dir = REPO / "results" / "real_trial_traj"
    traj_dir.mkdir(parents=True, exist_ok=True)

    print(f"{args.trials} trials, alternating {arms}, jumpstart {args.jumpstart} frames.")
    print("Between trials: put the adapter back at its start position and clear the tape roll.\n")

    for t in range(args.trials):
        arm = arms[t % len(arms)]
        input(f"--- trial {t+1}/{args.trials}  arm={arm}  "
              f"[reset the scene, then press Enter]")
        tag = f"{session.replace(':','')}_{t:02d}_{arm}"
        cmd = [PY, str(REPO / "scripts" / "run_policy_real.py"),
               "--checkpoint", CKPT[arm], "--arm", arm,
               "--home", args.root, "--jumpstart", str(args.jumpstart),
               "--max-steps", str(args.max_steps),
               "--out-traj", str(traj_dir / f"{tag}.npz")]
        subprocess.run(cmd, cwd=REPO)

        while True:
            v = input("    outcome -- [s]uccess (adapter in the tape roll), "
                      "[f]ail, [x] discard this trial: ").strip().lower()
            if v in ("s", "f", "x"):
                break
        if v == "x":
            print("    discarded")
            continue
        records.append(dict(session=session, trial=t, arm=arm,
                            success=(v == "s"), jumpstart=args.jumpstart,
                            traj=f"real_trial_traj/{tag}.npz"))
        out.write_text(json.dumps(records, indent=2))

        done = [r for r in records if r["arm"] == arm]
        k = sum(r["success"] for r in done)
        lo, hi = wilson(k, len(done))
        print(f"    {arm}: {k}/{len(done)}  Wilson 95% [{lo:.2f}, {hi:.2f}]")

    print("\n=== session summary ===")
    for a in arms:
        done = [r for r in records if r["arm"] == a]
        k = sum(r["success"] for r in done)
        lo, hi = wilson(k, len(done))
        print(f"  {a:10} {k:3d}/{len(done):<3d}  {100*k/max(len(done),1):5.1f}%  "
              f"Wilson 95% [{lo:.2f}, {hi:.2f}]")
    print(f"\nwrote {out}")
    print("These are jumpstarted rollouts on 50 demonstrations. A rate here is a")
    print("property of this pipeline at this budget, not a result about the channel.")


if __name__ == "__main__":
    main()
