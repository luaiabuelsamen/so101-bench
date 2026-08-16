"""Phase 2: measure the physical safe-grip interval, without feedback.

    python scripts/phase2_window.py --json results/phase2_window.json

Pre-declared protocol, configuration v1 (stock pads, unchanged since
characterisation):

* Object: 8.5 mm half-width block, centred grasp, +-2 mm xy jitter per rep.
* Setup: the block is seated and brought to the target force using the oracle
  regulator. Setup MAY use privileged force -- this is a physical feasibility
  measurement, not a controller evaluation. The jaw command is then FROZEN.
* Disturbance: the fixed 3.7 s transport trajectory validated in Phase 1c,
  followed by a 1.0 s stationary hold. No feedback of any kind runs during it.
* Success: the block is retained -- its displacement in the tool frame stays
  under 5 mm through the disturbance.
* 5 repetitions per force level; a level PASSES at >= 4/5 (declared here,
  before running).
* F_min is the lowest passing level on a descending grid
  [36, 28, 20, 14, 10, 7, 5, 3.5] N.

F_max in simulation is a declared material parameter, not a measurement, so
the deliverable is the window W(crush) = crush - F_min for candidate crush
levels, against the ACCEPTANCE GATE from Phase 1b: a window is usable by the
global calibration only if W exceeds ~20 N (2x the 9.2 N worst-case error
budget). Crush candidates below that line require per-width calibration or a
mechanical redesign (which would be configuration v2 and re-characterised).

Physics reference, stated in advance: holding a 50 g block against gravity at
pad friction mu = 1 needs roughly m*g/(2*mu) ~ 0.25 N; the measured F_min is
expected to sit far above that because the 3.7 s trajectory still carries an
8 N force excursion and the pads are 1 mm lines, so retention is governed by
the excursion and the torque about the pinch line, not by static friction.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from characterize_delta import find_contact, settle  # noqa: E402

from so101_bench.expert import HOME, ScriptedExpert  # noqa: E402
from so101_bench.scene import (  # noqa: E402
    JAW_OPEN,
    JAW_SHUT,
    RAD_PER_TICK,
    TABLE_TOP,
    DomainRandomization,
    PickScene,
)

CONTROL_HZ = 30.0
FORCE_LEVELS = [36.0, 28.0, 20.0, 14.0, 10.0, 7.0, 5.0, 3.5]
REPS = 5
PASS_AT = 4
TRANSPORT_S = 3.7
HOLD_S = 1.0
RETAIN_MM = 5.0
CRUSH_CANDIDATES = [60.0, 80.0, 100.0, 120.0]
BUDGET_N = 20.0          # 2x the 9.2 N worst-case estimation error


def seat_to_force(scene, expert, target_n, xy):
    """Place, seat, and bring the grip to target_n (setup uses oracle force)."""
    import mujoco

    blk = scene.reset(block_xy=xy, block_yaw=0.0)
    scene.model.geom_size[scene.ids.block_geom] = [0.0085, 0.018, 0.015]
    mujoco.mj_forward(scene.model, scene.data)
    yaw, _ = expert.choose_grasp_yaw(blk, 0.0)
    tool = scene.open_gap_local.copy()
    q_h, _ = scene.solve_ik(blk + [0, 0, 0.075], HOME, yaw=yaw, tool_local=tool)
    q, _ = scene.solve_ik(
        np.array([blk[0], blk[1], TABLE_TOP + 0.0316]), q_h, yaw=yaw, tool_local=tool
    )
    for a, b in ((HOME, q_h), (q_h, q)):
        for f in np.linspace(0, 1, 40):
            scene.hold(np.concatenate([a + (b - a) * f, [JAW_OPEN]]))
    settle(scene, q, JAW_OPEN, 10)
    jaw = find_contact(scene, q)
    if jaw is None:
        return None, None
    for _ in range(160):
        err = target_n - scene.grip_force()
        if abs(err) <= 1.5:
            break
        jaw = float(np.clip(jaw - np.sign(err) * RAD_PER_TICK, JAW_SHUT, JAW_OPEN))
        scene.hold(np.concatenate([q, [jaw]]), 2)
    settle(scene, q, jaw, 8)
    return q, jaw


def disturb(scene, q, jaw):
    """The fixed 3.7 s trajectory plus 1 s hold, jaw frozen. True if retained."""
    rel0 = scene.block_pos() - scene.tcp()
    q1, _ = scene.solve_ik(np.array([-0.05, -0.30, 0.18]), q, yaw=0.0)
    frames = int(round(TRANSPORT_S * CONTROL_HZ))
    worst = 0.0
    for k in range(frames):
        f = 0.5 * (1 - np.cos(np.pi * (k + 1) / frames))
        scene.hold(np.concatenate([q + (q1 - q) * f, [jaw]]))
        worst = max(worst, float(np.linalg.norm((scene.block_pos() - scene.tcp()) - rel0)))
    for _ in range(int(HOLD_S * CONTROL_HZ)):
        scene.hold(np.concatenate([q1, [jaw]]))
        worst = max(worst, float(np.linalg.norm((scene.block_pos() - scene.tcp()) - rel0)))
    return worst * 1000 <= RETAIN_MM, worst * 1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=str, default="results/phase2_window.json")
    args = parser.parse_args()

    scene = PickScene(seed=0, randomise=DomainRandomization.off())
    expert = ScriptedExpert(scene)
    rng = np.random.default_rng(0)

    levels = []
    print(f"{'force N':>8} {'retained':>9} {'achieved N':>11} {'worst slip mm':>14}")
    f_min = None
    for level in FORCE_LEVELS:
        kept, achieved, slips = 0, [], []
        for _ in range(REPS):
            xy = (0.10 + rng.uniform(-0.002, 0.002), -0.25 + rng.uniform(-0.002, 0.002))
            q, jaw = seat_to_force(scene, expert, level, xy)
            if q is None:
                slips.append(float("nan"))
                continue
            achieved.append(scene.grip_force())
            ok, slip = disturb(scene, q, jaw)
            kept += ok
            slips.append(slip)
        passed = kept >= PASS_AT
        levels.append(
            dict(target_n=level, kept=kept, reps=REPS, passed=passed,
                 achieved_n=float(np.mean(achieved)) if achieved else None,
                 worst_slip_mm=float(np.nanmax(slips)) if slips else None)
        )
        if passed:
            f_min = level
        print(f"{level:8.1f} {kept:6d}/{REPS} "
              f"{levels[-1]['achieved_n'] if achieved else float('nan'):11.1f} "
              f"{levels[-1]['worst_slip_mm']:14.2f}", flush=True)

    print("\n=== safe-grip window, configuration v1, 3.7 s protocol ===")
    if f_min is None:
        print("NO force level retained the block; the task is infeasible as built.")
    else:
        print(f"F_min = {f_min:.1f} N  (lowest level retaining >= {PASS_AT}/{REPS})")
        print(f"{'crush cand.':>12} {'W = crush - F_min':>18} {'usable (W >= 20 N)':>19}")
        for crush in CRUSH_CANDIDATES:
            w = crush - f_min
            print(f"{crush:12.0f} {w:18.1f} {'YES' if w >= BUDGET_N else 'no':>19}")

    with open(args.json, "w") as fh:
        json.dump(dict(levels=levels, f_min_n=f_min,
                       windows={str(c): c - f_min for c in CRUSH_CANDIDATES}
                       if f_min else None), fh, indent=2)
    print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
