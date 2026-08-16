"""Phase 1: characterise the tracking-error signal against ground-truth force.

    python scripts/characterize_delta.py --json results/delta_characterization.json

No task, no tournament, no controller. The arm is parked at a grasp pose, an
object is placed in the jaw, and the jaw command is stepped through a
loading/unloading staircase while every relevant quantity is logged
synchronously. The oracle force is ground truth; the question is what
``delta = cmd - meas`` (encoder counts, quantised) knows about it.

WHY THIS EXISTS. The project spent a week comparing task success rates between
controllers that consume this signal, which cannot establish what the signal
measures: task outcomes were confounded by grasp geometry, transport dynamics,
slip mechanics and controller sequencing. Tracking error is not automatically a
force measurement -- it also responds to servo lag, saturation, friction and
inertial load -- so the mapping F(delta) has to be measured, with its error
statistics, before any claim about "sensing" is made on top of it.

WHAT IS REPORTED, per condition cell and pooled:

  * static calibration curve  F vs delta  (loading and unloading separately)
  * hysteresis: mean |F_load(delta) - F_unload(delta)|
  * per-cell linear fit slope (N/count), bias, RMSE of F predicted from delta
  * lag: frames from force onset to first delta response at contact
  * resolution: smallest delta step observed against force noise within holds
  * the confound measurement: delta response to ARM MOTION WITH NO OBJECT,
    which is what an estimator would wrongly read as force

Held-out evaluation: fits are trained on a subset of object widths and tested
on the remaining widths, because a calibration that only interpolates its own
conditions is a lookup table, not a sensor model.
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from so101_bench.expert import HOME, ScriptedExpert
from so101_bench.scene import (
    JAW_OPEN,
    JAW_SHUT,
    RAD_PER_TICK,
    TABLE_TOP,
    DomainRandomization,
    PickScene,
)

#: Object half-widths (m). The last two are held out of every fit.
WIDTHS_TRAIN = [0.0076, 0.0085, 0.0095]
WIDTHS_TEST = [0.0080, 0.0090]
#: Lateral offset of the object from the jaw centre along the closing axis (m).
OFFSETS = [0.0, 0.003]
#: Command step per control frame during the staircase (counts).
STEP_COUNTS = [1.0, 4.0]
#: Depth of the staircase past first contact (counts).
DEPTH_COUNTS = 30


def settle(scene, q, jaw, frames=8):
    for _ in range(frames):
        scene.hold(np.concatenate([q, [jaw]]))


def place_block_in_jaw(scene, expert, half_w, offset):
    """Park the arm at the grasp pose and put the block between the pads.

    Order matters twice here, and both orderings have bitten this project
    before: the width must be set AFTER ``reset()`` (reset restores nominal
    geometry), and the approach must go through the hover waypoint -- a direct
    HOME-to-grasp joint interpolation sweeps the gripper through the block and
    knocks it away, which silently empties every cell.
    """
    blk = scene.reset(block_xy=(0.10, -0.25), block_yaw=0.0)
    scene.model.geom_size[scene.ids.block_geom] = [half_w, 0.018, 0.015]
    import mujoco
    mujoco.mj_forward(scene.model, scene.data)
    yaw, _ = expert.choose_grasp_yaw(blk, 0.0)
    tool = scene.open_gap_local.copy()
    tool[0] += offset
    q_h, _ = scene.solve_ik(blk + [0, 0, 0.075], HOME, yaw=yaw, tool_local=tool)
    q, _ = scene.solve_ik(
        np.array([blk[0], blk[1], TABLE_TOP + 0.0316]), q_h, yaw=yaw, tool_local=tool
    )
    for a, b in ((HOME, q_h), (q_h, q)):
        for f in np.linspace(0, 1, 40):
            scene.hold(np.concatenate([a + (b - a) * f, [JAW_OPEN]]))
    settle(scene, q, JAW_OPEN, 10)
    return q


def find_contact(scene, q, step_rad=0.0015, seat_newtons=6.0):
    """Close until the object is SEATED against the fixed jaw, then back off.

    First touch is not loadable contact. This gripper actuates one jaw, so
    between first touch and seating the close merely slides the object across
    the table at friction-level force: a calibration staircase started at
    first touch produced 0-4 N over its whole 30-count depth and slopes from
    -1.8 to +1.0 N/count -- it measured sliding friction, not grip. Force only
    becomes a function of jaw position once the object is pinched against the
    fixed pad, so that is where a calibration must start.
    """
    jaw = JAW_OPEN
    while jaw > JAW_SHUT:
        jaw = max(JAW_SHUT, jaw - step_rad)
        scene.hold(np.concatenate([q, [jaw]]))
        if scene.grip_force() >= seat_newtons:
            jaw = min(JAW_OPEN, jaw + 2 * RAD_PER_TICK)   # relieve the transient
            settle(scene, q, jaw, 8)
            return jaw
    return None


def staircase(scene, q, jaw0, step_counts):
    """Load then unload in encoder-count steps; log (delta, F) at each hold."""
    rows = []
    jaw = jaw0
    seq = list(np.arange(0, DEPTH_COUNTS + 1, step_counts)) + list(
        np.arange(DEPTH_COUNTS, -1, -step_counts)
    )
    half = len(seq) // 2
    for i, depth in enumerate(seq):
        jaw = max(JAW_SHUT, jaw0 - depth * RAD_PER_TICK)
        settle(scene, q, jaw, 6)          # quasi-static: read at rest
        rows.append(
            {
                "phase": "load" if i <= half else "unload",
                "depth_counts": float(depth),
                "delta": float(abs(scene.delta()[5])),
                "force": scene.grip_force(),
            }
        )
    return rows


def motion_confound(scene, expert):
    """Delta while the arm MOVES with an EMPTY jaw: pure non-contact response.

    This is the number that decides whether delta can be read as force during
    motion at all. Everything an estimator would see here is error.
    """
    scene.model.geom_size[scene.ids.block_geom] = [0.008, 0.018, 0.015]
    scene.reset(block_xy=(0.30, -0.35), block_yaw=0.0)   # block far away
    q0, _ = scene.solve_ik(np.array([0.10, -0.25, 0.20]), HOME, yaw=0.0)
    q1, _ = scene.solve_ik(np.array([-0.05, -0.30, 0.16]), q0, yaw=0.0)
    out = []
    for frames in (30, 60):               # two speeds
        vals = []
        for a, b in ((q0, q1), (q1, q0)):
            for k in range(frames):
                f = 0.5 * (1 - np.cos(np.pi * (k + 1) / frames))
                scene.hold(np.concatenate([a + (b - a) * f, [JAW_SHUT + 0.1]]))
                vals.append(abs(float(scene.delta()[5])))
        out.append(
            {"frames": frames, "delta_median": float(np.median(vals)),
             "delta_p95": float(np.percentile(vals, 95)),
             "delta_max": float(np.max(vals))}
        )
    return out


def contact_lag(scene, q, jaw0):
    """Frames between first true force and first delta response.

    The probe must begin from genuinely broken contact: a fixed 20-count
    retreat does not always release a seated block, and measuring "onset" from
    inside contact produced nonsense (negative lags). Open until the force
    actually reads zero, then approach.
    """
    jaw = jaw0
    for _ in range(120):
        if scene.grip_force() <= 0.0 and jaw > jaw0 + 6 * RAD_PER_TICK:
            break
        jaw = min(JAW_OPEN, jaw + RAD_PER_TICK)
        scene.hold(np.concatenate([q, [jaw]]))
    settle(scene, q, jaw, 8)
    base = abs(float(scene.delta()[5]))
    # Creep at one count per FOUR frames. At one count per frame the probe's
    # own following lag reaches the 2-count trigger and the measurement reads
    # its own approach, not the contact -- the third appearance of the same
    # confound in this project. At a quarter count per frame the following lag
    # is below one count and the trigger can only be contact.
    t_force = t_delta = None
    for k in range(480):
        if k % 4 == 0:
            jaw = max(JAW_SHUT, jaw - RAD_PER_TICK)
        scene.hold(np.concatenate([q, [jaw]]))
        if t_force is None and scene.grip_force() > 0.5:
            t_force = k
        if t_delta is None and abs(float(scene.delta()[5])) - base >= 2.0:
            t_delta = k
        if t_force is not None and t_delta is not None:
            break
    if t_force is None:
        return None
    return None if t_delta is None else int(t_delta - t_force)


def fit_and_score(rows_train, rows_test):
    """Linear F = a*delta + b on train; report train/test RMSE."""
    def arr(rows):
        d = np.array([r["delta"] for r in rows])
        f = np.array([r["force"] for r in rows])
        keep = f > 0.2
        return d[keep], f[keep]

    dtr, ftr = arr(rows_train)
    dte, fte = arr(rows_test)
    if len(dtr) < 4 or len(dte) < 2:
        return None
    a, b = np.polyfit(dtr, ftr, 1)
    return {
        "slope_n_per_count": float(a),
        "bias_n": float(b),
        "train_rmse_n": float(np.sqrt(np.mean((a * dtr + b - ftr) ** 2))),
        "test_rmse_n": float(np.sqrt(np.mean((a * dte + b - fte) ** 2))),
        "train_points": int(len(dtr)),
        "test_points": int(len(dte)),
        "force_range_n": [float(ftr.min()), float(ftr.max())],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=str, default="results/delta_characterization.json")
    args = parser.parse_args()

    scene = PickScene(seed=0, randomise=DomainRandomization.off())
    expert = ScriptedExpert(scene)

    cells = []
    print(f"{'width':>7} {'offset':>7} {'step':>5} {'slope':>8} {'hyst':>7} "
          f"{'pts':>4}  N-per-count / hysteresis-N")
    for half_w in WIDTHS_TRAIN + WIDTHS_TEST:
        for offset in OFFSETS:
            for step in STEP_COUNTS:
                q = place_block_in_jaw(scene, expert, half_w, offset)
                jaw0 = find_contact(scene, q)
                if jaw0 is None:
                    continue
                rows = staircase(scene, q, jaw0, step)
                load = [r for r in rows if r["phase"] == "load" and r["force"] > 0.2]
                unload = [r for r in rows if r["phase"] == "unload" and r["force"] > 0.2]
                hyst = None
                if load and unload:
                    ld = {r["depth_counts"]: r["force"] for r in load}
                    ul = {r["depth_counts"]: r["force"] for r in unload}
                    common = set(ld) & set(ul)
                    if common:
                        hyst = float(np.mean([abs(ld[c] - ul[c]) for c in common]))
                d = np.array([r["delta"] for r in load])
                f = np.array([r["force"] for r in load])
                slope = float(np.polyfit(d, f, 1)[0]) if len(d) > 3 else float("nan")
                lag = contact_lag(scene, q, jaw0)
                cells.append(
                    {"half_width": half_w, "offset": offset, "step_counts": step,
                     "held_out": half_w in WIDTHS_TEST, "rows": rows,
                     "slope_n_per_count": slope, "hysteresis_n": hyst,
                     "contact_lag_frames": lag}
                )
                print(f"{1000*half_w:7.1f} {1000*offset:7.1f} {step:5.0f} "
                      f"{slope:8.2f} {str(hyst)[:7]:>7} {len(load):4d}", flush=True)

    train_rows = [r for c in cells if not c["held_out"] for r in c["rows"]]
    test_rows = [r for c in cells if c["held_out"] for r in c["rows"]]
    fit = fit_and_score(train_rows, test_rows)

    confound = motion_confound(scene, expert)
    slopes = [c["slope_n_per_count"] for c in cells if np.isfinite(c["slope_n_per_count"])]
    lags = [c["contact_lag_frames"] for c in cells if c["contact_lag_frames"] is not None]

    print("\n=== summary ===")
    if slopes:
        spread = np.max(slopes) / max(np.min(slopes), 1e-9)
        print(f"slope across cells: {np.min(slopes):.2f} .. {np.max(slopes):.2f}"
              f" N/count (median {np.median(slopes):.2f}) -- {spread:.1f}x spread")
    if fit:
        print(f"single linear fit: {fit['slope_n_per_count']:.2f} N/count, "
              f"train RMSE {fit['train_rmse_n']:.1f} N, "
              f"HELD-OUT RMSE {fit['test_rmse_n']:.1f} N "
              f"over {fit['force_range_n'][0]:.0f}-{fit['force_range_n'][1]:.0f} N")
    print(f"contact lag: {lags} frames")
    print("non-contact motion response (empty jaw):")
    for c in confound:
        print(f"  {c['frames']}-frame moves: median {c['delta_median']:.1f}, "
              f"p95 {c['delta_p95']:.1f}, max {c['delta_max']:.1f} counts "
              f"-- this is pure error for any force reading taken in motion")

    out = {"cells": [{k: v for k, v in c.items() if k != "rows"} for c in cells],
           "pooled_fit": fit, "motion_confound": confound}
    with open(args.json, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
