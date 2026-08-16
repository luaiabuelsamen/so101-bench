"""Phase 1b: validity and transition audit for the tracking-error signal.

    python scripts/phase1b_validity.py --json results/phase1b.json

Three experiments the Phase 1 calibration does not cover, each demanded by
audit before any task benchmark is interpretable:

A. SEATING DETECTION, evaluated as a classifier. Phase 1 claimed "seating is
   detectable from the same channel" without evidence. Here ground truth is
   defined INDEPENDENTLY of delta -- the block is seated when it is in
   simultaneous contact with at least one fixed-jaw pad and one moving-jaw pad
   (the physical definition of a pinch) -- and two bus-observable detectors are
   evaluated against it on a grid of widths, offsets, yaw errors, closing
   speeds and observation quanta, with thresholds FROZEN in advance (the
   expert's existing constants; nothing is tuned on any split). Jammed and
   escaped objects are separate outcome classes, not folded into seating.

B. ERROR IN CONTROL-RELEVANT TERMS. RMSE alone under-reports risk; the
   staircases are rerun and the pooled Phase 1 calibration (2.00 N/count) is
   scored on held-out widths with bias, p95 and WORST-CASE absolute error,
   per cell. Margin-normalisation is deferred to Phase 2, which measures the
   safe window; errors here are reported in newtons.

C. IN-GRASP TRANSPORT CONFOUND. The Phase 1 motion test used an empty jaw,
   which has different inertia and constraints from a loaded one. Here a block
   is seated at a known force, the jaw command is FROZEN, and the actual
   transport trajectory is run at three speeds while delta, true force, slip
   (block displacement in the jaw frame) and arm acceleration are logged
   synchronously. Reported per speed: how far delta departs from what the
   calibration predicts for the true force (the dynamic deviation, in counts
   and newtons), correlation of that residual with arm acceleration, and slip.

All latencies are reported in seconds (control rate 30 Hz) as well as frames.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from characterize_delta import find_contact, place_block_in_jaw, settle  # noqa: E402

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
WIDTHS_TRAIN = [0.0076, 0.0085, 0.0095]
WIDTHS_TEST = [0.0080, 0.0090]
YAW_ERRORS = [0.0, 0.14, 0.35]          # rad: aligned, 8 deg, 20 deg
CLOSE_SPEEDS = [0.0015, 0.003, 0.006]   # rad per control frame
QUANTA = [1.0, 8.0]

#: Frozen detector constants -- the expert's, declared before evaluation.
STALL_FRACTION = 0.3
STALL_WARMUP = 4
JUMP_MARGIN = 6.0


def pad_sides(scene):
    m = scene.model
    fixed = {m.geom(f"fixed_jaw_pad_{i}").id for i in range(1, 5)}
    moving = {m.geom(f"moving_jaw_pad_{i}").id for i in range(1, 5)}
    return fixed, moving


def contact_sides(scene, fixed, moving):
    """(touching_fixed, touching_moving) for the block, from the contact list."""
    d = scene.data
    tf = tm = False
    for i in range(d.ncon):
        c = d.contact[i]
        pair = {c.geom1, c.geom2}
        if scene.ids.block_geom not in pair:
            continue
        if pair & fixed:
            tf = True
        if pair & moving:
            tm = True
    return tf, tm


# ---------------------------------------------------------------- experiment A


def run_closing_pass(scene, expert, half_w, yaw_err, step_rad, quantum):
    """One instrumented close. Returns per-frame log and the outcome label."""
    scene.obs_quantum = quantum
    blk = scene.reset(block_xy=(0.10, -0.25), block_yaw=0.0)
    scene.model.geom_size[scene.ids.block_geom] = [half_w, 0.018, 0.015]
    import mujoco

    mujoco.mj_forward(scene.model, scene.data)
    yaw = yaw_err                      # commanded heading error vs true block yaw
    tool = scene.open_gap_local.copy()
    q_h, _ = scene.solve_ik(blk + [0, 0, 0.075], HOME, yaw=yaw, tool_local=tool)
    q, _ = scene.solve_ik(
        np.array([blk[0], blk[1], TABLE_TOP + 0.0316]), q_h, yaw=yaw, tool_local=tool
    )
    for a, b in ((HOME, q_h), (q_h, q)):
        for f in np.linspace(0, 1, 40):
            scene.hold(np.concatenate([a + (b - a) * f, [JAW_OPEN]]))
    settle(scene, q, JAW_OPEN, 10)

    fixed, moving = pad_sides(scene)
    yaw0 = scene.block_yaw()
    p0 = scene.block_pos()
    jaw = JAW_OPEN
    log = []
    while jaw > JAW_SHUT:
        jaw = max(JAW_SHUT, jaw - step_rad)
        scene.hold(np.concatenate([q, [jaw]]))
        tf, tm = contact_sides(scene, fixed, moving)
        log.append(
            {
                "jaw": jaw,
                "meas": float(scene.state_ticks()[5]),
                "delta": float(abs(scene.delta()[5])),
                "seated": bool(tf and tm),
                "force": scene.grip_force(),
            }
        )
    drift = float(np.linalg.norm(scene.block_pos()[:2] - p0[:2]))
    spun = abs(scene.block_yaw() - yaw0) > 0.26          # 15 degrees
    ever_seated = any(r["seated"] for r in log)
    if ever_seated and spun:
        outcome = "jammed"
    elif ever_seated:
        outcome = "seated"
    elif drift > 0.025:
        outcome = "escaped"
    else:
        outcome = "missed"
    return log, outcome


def detector_stall(log, step_rad, quantum):
    """Frozen windowed stall detector on the quantised measurement."""
    step_counts = step_rad / RAD_PER_TICK
    window = max(3, int(np.ceil(2.0 * quantum / step_counts)))
    hist: deque[float] = deque(maxlen=window + 1)
    for k, row in enumerate(log):
        hist.append(row["meas"])
        if k >= STALL_WARMUP and len(hist) == window + 1:
            if abs(hist[-1] - hist[0]) < STALL_FRACTION * window * step_counts:
                return k
    return None


def detector_jump(log, quantum):
    """Frozen single-frame jump detector on |delta| vs its running median."""
    recent: deque[float] = deque(maxlen=8)
    thresh = max(JUMP_MARGIN, quantum)
    for k, row in enumerate(log):
        base = float(np.median(recent)) if recent else row["delta"]
        if row["delta"] - base >= thresh:
            return k
        recent.append(row["delta"])
    return None


def score_detector(fire, log, outcome):
    """Classify one pass: TP / FP(early) / FN / TN, with latency for TPs."""
    seat_frame = next((k for k, r in enumerate(log) if r["seated"]), None)
    if outcome in ("escaped", "missed"):
        return ("fp" if fire is not None else "tn"), None
    if fire is None:
        return "fn", None
    if seat_frame is not None and fire < seat_frame - 3:
        return "fp_early", None
    lat = None if seat_frame is None else fire - seat_frame
    return "tp", lat


def experiment_a(scene, expert):
    rows = []
    for half_w in WIDTHS_TRAIN + WIDTHS_TEST:
        for yaw_err in YAW_ERRORS:
            for step in CLOSE_SPEEDS:
                for quantum in QUANTA:
                    log, outcome = run_closing_pass(
                        scene, expert, half_w, yaw_err, step, quantum
                    )
                    for name, fire in (
                        ("stall", detector_stall(log, step, quantum)),
                        ("jump", detector_jump(log, quantum)),
                    ):
                        verdict, lat = score_detector(fire, log, outcome)
                        rows.append(
                            {
                                "half_width": half_w,
                                "held_out": half_w in WIDTHS_TEST,
                                "yaw_err": yaw_err,
                                "step_rad": step,
                                "quantum": quantum,
                                "outcome": outcome,
                                "detector": name,
                                "verdict": verdict,
                                "latency_frames": lat,
                            }
                        )
    scene.obs_quantum = 1.0
    return rows


def summarize_a(rows):
    out = {}
    for split in ("train", "held_out"):
        for det in ("stall", "jump"):
            sel = [
                r
                for r in rows
                if r["detector"] == det and r["held_out"] == (split == "held_out")
            ]
            tp = sum(r["verdict"] == "tp" for r in sel)
            fp = sum(r["verdict"] in ("fp", "fp_early") for r in sel)
            fn = sum(r["verdict"] == "fn" for r in sel)
            tn = sum(r["verdict"] == "tn" for r in sel)
            lats = [r["latency_frames"] for r in sel if r["latency_frames"] is not None]
            out[(split, det)] = {
                "n": len(sel),
                "precision": tp / (tp + fp) if tp + fp else None,
                "recall": tp / (tp + fn) if tp + fn else None,
                "fp": fp,
                "fn": fn,
                "tn": tn,
                "latency_median_s": float(np.median(lats)) / CONTROL_HZ if lats else None,
                "latency_p95_s": float(np.percentile(lats, 95)) / CONTROL_HZ
                if lats
                else None,
            }
    return out


# ---------------------------------------------------------------- experiment B

SLOPE = 2.00      # pooled Phase 1 calibration, frozen
BIAS = 0.0


def experiment_b(scene, expert):
    from characterize_delta import staircase

    cells = []
    for half_w in WIDTHS_TEST:            # held-out only: this scores, not fits
        for offset in (0.0, 0.003):
            q = place_block_in_jaw(scene, expert, half_w, offset)
            jaw0 = find_contact(scene, q)
            if jaw0 is None:
                continue
            rows = staircase(scene, q, jaw0, 1.0)
            errs = [
                (SLOPE * r["delta"] + BIAS) - r["force"]
                for r in rows
                if r["force"] > 0.2
            ]
            if not errs:
                continue
            errs = np.array(errs)
            cells.append(
                {
                    "half_width": half_w,
                    "offset": offset,
                    "bias_n": float(np.mean(errs)),
                    "rmse_n": float(np.sqrt(np.mean(errs**2))),
                    "p95_abs_n": float(np.percentile(np.abs(errs), 95)),
                    "worst_abs_n": float(np.max(np.abs(errs))),
                    "points": int(len(errs)),
                }
            )
    return cells


# ---------------------------------------------------------------- experiment C


def experiment_c(scene, expert):
    """Seated block, frozen jaw command, real transport at three speeds."""
    out = []
    for frames in (30, 55, 110):
        q = place_block_in_jaw(scene, expert, 0.0085, 0.0)
        jaw0 = find_contact(scene, q)     # seats the block (seat-first version)
        if jaw0 is None:
            continue
        jaw = jaw0 - 15 * RAD_PER_TICK    # ~30 N seated grip
        settle(scene, q, jaw, 10)
        f_seat = scene.grip_force()
        d_seat = float(abs(scene.delta()[5]))
        blk_rel0 = scene.block_pos() - scene.tcp()

        q1, _ = scene.solve_ik(np.array([-0.05, -0.30, 0.18]), q, yaw=0.0)
        rec = {"delta": [], "force": [], "qacc": [], "slip": []}
        prev_v = np.zeros(5)
        for k in range(frames):
            f = 0.5 * (1 - np.cos(np.pi * (k + 1) / frames))
            scene.hold(np.concatenate([q + (q1 - q) * f, [jaw]]))
            v = scene.data.qvel[:5].copy()
            rec["qacc"].append(float(np.linalg.norm(v - prev_v) * CONTROL_HZ))
            prev_v = v
            rec["delta"].append(float(abs(scene.delta()[5])))
            rec["force"].append(scene.grip_force())
            rec["slip"].append(
                float(np.linalg.norm((scene.block_pos() - scene.tcp()) - blk_rel0))
            )
        delta = np.array(rec["delta"])
        force = np.array(rec["force"])
        # what the calibration would predict delta to be, given the true force
        d_pred = d_seat + (force - f_seat) / SLOPE
        resid = delta - d_pred
        qacc = np.array(rec["qacc"])
        out.append(
            {
                "frames": frames,
                "duration_s": frames / CONTROL_HZ,
                "seat_force_n": float(f_seat),
                "force_range_n": [float(force.min()), float(force.max())],
                "dyn_deviation_median_counts": float(np.median(np.abs(resid))),
                "dyn_deviation_p95_counts": float(np.percentile(np.abs(resid), 95)),
                "dyn_deviation_p95_newtons": float(
                    np.percentile(np.abs(resid), 95) * SLOPE
                ),
                "corr_delta_force": float(np.corrcoef(delta, force)[0, 1])
                if force.std() > 0
                else None,
                "corr_resid_qacc": float(np.corrcoef(resid, qacc)[0, 1])
                if qacc.std() > 0
                else None,
                "max_slip_mm": float(np.max(rec["slip"]) * 1000),
            }
        )
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=str, default="results/phase1b.json")
    args = parser.parse_args()

    scene = PickScene(seed=0, randomise=DomainRandomization.off())
    expert = ScriptedExpert(scene)

    print("A: seating detection as a classifier (thresholds frozen a priori)")
    rows_a = experiment_a(scene, expert)
    summary_a = summarize_a(rows_a)
    outcomes = {}
    for r in rows_a:
        if r["detector"] == "stall":
            outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    print(f"  pass outcomes: {outcomes}")
    for (split, det), s in summary_a.items():
        p = "-" if s["precision"] is None else f"{s['precision']:.2f}"
        r = "-" if s["recall"] is None else f"{s['recall']:.2f}"
        lm = "-" if s["latency_median_s"] is None else f"{s['latency_median_s']:.2f}s"
        lp = "-" if s["latency_p95_s"] is None else f"{s['latency_p95_s']:.2f}s"
        print(
            f"  {split:>8} {det:>6}: n={s['n']:3d} precision {p} recall {r} "
            f"fp {s['fp']} fn {s['fn']} latency med {lm} p95 {lp}"
        )

    print("\nB: held-out force error, control-relevant statistics")
    cells_b = experiment_b(scene, expert)
    for c in cells_b:
        print(
            f"  w={1000*c['half_width']:.1f}mm off={1000*c['offset']:.0f}mm: "
            f"bias {c['bias_n']:+.1f} N rmse {c['rmse_n']:.1f} N "
            f"p95 {c['p95_abs_n']:.1f} N worst {c['worst_abs_n']:.1f} N"
        )

    print("\nC: in-grasp transport confound (frozen jaw command)")
    rows_c = experiment_c(scene, expert)
    for c in rows_c:
        cd = "-" if c["corr_delta_force"] is None else f"{c['corr_delta_force']:+.2f}"
        cr = "-" if c["corr_resid_qacc"] is None else f"{c['corr_resid_qacc']:+.2f}"
        print(
            f"  {c['duration_s']:.1f}s transport: seat {c['seat_force_n']:.0f} N, "
            f"true force {c['force_range_n'][0]:.0f}-{c['force_range_n'][1]:.0f} N, "
            f"dyn dev p95 {c['dyn_deviation_p95_counts']:.1f} counts "
            f"({c['dyn_deviation_p95_newtons']:.1f} N), "
            f"corr(delta,F) {cd}, corr(resid,accel) {cr}, "
            f"slip {c['max_slip_mm']:.1f} mm"
        )

    with open(args.json, "w") as fh:
        json.dump(
            {
                "a_rows": rows_a,
                "a_summary": {f"{k[0]}/{k[1]}": v for k, v in summary_a.items()},
                "b_cells": cells_b,
                "c_transport": rows_c,
            },
            fh,
            indent=2,
        )
    print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
