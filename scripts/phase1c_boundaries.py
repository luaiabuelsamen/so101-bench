"""Phase 1c: adversarial negatives for the seating detector + dynamic boundary.

    python scripts/phase1c_boundaries.py --json results/phase1c.json

Two measurements, both required before the envelope is numerical rather than
narrative. Configuration is v1 (stock pads, stock scene); no contact mechanics
are changed here or between this and any later evaluation.

PART 1 -- ADVERSARIAL NEGATIVES. Phase 1b evaluated the stall detector on 90
passes that all ended seated, so its precision was measured on a distribution
with no negatives. Here the detector (thresholds unchanged) runs against
states constructed to be negative or degenerate: empty closure, object outside
the gap on either side, corner/edge contact, diagonal yaw, over-width objects
that jam under the fingertip, under- and over-height objects. Ground truth per
frame is delta-independent (pad contact states and block pose). A full
confusion matrix and per-state latency are reported. A detector fire at the
jaw's own travel limit is counted separately: a real deployment masks the
known hard-stop angle, but the mask is reported, not silently applied.

PART 2 -- DYNAMIC BOUNDARY. Phase 1b sampled three transport speeds; the
envelope needs the boundary, not three points. Pre-declared duration grid:
0.7, 1.0, 1.4, 1.8, 2.5, 3.0, 3.7, 5.0 s over the same fixed trajectory,
frozen jaw command, seated at ~36 N. Pre-declared acceptance criteria for the
operating tier (ALL must hold):

    C1  no contact loss:      min true force > 5 N
    C2  slip < 1.0 mm
    C3  delta residual p95 <= 4 counts (8 N -- inside the static error budget)
    C4  true-force excursion within +-15 N of the seated value

The published tier is the FASTEST duration meeting all four. Nothing about
the criteria was chosen after seeing the sweep.
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

from so101_bench.expert import (
    HOME,  # noqa: E402
    ScriptedExpert,  # noqa: E402
)
from so101_bench.scene import (  # noqa: E402
    JAW_OPEN,
    JAW_SHUT,
    RAD_PER_TICK,
    TABLE_TOP,
    DomainRandomization,
    PickScene,
)

CONTROL_HZ = 30.0
STALL_FRACTION = 0.3
STALL_WARMUP = 4
CLOSE_SPEED = 0.003
#: Detector fires within this many counts of the jaw's hard stop are attributed
#: to the travel limit, not to an object.
HARD_STOP_COUNTS = 8

#: Adversarial states: name -> (block placement + geometry overrides).
STATES = [
    ("seated_control", dict()),                                  # positive control
    ("empty", dict(empty=True)),
    ("outside_fixed", dict(tool_dx=+0.014)),
    ("outside_moving", dict(tool_dx=-0.020)),
    ("corner_contact", dict(perp=0.015)),
    ("mostly_missing", dict(perp=0.025)),
    ("yaw_45deg", dict(yaw=0.785)),
    ("too_wide", dict(half_w=0.013)),
    ("too_short", dict(half_h=0.006)),
    ("too_tall", dict(half_h=0.030)),
    ("narrow", dict(half_w=0.004)),
]


def setup_state(scene, spec):
    """Place the arm and object for one adversarial state. Returns arm q."""
    import mujoco

    xy = (0.30, -0.35) if spec.get("empty") else (0.10, -0.25)
    blk = scene.reset(block_xy=xy, block_yaw=0.0)
    scene.model.geom_size[scene.ids.block_geom] = [
        spec.get("half_w", 0.0085),
        0.018,
        spec.get("half_h", 0.015),
    ]
    mujoco.mj_forward(scene.model, scene.data)
    target = np.array([0.10, -0.25 + spec.get("perp", 0.0), 0.0])
    if not spec.get("empty"):
        target[:2] = blk[:2] + [0, spec.get("perp", 0.0)]
    yaw = spec.get("yaw", 0.0)
    tool = scene.open_gap_local.copy()
    tool[0] += spec.get("tool_dx", 0.0)
    q_h, _ = scene.solve_ik(
        np.array([target[0], target[1], TABLE_TOP + 0.09]), HOME, yaw=yaw,
        tool_local=tool,
    )
    q, _ = scene.solve_ik(
        np.array([target[0], target[1], TABLE_TOP + 0.0316]), q_h, yaw=yaw,
        tool_local=tool,
    )
    for a, b in ((HOME, q_h), (q_h, q)):
        for f in np.linspace(0, 1, 40):
            scene.hold(np.concatenate([a + (b - a) * f, [JAW_OPEN]]))
    settle(scene, q, JAW_OPEN, 10)
    return q


def pad_sides(scene):
    m = scene.model
    return (
        {m.geom(f"fixed_jaw_pad_{i}").id for i in range(1, 5)},
        {m.geom(f"moving_jaw_pad_{i}").id for i in range(1, 5)},
    )


def contact_sides(scene, fixed, moving):
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


def closing_pass(scene, q):
    fixed, moving = pad_sides(scene)
    yaw0, p0 = scene.block_yaw(), scene.block_pos()
    jaw, log = JAW_OPEN, []
    while jaw > JAW_SHUT:
        jaw = max(JAW_SHUT, jaw - CLOSE_SPEED)
        scene.hold(np.concatenate([q, [jaw]]))
        tf, tm = contact_sides(scene, fixed, moving)
        log.append(
            dict(jaw=jaw, meas=float(scene.state_ticks()[5]),
                 seated=bool(tf and tm), one_pad=bool(tf ^ tm),
                 force=scene.grip_force())
        )
    drift = float(np.linalg.norm(scene.block_pos()[:2] - p0[:2]))
    spun = abs(scene.block_yaw() - yaw0) > 0.26
    R = scene.data.xmat[scene.ids.block].reshape(3, 3)
    tilted = float(R[2, 2]) < 0.95
    ever = any(r["seated"] for r in log)
    if ever and (spun or tilted):
        label = "jammed"
    elif ever:
        label = "seated"
    elif drift > 0.025:
        label = "escaped"
    else:
        label = "no_contact"
    return log, label


def stall_fire(log):
    step_counts = CLOSE_SPEED / RAD_PER_TICK
    window = max(3, int(np.ceil(2.0 / step_counts)))
    hist: deque[float] = deque(maxlen=window + 1)
    for k, row in enumerate(log):
        hist.append(row["meas"])
        if k >= STALL_WARMUP and len(hist) == window + 1:
            if abs(hist[-1] - hist[0]) < STALL_FRACTION * window * step_counts:
                at_stop = row["jaw"] <= JAW_SHUT + HARD_STOP_COUNTS * RAD_PER_TICK
                return k, at_stop
    return None, False


def part1(scene):
    rows = []
    print(f"{'state':>16} {'truth':>11} {'fire':>6} {'at_stop':>8} "
          f"{'verdict':>9} {'latency':>8}")
    for name, spec in STATES:
        q = setup_state(scene, spec)
        log, label = closing_pass(scene, q)
        fire, at_stop = stall_fire(log)
        seat_frame = next((k for k, r in enumerate(log) if r["seated"]), None)
        if label in ("seated", "jammed"):
            # jammed counts as a state the detector SHOULD NOT report as a
            # clean seat; firing on a jam is a false positive for grasp-ready.
            if label == "seated":
                if fire is None:
                    verdict = "fn"
                elif seat_frame is not None and fire < seat_frame - 3:
                    verdict = "fp_early"
                else:
                    verdict = "tp"
            else:
                verdict = "fp_jam" if (fire is not None and not at_stop) else "tn_jam"
        else:
            if fire is None or at_stop:
                verdict = "tn" if fire is None else "tn_hardstop"
            else:
                verdict = "fp"
        lat = None
        if verdict == "tp" and seat_frame is not None and fire is not None:
            lat = (fire - seat_frame) / CONTROL_HZ
        rows.append(
            dict(state=name, truth=label, fire_frame=fire, fired_at_stop=at_stop,
                 verdict=verdict, latency_s=lat)
        )
        print(f"{name:>16} {label:>11} {str(fire):>6} {str(at_stop):>8} "
              f"{verdict:>9} {('%.2fs' % lat) if lat is not None else '-':>8}")
    return rows


def part2(scene, expert):
    durations = [0.7, 1.0, 1.4, 1.8, 2.5, 3.0, 3.7, 5.0]
    out = []
    print(f"\n{'dur':>5} {'Fmin':>6} {'Fexc':>6} {'slip':>6} {'resid p95':>10} "
          f"{'C1':>3} {'C2':>3} {'C3':>3} {'C4':>3}")
    for dur in durations:
        frames = int(round(dur * CONTROL_HZ))
        q = place_block_in_jaw(scene, expert, 0.0085, 0.0)
        jaw0 = find_contact(scene, q)
        if jaw0 is None:
            continue
        jaw = jaw0 - 15 * RAD_PER_TICK
        settle(scene, q, jaw, 10)
        f_seat, d_seat = scene.grip_force(), float(abs(scene.delta()[5]))
        rel0 = scene.block_pos() - scene.tcp()
        q1, _ = scene.solve_ik(np.array([-0.05, -0.30, 0.18]), q, yaw=0.0)
        force, delta, slip = [], [], []
        for k in range(frames):
            f = 0.5 * (1 - np.cos(np.pi * (k + 1) / frames))
            scene.hold(np.concatenate([q + (q1 - q) * f, [jaw]]))
            force.append(scene.grip_force())
            delta.append(float(abs(scene.delta()[5])))
            slip.append(float(np.linalg.norm((scene.block_pos() - scene.tcp()) - rel0)))
        force, delta = np.array(force), np.array(delta)
        resid = np.abs(delta - (d_seat + (force - f_seat) / 2.0))
        c1 = bool(force.min() > 5.0)
        c2 = bool(max(slip) < 0.001)
        c3 = bool(np.percentile(resid, 95) <= 4.0)
        c4 = bool(np.max(np.abs(force - f_seat)) <= 15.0)
        out.append(
            dict(duration_s=dur, seat_n=float(f_seat), min_force_n=float(force.min()),
                 excursion_n=float(np.max(np.abs(force - f_seat))),
                 slip_mm=float(1000 * max(slip)),
                 resid_p95_counts=float(np.percentile(resid, 95)),
                 resid_worst_counts=float(resid.max()),
                 c1=c1, c2=c2, c3=c3, c4=c4, passes=bool(c1 and c2 and c3 and c4))
        )
        r = out[-1]
        print(f"{dur:5.1f} {r['min_force_n']:6.1f} {r['excursion_n']:6.1f} "
              f"{r['slip_mm']:6.2f} {r['resid_p95_counts']:10.1f} "
              f"{'Y' if c1 else 'n':>3} {'Y' if c2 else 'n':>3} "
              f"{'Y' if c3 else 'n':>3} {'Y' if c4 else 'n':>3}", flush=True)
    tier = next((r["duration_s"] for r in out if r["passes"]), None)
    print(f"\noperating tier (fastest passing all four criteria): "
          f"{tier if tier is not None else 'NONE PASSES'} s")
    return out, tier


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=str, default="results/phase1c.json")
    args = parser.parse_args()
    scene = PickScene(seed=0, randomise=DomainRandomization.off())
    expert = ScriptedExpert(scene)
    print("PART 1: adversarial negatives, frozen stall detector")
    rows1 = part1(scene)
    rows2, tier = part2(scene, expert)
    with open(args.json, "w") as fh:
        json.dump(dict(adversarial=rows1, boundary=rows2, operating_tier_s=tier),
                  fh, indent=2)
    print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
