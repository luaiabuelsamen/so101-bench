"""Final Clamp / Delta / Oracle comparison under the validated protocol.

    python scripts/tournament_final.py --json results/tournament_final.json

THE QUESTION. Given the same task, mechanics, motion protocol and crush
threshold: does seated delta feedback reduce crush/drop failures relative to
fixed clamping, and how close does it get to oracle-force feedback?

EVERYTHING BELOW IS FROZEN BEFORE THE FIRST SCORED EPISODE.

Configuration: v1 (stock pads), unchanged since characterisation.

Object distribution: half-width uniform in [7.6, 9.5] mm (the calibrated
range), xy jitter +-5 mm, yaw +-10 deg (within the seatable envelope; jam
poses are excluded by design and the detector's 0.78 adversarial precision is
reported alongside, not averaged in). Nominal mass and friction.

Motion: the validated 3.7 s transport trajectory + 1.0 s hold. Same approach,
descend, closure speed (0.003 rad/frame), seating logic (frozen stall
detector), regulation law (one-count steps, relief ceiling seat+10 counts),
and timeouts for all three conditions.

Grip targets (guard-banded, from the audit's formula):
    F_target(F_max) = (F_min + (F_max - 9.2 - 8.0)) / 2
                    = (20 + F_max - 17.2) / 2
so 31.4 / 41.4 / 51.4 / 61.4 N for F_max = 60 / 80 / 100 / 120 N. The interval
is non-empty at every threshold, so all four are inside the tested capability
envelope. Delta and Oracle have ZERO tuned parameters: their setpoint comes
from this formula, with Delta reading force through the frozen calibration
F_est = 2.00 N/count. Clamp's single knob (counts closed past seat) is tuned
on a development set of 10 instances per threshold over the declared grid
{2, 6, 10, 16, 24} -- the tuning budget therefore favours the BASELINE.

Outcomes: crush = any frame with true force > F_max after grip start;
drop = tool-frame displacement >= 5 mm; seat_fail = no valid stall detection
within 300 frames (fires at the jaw hard stop count as invalid); success =
seated, retained, never crushed.

Evaluation: 30 held-out instances (seeds 100-129) x 4 thresholds x 3
conditions, PAIRED -- each instance presents the identical object and pose to
all three conditions. Wilson 95% intervals on rates; paired win/loss counts
for delta vs clamp.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from so101_bench.expert import HOME  # noqa: E402
from so101_bench.scene import (  # noqa: E402
    JAW_OPEN,
    JAW_SHUT,
    RAD_PER_TICK,
    TABLE_TOP,
    DomainRandomization,
    PickScene,
)

CONTROL_HZ = 30.0
CLOSE_SPEED = 0.003
STALL_FRACTION = 0.3
STALL_WARMUP = 4
SEAT_TIMEOUT = 300
HARD_STOP_COUNTS = 8
SLOPE = 2.00                      # frozen calibration, N per count
F_MIN = 20.0
GUARD = 17.2                      # 9.2 N estimation + 8.0 N transient margin
CRUSH_LEVELS = [60.0, 80.0, 100.0, 120.0]
CLAMP_GRID = [2, 6, 10, 16, 24]   # counts past seat, tuned on dev
DEV_SEEDS = list(range(10))
EVAL_SEEDS = list(range(100, 130))
TRANSPORT_S = 3.7
HOLD_S = 1.0
RETAIN_MM = 5.0
REG_ITERS = 160
JAW_CEILING_COUNTS = 10.0


def f_target(f_max: float) -> float:
    return (F_MIN + (f_max - GUARD)) / 2.0


def instance(seed: int) -> dict:
    rng = np.random.default_rng(seed)
    return dict(
        half_w=float(rng.uniform(0.0076, 0.0095)),
        x=float(0.10 + rng.uniform(-0.005, 0.005)),
        y=float(-0.25 + rng.uniform(-0.005, 0.005)),
        yaw=float(rng.uniform(-0.175, 0.175)),
    )


def setup(scene: PickScene, inst: dict):
    import mujoco

    blk = scene.reset(block_xy=(inst["x"], inst["y"]), block_yaw=inst["yaw"])
    scene.model.geom_size[scene.ids.block_geom] = [inst["half_w"], 0.018, 0.015]
    mujoco.mj_forward(scene.model, scene.data)
    tool = scene.open_gap_local.copy()
    yaw = inst["yaw"]
    q_h, _ = scene.solve_ik(blk + [0, 0, 0.075], HOME, yaw=yaw, tool_local=tool)
    q, _ = scene.solve_ik(
        np.array([blk[0], blk[1], TABLE_TOP + 0.0316]), q_h, yaw=yaw, tool_local=tool
    )
    for a, b in ((HOME, q_h), (q_h, q)):
        for f in np.linspace(0, 1, 40):
            scene.hold(np.concatenate([a + (b - a) * f, [JAW_OPEN]]))
    for _ in range(10):
        scene.hold(np.concatenate([q, [JAW_OPEN]]))
    return q


def seat(scene: PickScene, q) -> tuple[float | None, bool]:
    """Shared frozen seating. Returns (jaw_angle, valid)."""
    step_counts = CLOSE_SPEED / RAD_PER_TICK
    window = max(3, int(np.ceil(2.0 * scene.obs_quantum / step_counts)))
    hist: deque[float] = deque(maxlen=window + 1)
    hist.append(float(scene.state_ticks()[5]))
    jaw = JAW_OPEN
    for k in range(SEAT_TIMEOUT):
        if jaw <= JAW_SHUT:
            return None, False
        jaw = max(JAW_SHUT, jaw - CLOSE_SPEED)
        scene.hold(np.concatenate([q, [jaw]]))
        hist.append(float(scene.state_ticks()[5]))
        if k >= STALL_WARMUP and len(hist) == window + 1:
            if abs(hist[-1] - hist[0]) < STALL_FRACTION * window * step_counts:
                if jaw <= JAW_SHUT + HARD_STOP_COUNTS * RAD_PER_TICK:
                    return None, False
                return jaw, True
    return None, False


def episode(scene: PickScene, inst: dict, condition: str, f_max: float,
            clamp_extra: int) -> dict:
    q = setup(scene, inst)
    jaw, ok = seat(scene, q)
    if not ok:
        return dict(outcome="seat_fail")
    seat_jaw = jaw
    ceiling = seat_jaw + JAW_CEILING_COUNTS * RAD_PER_TICK
    target = f_target(f_max)

    if condition == "clamp":
        measure = None
        for _ in range(clamp_extra):
            jaw = max(JAW_SHUT, jaw - RAD_PER_TICK)
            scene.hold(np.concatenate([q, [jaw]]), 2)
    else:
        if condition == "delta":
            def measure() -> float:
                return SLOPE * abs(float(scene.delta()[5]))
        else:
            measure = scene.grip_force
        for _ in range(REG_ITERS):
            err = target - measure()
            if abs(err) <= SLOPE:                     # one-count deadband
                break
            jaw = float(np.clip(jaw - np.sign(err) * RAD_PER_TICK, JAW_SHUT, ceiling))
            scene.hold(np.concatenate([q, [jaw]]), 2)
    for _ in range(8):
        scene.hold(np.concatenate([q, [jaw]]))

    rel0 = scene.block_pos() - scene.tcp()
    q1, _ = scene.solve_ik(np.array([-0.05, -0.30, 0.18]), q, yaw=0.0)
    frames = int(round(TRANSPORT_S * CONTROL_HZ)) + int(round(HOLD_S * CONTROL_HZ))
    tframes = int(round(TRANSPORT_S * CONTROL_HZ))
    force_log, est_err, disp_log = [], [], []
    for k in range(frames):
        if k < tframes:
            f = 0.5 * (1 - np.cos(np.pi * (k + 1) / tframes))
            arm = q + (q1 - q) * f
        else:
            arm = q1
        if measure is not None:
            err = target - measure()
            if abs(err) > SLOPE:
                step = (3.0 if err < 0 else 1.0) * RAD_PER_TICK
                jaw = float(np.clip(jaw - np.sign(err) * step, JAW_SHUT, ceiling))
        scene.hold(np.concatenate([arm, [jaw]]))
        truth = scene.grip_force()
        force_log.append(truth)
        disp_log.append(
            float(np.linalg.norm((scene.block_pos() - scene.tcp()) - rel0))
        )
        if condition == "delta":
            est_err.append(abs(SLOPE * abs(float(scene.delta()[5])) - truth))
        elif condition == "oracle":
            est_err.append(abs(truth - target))

    force = np.array(force_log)
    disp = np.array(disp_log)
    crushed = bool((force > f_max).any())
    dropped = bool(disp.max() * 1000 >= RETAIN_MM)
    return dict(
        outcome="crush" if crushed else ("drop" if dropped else "success"),
        crushed=crushed,
        dropped=dropped,
        force_mean=float(force.mean()),
        force_p95=float(np.percentile(force, 95)),
        force_max=float(force.max()),
        force_min_transport=float(force[:tframes].min()),
        frac_above=float((force > f_max).mean()),
        final_disp_mm=float(disp[-1] * 1000),
        est_err_mean=float(np.mean(est_err)) if est_err else None,
        est_err_p95=float(np.percentile(est_err, 95)) if est_err else None,
    )


def wilson(k: int, n: int) -> tuple[float, float]:
    if n == 0:
        return 0.0, 0.0
    z = 1.96
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=str, default="results/tournament_final.json")
    args = parser.parse_args()
    scene = PickScene(seed=0, randomise=DomainRandomization.off())

    # ---- dev: tune the clamp's single knob per threshold (equal-budget note:
    # delta and oracle have zero tuned parameters; the budget favours clamp)
    clamp_choice = {}
    print("dev tuning (clamp counts past seat), 10 instances per threshold")
    for f_max in CRUSH_LEVELS:
        best, best_s = CLAMP_GRID[0], -1
        for extra in CLAMP_GRID:
            wins = sum(
                episode(scene, instance(s), "clamp", f_max, extra)["outcome"]
                == "success"
                for s in DEV_SEEDS
            )
            if wins > best_s:
                best, best_s = extra, wins
        clamp_choice[f_max] = best
        print(f"  F_max {f_max:.0f}: clamp_extra = {best} ({best_s}/10 dev)", flush=True)

    # ---- held-out evaluation, paired
    rows = []
    for f_max in CRUSH_LEVELS:
        for s in EVAL_SEEDS:
            inst = instance(s)
            for cond in ("clamp", "delta", "oracle"):
                r = episode(scene, inst, cond, f_max, clamp_choice[f_max])
                r.update(seed=s, condition=cond, f_max=f_max)
                rows.append(r)
        done = [r for r in rows if r["f_max"] == f_max]
        print(f"F_max {f_max:.0f} done "
              f"({sum(r['outcome'] == 'success' for r in done)} successes "
              f"of {len(done)})", flush=True)

    # ---- report
    print("\n=== held-out results (30 paired instances per cell) ===")
    print(f"{'F_max':>6} {'cond':>7} {'success':>9} {'95% CI':>14} {'crush':>6} "
          f"{'drop':>5} {'seatf':>6} {'Fp95':>7} {'Fmax':>7} {'minF':>6} "
          f"{'esterr':>7}")
    summary = {}
    for f_max in CRUSH_LEVELS:
        for cond in ("clamp", "delta", "oracle"):
            sel = [r for r in rows if r["f_max"] == f_max and r["condition"] == cond]
            n = len(sel)
            k = sum(r["outcome"] == "success" for r in sel)
            lo, hi = wilson(k, n)
            cr = sum(r["outcome"] == "crush" for r in sel)
            dr = sum(r["outcome"] == "drop" for r in sel)
            sf = sum(r["outcome"] == "seat_fail" for r in sel)
            ok = [r for r in sel if "force_p95" in r and r["outcome"] != "seat_fail"]
            fp95 = np.median([r["force_p95"] for r in ok]) if ok else float("nan")
            fmx = np.max([r["force_max"] for r in ok]) if ok else float("nan")
            fmn = np.min([r["force_min_transport"] for r in ok]) if ok else float("nan")
            ee = [r["est_err_p95"] for r in ok if r.get("est_err_p95") is not None]
            eem = np.median(ee) if ee else None
            summary[(f_max, cond)] = dict(success=k, n=n, ci=[lo, hi], crush=cr,
                                          drop=dr, seat_fail=sf)
            print(f"{f_max:6.0f} {cond:>7} {k:5d}/{n:<3d} "
                  f"[{lo:.2f},{hi:.2f}]  {cr:6d} {dr:5d} {sf:6d} "
                  f"{fp95:7.1f} {fmx:7.1f} {fmn:6.1f} "
                  f"{eem if eem is None else round(eem, 1)!s:>7}")

    print("\npaired delta vs clamp (per instance):")
    for f_max in CRUSH_LEVELS:
        dw = cw = tie = 0
        for s in EVAL_SEEDS:
            d = next(r for r in rows if r["f_max"] == f_max and r["seed"] == s
                     and r["condition"] == "delta")["outcome"] == "success"
            c = next(r for r in rows if r["f_max"] == f_max and r["seed"] == s
                     and r["condition"] == "clamp")["outcome"] == "success"
            if d and not c:
                dw += 1
            elif c and not d:
                cw += 1
            else:
                tie += 1
        print(f"  F_max {f_max:.0f}: delta wins {dw}, clamp wins {cw}, tied {tie}")

    with open(args.json, "w") as fh:
        json.dump(dict(clamp_choice=clamp_choice, rows=rows,
                       f_targets={str(f): f_target(f) for f in CRUSH_LEVELS}),
                  fh, indent=2)
    print(f"\nwrote {args.json}")


if __name__ == "__main__":
    main()
