"""Boundary-focused contact-transition dataset from the validated expert.

    python scripts/contact_dataset.py --out data/contact_transitions.parquet

Collected ONCE at native quantum; every coarse arm is generated OFFLINE from
the same episodes (quantisation of the observation is a pure function of the
recorded native state), so all resolution arms are paired at frame level. This
is valid for the perception question asked here; any later CLOSED-LOOP
evaluation must run online coarsening instead, because coarse observations
change behaviour (measured: the stall window widens with the quantum and entry
force rises).

Per control frame:
    state (6 joints, native encoder counts, float), previous action (counts),
    plus analysis-only oracle labels never available to a model at inference:
    per-pad contact sides, true grip force, block pose/upright/spun, slip
    speed, and the episode's geometry and type.

Episode taxonomy (oversampling the boundary states from the Phase 1c audit):
in-envelope seats across widths/offsets/yaws, long-slide seats (centred
descend maximises the sliding phase), empty closure, object outside the gap on
either side, corner contact, 45-degree jams, over-width fingertip topping,
under- and over-height objects, and out-of-envelope fast-transport segments
appended after a seat (frames where the channel is known-invalid).

Splits are BY EPISODE GEOMETRY: episodes carry a width-bin field, and the
classifier holds out whole bins, never frames.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))

from so101_bench.expert import HOME  # noqa: E402
from so101_bench.scene import (  # noqa: E402
    JAW_OPEN,
    JAW_SHUT,
    TABLE_TOP,
    DomainRandomization,
    PickScene,
)

CONTROL_HZ = 30.0
CLOSE_SPEED = 0.003

#: (episode type, count, spec factory). Specs randomised per episode by seed.
def episode_plan(rng):
    plans = []
    for _ in range(40):        # in-envelope seats, the positive mass
        plans.append(("seat", dict(
            half_w=rng.uniform(0.0076, 0.0095),
            perp=rng.uniform(-0.004, 0.004),
            yaw=rng.uniform(-0.175, 0.175),
            tool_dx=0.0,
        )))
    for _ in range(15):        # long-slide seats: centred descend, long push
        plans.append(("seat_longslide", dict(
            half_w=rng.uniform(0.0076, 0.0095), perp=0.0,
            yaw=rng.uniform(-0.1, 0.1), tool_dx=-0.006,   # undo fixed-pad bias
        )))
    for _ in range(10):
        plans.append(("empty", dict(empty=True)))
    for _ in range(8):
        plans.append(("outside_fixed", dict(tool_dx=+0.014,
                                            half_w=rng.uniform(0.0076, 0.0095))))
    for _ in range(8):
        plans.append(("outside_moving", dict(tool_dx=-0.020,
                                             half_w=rng.uniform(0.0076, 0.0095))))
    for _ in range(8):
        plans.append(("corner", dict(perp=0.015,
                                     half_w=rng.uniform(0.0076, 0.0095))))
    for _ in range(8):
        plans.append(("missing", dict(perp=0.025,
                                      half_w=rng.uniform(0.0076, 0.0095))))
    for _ in range(12):        # jams
        plans.append(("jam_yaw", dict(yaw=rng.uniform(0.6, 0.95),
                                      half_w=rng.uniform(0.0076, 0.0095))))
    for _ in range(12):        # fingertip topping
        plans.append(("jam_wide", dict(half_w=rng.uniform(0.0115, 0.0135))))
    for _ in range(6):
        plans.append(("short", dict(half_h=0.006,
                                    half_w=rng.uniform(0.0076, 0.0095))))
    for _ in range(6):
        plans.append(("tall", dict(half_h=0.030,
                                   half_w=rng.uniform(0.0076, 0.0095))))
    rng.shuffle(plans)
    return plans


def setup(scene, spec):
    import mujoco

    xy = (0.30, -0.35) if spec.get("empty") else (
        0.10 + spec.get("dx", 0.0), -0.25 + spec.get("perp", 0.0))
    blk = scene.reset(block_xy=(0.10, -0.25), block_yaw=spec.get("yaw", 0.0))
    scene.model.geom_size[scene.ids.block_geom] = [
        spec.get("half_w", 0.0085), 0.018, spec.get("half_h", 0.015)]
    mujoco.mj_forward(scene.model, scene.data)
    tool = scene.open_gap_local.copy()
    tool[0] += spec.get("tool_dx", 0.0)
    yaw = spec.get("yaw", 0.0)
    tx, ty = (xy if spec.get("empty") else (blk[0], blk[1] + spec.get("perp", 0.0)))
    q_h, _ = scene.solve_ik(np.array([tx, ty, TABLE_TOP + 0.09]), HOME,
                            yaw=yaw, tool_local=tool)
    q, _ = scene.solve_ik(np.array([tx, ty, TABLE_TOP + 0.0316]), q_h,
                          yaw=yaw, tool_local=tool)
    for a, b in ((HOME, q_h), (q_h, q)):
        for f in np.linspace(0, 1, 40):
            scene.hold(np.concatenate([a + (b - a) * f, [JAW_OPEN]]))
    for _ in range(8):
        scene.hold(np.concatenate([q, [JAW_OPEN]]))
    return q


class Tap:
    """Captures (state, action) for every control frame via scene.recorder."""

    def __init__(self):
        self.states, self.actions = [], []

    def append(self, state, action, scene):
        self.states.append(np.asarray(state, float).copy())
        self.actions.append(np.asarray(action, float).copy())


def pad_sides(scene):
    m = scene.model
    return ({m.geom(f"fixed_jaw_pad_{i}").id for i in range(1, 5)},
            {m.geom(f"moving_jaw_pad_{i}").id for i in range(1, 5)})


def collect_episode(scene, ep_id, ep_type, spec):
    fixed, moving = pad_sides(scene)
    q = setup(scene, spec)
    tap = Tap()
    scene.recorder = tap
    labels = []
    yaw0, upright = scene.block_yaw(), True
    prev_pos = scene.block_pos()

    def snap(phase):
        import mujoco  # noqa: F401

        d = scene.data
        tf = tm = False
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {c.geom1, c.geom2}
            if scene.ids.block_geom in pair:
                if pair & fixed:
                    tf = True
                if pair & moving:
                    tm = True
        R = d.xmat[scene.ids.block].reshape(3, 3)
        return dict(contact_fixed=tf, contact_moving=tm,
                    force=scene.grip_force(),
                    upright=bool(R[2, 2] >= 0.95),
                    spun=bool(abs(scene.block_yaw() - yaw0) > 0.26),
                    phase=phase)

    # closing pass
    jaw = JAW_OPEN
    while jaw > JAW_SHUT:
        jaw = max(JAW_SHUT, jaw - CLOSE_SPEED)
        scene.hold(np.concatenate([q, [jaw]]))
        pos = scene.block_pos()
        row = snap("close")
        row["slip_speed"] = float(np.linalg.norm(pos - prev_pos) * CONTROL_HZ)
        prev_pos = pos
        labels.append(row)

    # if seated, append an out-of-envelope fast transport (known-invalid frames)
    seated_now = labels[-1]["contact_fixed"] and labels[-1]["contact_moving"]
    if seated_now and ep_type.startswith("seat"):
        q1, _ = scene.solve_ik(np.array([-0.05, -0.30, 0.18]), q, yaw=0.0)
        frames = int(1.4 * CONTROL_HZ)
        for k in range(frames):
            f = 0.5 * (1 - np.cos(np.pi * (k + 1) / frames))
            scene.hold(np.concatenate([q + (q1 - q) * f, [jaw]]))
            pos = scene.block_pos()
            row = snap("fast_transport")
            row["slip_speed"] = float(np.linalg.norm(pos - prev_pos) * CONTROL_HZ)
            prev_pos = pos
            labels.append(row)
    scene.recorder = None

    n = min(len(tap.states), len(labels))
    rows = []
    for t in range(n):
        r = dict(labels[t])
        r.update(
            episode=ep_id, ep_type=ep_type, t=t,
            half_w=spec.get("half_w", 0.0085),
            width_bin=int(np.digitize(spec.get("half_w", 0.0085),
                                      [0.0080, 0.0085, 0.0090, 0.0100])),
            yaw_spec=spec.get("yaw", 0.0),
            jaw_state=tap.states[t][5],
            jaw_action=tap.actions[t][5],
        )
        for j in range(6):
            r[f"s{j}"] = tap.states[t][j]
            r[f"a{j}"] = tap.actions[t][j]
        rows.append(r)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=str, default="data/contact_transitions.parquet")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    scene = PickScene(seed=0, randomise=DomainRandomization.off())
    rng = np.random.default_rng(args.seed)
    plans = episode_plan(rng)
    all_rows = []
    for ep_id, (ep_type, spec) in enumerate(plans):
        all_rows.extend(collect_episode(scene, ep_id, ep_type, spec))
        if (ep_id + 1) % 20 == 0:
            print(f"  {ep_id + 1}/{len(plans)} episodes, {len(all_rows)} frames",
                  flush=True)

    df = pd.DataFrame(all_rows)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(out)
    print(f"\nwrote {out}: {len(df)} frames, {df['episode'].nunique()} episodes")
    print(df.groupby("ep_type")["episode"].nunique().to_string())
    both = df["contact_fixed"] & df["contact_moving"]
    print(f"\nframe classes: free {(~df['contact_fixed'] & ~df['contact_moving']).sum()}, "
          f"one-pad {(df['contact_fixed'] ^ df['contact_moving']).sum()}, "
          f"two-pad {both.sum()} "
          f"(of which not-upright/spun {(both & ~(df['upright'] & ~df['spun'])).sum()}), "
          f"fast-transport {(df['phase'] == 'fast_transport').sum()}")


if __name__ == "__main__":
    main()
