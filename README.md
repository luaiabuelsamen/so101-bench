# so101-bench

A MuJoCo pick-and-place environment for the low-cost **SO-100 / SO-101** arm,
with domain randomisation, a scripted expert, and LeRobot-schema data
collection — built around the proprioceptive force channel that a
position-controlled hobby servo exposes through its own tracking error.

```python
from so101_bench import DemoEnv

env = DemoEnv(seed=0)
result, frames = env.rollout()          # one scripted episode
env.collect(200, root="~/data/pick_place")   # LeRobotDataset
```

## Why

The SO-101 has no torque sensor. It does have a P-controlled servo, and a servo
holding against a load sits at an offset from its commanded position
proportional to that load. That offset

```
delta = action[t-1] - state[t]        # encoder counts
```

is a force signal that already exists in every recorded SO-100/101 dataset, is
computable at inference time, and is quantised by the encoder. This package
exists to study what that channel is worth, in a task where grasping actually
has to work.

## Status

| metric | value |
|---|---|
| pick rate (randomised) | **78%** (31/40) |
| place rate (randomised) | **60%** (24/40) |
| nominal scene | 100% |
| speed, headless | 0.65 s/episode |
| speed, 2 × 128 px cameras | 2.7 s/episode |
| dataset yield (successes only) | ~45% of attempts |

Reproduce with `python scripts/evaluate.py --episodes 40`.

## Install

```bash
pip install -e ".[data,video,dev]"
make test
```

Requires Python ≥3.10. Rendering uses EGL and works headless.

> `make` is used rather than bare `pytest` because a system ROS installation
> registers pytest plugins globally that fail on an unrelated import; the
> Makefile disables plugin autoloading. This project does not use ROS.

## What is recorded

Per control frame, in the same schema as the public SO-100/101 datasets:

| column | meaning |
|---|---|
| `observation.state` | 6 joints, integer encoder counts, read **before** the command |
| `action` | 6 joint goals, integer encoder counts |
| `observation.images.<camera>` | `front`, `side`, `gripper_fpv` |
| `grip_force_N` | true contact force — **analysis only, never an observation** |

`delta = action[t-1] - state[t]` is therefore reconstructable downstream exactly
as it is from real data. Both state and command are rounded to whole counts,
because `Goal_Position` is an integer register; without that the channel is
continuous and the question of *resolution* disappears.

`obs_quantum` coarsens or refines the observation quantum alone
(`obs_quantum=0.25` models a 4× finer encoder), which is the knob for
resolution ablations.

## Domain randomisation

Per episode: block position, yaw (full ±π), size, mass, friction, and the
container's position. See `DomainRandomization`.

The block's half-width is capped at 11.9 mm, which is **not** a taste
parameter — it is where the fixed jaw's fingertip sits relative to the tool
centre. Anything wider is struck from above rather than enclosed.

## Layout

```
src/so101_bench/
  scene.py       PickScene: MuJoCo scene, IK on the true tool frame, DR, force channel
  expert.py      ScriptedExpert: waypoint policy, clamp and force-regulated grasps
  demo_env.py    DemoEnv: rollout, evaluation, LeRobotDataset collection
  assets/        MJCF scene and meshes
scripts/         evaluate.py, collect.py, render_demo.py
docs/findings.md what was measured, and what it cost to find out
```

## Grip modes

`ExpertConfig(grip_mode=...)` selects how the jaw closes:

- **`clamp`** (default) drives the jaw to a fixed angle. More reliable as a
  demonstrator, but the servo saturates: tracking error pins at −147 counts
  while true grip force runs 280–580 N, so the force channel stops responding
  to the object.
- **`force`** closes until the tracking error exceeds its free-space lag by a
  target, using only the quantised channel. Grip force becomes graded
  (0–39 N), but the current pick rate is lower. Needed for force-sensitive
  tasks.

See `docs/findings.md` for the measurements behind both.

## Provenance

The MJCF scene originates from a working teleoperation setup and was verified by
replaying its recorded demonstrations (8/10 episodes place the block). Three
properties make grasping possible here and are absent from a URDF-derived
model: an elliptic friction cone at `impratio=10`, explicit thin-box finger
pads, and collision meshes separate from visual meshes.
