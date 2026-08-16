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
| place rate (randomised) | **78%** (31/40) |
| retention of picked blocks | **100%** |
| nominal scene | 100% |
| speed, headless | 0.65 s/episode |
| speed, 2 × 128 px cameras | 2.7 s/episode |
| dataset yield (successes only) | ~78% of attempts |

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

The block's half-width is capped at 9.5 mm, which is **not** a taste
parameter. The fixed jaw's fingertip sits 11.9 mm from the tool centre across
the closing axis, and anything wider is struck from above rather than enclosed;
the cap leaves margin against that limit instead of sitting on it. Capping at
11.0 mm left the largest blocks 0.9 mm of clearance — which an open-loop clamp
forces through, but a force-regulated grip cannot. Widening the margin raised
the clamp expert's place rate from 10/20 to 15/20 and the force-regulated
expert's pick rate from 8/20 to 15/20.

## Layout

```
src/so101_bench/
  scene.py       PickScene: MuJoCo scene, IK on the true tool frame, DR, force channel
  expert.py      ScriptedExpert: waypoint policy, clamp/force/oracle grasps
  demo_env.py    DemoEnv: rollout, evaluation, LeRobotDataset collection
  assets/        MJCF scene and meshes
scripts/         evaluate.py, collect.py, render_demo.py
docs/findings.md what was measured, and what it cost to find out
```

## Grip modes, and what they measure

`ExpertConfig(grip_mode=...)` selects the signal that sets the squeeze. The
three modes are identical apart from that, which makes comparing them a
measurement rather than three implementations:

| mode | grip signal | role |
|---|---|---|
| `clamp` | fixed jaw angle | no force feedback at all |
| `force` | `delta`, quantised | the channel the real robot has |
| `oracle` | true contact force | privileged upper bound |

The gaps decompose: `clamp → oracle` is the total value of force feedback on a
task, `clamp → force` the value of the channel that actually exists, and
`force → oracle` the price of having a tracking-error proxy instead of a sensor.
Sweeping `obs_quantum` under `force` gives a dose-response curve — all without
training anything.

**On this task, that gap is zero.** Over 40 randomised episodes:

| expert | pick | place | grip force |
|---|---|---|---|
| `clamp` | 31/40 | **31/40** | 301 N |
| `force` | 31/40 | 28/40 | 51 N |
| `oracle` | 29/40 | 25/40 | 37 N |

Force feedback buys nothing here, and costs a little: a gentler grip drops more
often and nothing penalises crushing. This is the intended negative control —
rigid-block pick-and-place is *not* a force-sensitive task, so it cannot be used
to study force resolution.

## The fragile task, and the resolution cliff

`crush_newtons` loses the episode if grip force ever exceeds a limit, so success
requires landing inside a *window* instead of squeezing as hard as possible.
That inverts the control:

| task | clamp | force (`delta`) | oracle (true N) |
|---|---|---|---|
| rigid block | **23/30** | 19/30 | 18/30 |
| crush 120 N | **0/30** | **17/30** | 16/30 |

The open-loop clamp goes from best to zero — a 356 N median grip destroys the
block every time — while the quantised `delta` proxy substitutes fully for a
real force sensor.

Sweeping the observation quantum against the crush limit (`placed / 30`,
`scripts/resolution_sweep.py`):

| crush limit | q=1 | q=4 | q=8 | q=16 | q=32 | clamp | oracle |
|---|---|---|---|---|---|---|---|
| 100 N | 11 | 11 | 9 | **1** | 0 | 0 | 12 |
| 120 N | 17 | 17 | 16 | 10 | **0** | 0 | 14 |
| 160 N | 20 | 20 | 19 | 17 | **11** | 0 | 17 |

Coarsening the channel destroys the task, as a cliff rather than a slope — and
**the cliff moves with the margin**. An effect that appeared at every crush limit
alike would just be the loss of input bits; this one tracks the physical
headroom. At the real encoder's resolution the proxy matches the oracle.

The quantitative prediction (`cliff ≈ margin / 2.4 N per count`) gets the
ordering right and the constant wrong — off by 3.1x, 1.5x and 1.2x as the margin
grows. See `docs/findings.md`; the limiting error at present is contact-detection
latency (19–56 N of entry force), not the encoder.

## Provenance

The MJCF scene originates from a working teleoperation setup and was verified by
replaying its recorded demonstrations (8/10 episodes place the block). Three
properties make grasping possible here and are absent from a URDF-derived
model: an elliptic friction cone at `impratio=10`, explicit thin-box finger
pads, and collision meshes separate from visual meshes.
