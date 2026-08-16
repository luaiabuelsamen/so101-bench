# Findings

Measurements behind the constants in the code, and the mistakes that produced
them. Recorded because several were expensive, and because most of them are
invisible to the obvious diagnostic.

## The URDF-derived scene cannot grasp

An earlier version of this environment built its model from
`so101_new_calib.urdf`. A URDF has no notion of a collision geometry separate
from a visual one, so every gripper geom becomes the convexified visual mesh:
the jaws are smooth blobs with no pinch surface.

Measured there, with the fixed nose driven 12 mm *into* the block, the moving
jaw was still 38 mm away — at **every** approach height from 0 to 140 mm. No
controller fixes that.

It also breaks the obvious diagnostic. `mj_geomDistance` returns spurious exact
zeros on those mesh pairs:

```
gap z   0.0400  0.0425  0.0450  0.0475  0.0500  0.0525
nose      6.29    0.00    7.66    8.35    9.05    0.00
```

A search that scores candidate grasps by minimising `|distance|` converges
straight onto those artifacts. Two separate searches reported "perfect" grasps
at cost 0.000000; simulating them lifted the block **1.6 mm and 0.8 mm**.

**Rule this produced:** score candidate grasps by simulating them and measuring
the object, never by querying distances.

## The scene that works

Three properties, all absent from a URDF build:

- `<option cone="elliptic" impratio="10"/>` — the default pyramidal cone at
  `impratio=1` lets a grasped object slide out of the fingers.
- Explicit thin-box finger pads (`fixed_jaw_pad_1..4`, `moving_jaw_pad_1..4`,
  `solimp="2 1 0.01"`, `solref="0.01 1"`).
- Collision meshes distinct from visual meshes.

Verified by replaying recorded teleoperation demonstrations: 8 of 10 episodes
lift the block 65–213 mm and deposit it in the container.

## The tool frame is not where the model says

The URDF scene's `tool` site sits **71 mm below the jaws** — a TCP for a long
tool, not a pinch point. Aiming it at a block on the table drives the gripper
body down through the block.

The real grasp point was recovered from the demonstrations rather than the
model: for each successful episode, the block's pose at the moment it left the
table, expressed in the `Fixed_Jaw` frame. Across eight episodes:

```
[-0.003, -0.0926, 0.0]      std 0.75 mm / 1.75 mm / 9.92 mm
```

The large scatter is along the jaw opening, where the block may sit anywhere.

## The jaws close along local X

The pads are separated along the jaw body's **local X** (fixed pads at
x = +0.009…+0.014, moving pads at x = −0.011…−0.007), not local Z. Constraining
the wrong axis in IK aims the jaws along the block's 40 mm side, which they
cannot span. Symptom: everything looks correct, nothing grasps.

## Grasp yaw: `yaw + π` is a fallback, not an equal

A rectangular block's short axis is unchanged by a 180° rotation, so it is
tempting to treat `yaw` and `yaw + π` as interchangeable and pick whichever the
arm reaches more easily. **This is wrong here.** The tool frame is not symmetric
about the closing axis: the fixed fingertip sits 11.9 mm to one side of the tool
centre, so flipping by π mirrors the pads about the tool and puts the block on
the wrong side of the jaw.

Ranking the two on IK residual alone picked the flipped heading on the nominal
scene — residual **0.0009 mm against 0.88 mm** — and lifted the block 1.3 mm.
Restricting the flip to a genuine fallback took the randomised rates from
70%/55% to **78%/60%**.

Near-miss headings (`yaw ± 0.25`) are not offered at all: 14° of misalignment on
a 25 mm long axis raises the effective half-width to 16.5 mm, past the 11.9 mm
the fingertip clears, and the tip lands on the block and presses it into the
table — measured at 35 N with the tool stalled 30 mm above the grasp.

**IK residual cannot see any of this.** In a 30-episode diagnostic, failures had
*lower* mean IK error than successes (0.54 mm vs 1.81 mm).

## Grasp depth decides retention, not pick rate

Grasps that survived the move to the container held the block within 3–12 mm of
the tool. Every one that failed sat at 14–23 mm after the lift and then jumped
to 140–168 mm — shed during transport.

Swept over 20 episodes per setting:

| `grasp_dz` | picked | placed | retained |
|---|---|---|---|
| +8 mm | 15/20 | 8/20 | 53% |
| +4 mm | 15/20 | 9/20 | 60% |
| 0 mm | 14/20 | 11/20 | **79%** |
| −4 mm | 14/20 | 11/20 | 79% |

Slowing the transport (55 → 85 frames) changes nothing, so the block is not
being shed by inertia; it was never seated deeply enough. The depth limit is the
fingertip, 8.8 mm below the tool, which must clear the table.

## Release height

The container walls top out at z = 0.145. Releasing with the tool at
`box + 0.085` puts the block's underside at 0.16 — above the walls, falling
6 cm — and it bounces back out.

## The force channel saturates under a clamp grasp

Closing to a fixed jaw angle commands the jaw ~0.23 rad *past* the block. The
servo pins against its limit and the tracking error stops responding to the
object:

| grip mode | tracking error | true grip force | corr(\|delta\|, force) |
|---|---|---|---|
| `clamp` | −147 counts, constant | 280–580 N | −0.33 |
| `force` | graded | 0–39 N | **+0.50** |

A 500 N "grasp" is a rigid clamp, and there is no gradation left to resolve.

## Free-space lag must be subtracted before thresholding

A P-controlled servo that is *moving* sits behind its goal whether or not it is
touching anything. Stepping the jaw 0.006 rad per frame is 3.9 counts of command
travel, so the tracking error reads several counts in free space.

Thresholding the raw value stopped the close almost immediately — **1 pick in
15**, jaw halting at 1–6 counts with 0.00 N of actual contact. Only the excess
over the free-running lag is load.

This confound is not a simulation artifact. It exists on hardware, and it cannot
be avoided there by reading a force sensor, because there isn't one.

## Testing notes

Two failures in the first test run were the tests, not the code, and both are
easy to repeat:

- `data.xmat` and `data.geom_xpos` are **live views** into `MjData`. Copy before
  restoring the caller's state, or the assertion silently reads the restored
  pose.
- IK seeded from `zeros(5)` leaves a 47 mm residual at grasp height. The expert
  warm-starts from the hover solution; a test that does not, tests nothing.
- Commanding the grasp configuration directly from home swings the arm through
  the block and knocks it clear. Ramp between poses.
