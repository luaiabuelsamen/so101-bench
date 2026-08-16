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
70%/55% to 78%/60%; a later widening of the graspable-width margin took the
place rate to 78% (see *The rigid-block task does not need force*).

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

| grip mode | tracking error | true grip force |
|---|---|---|
| `clamp` | −147 counts, constant | 280–580 N |
| `force` | graded, −10..−30 counts | 0–74 N |

A 500 N "grasp" is a rigid clamp, and there is no gradation left to resolve.
Correlation between `|delta|` and true force across contact frames was −0.33
under the clamp — the sign is right but the magnitude is meaningless, because
the error was saturated.

## Free-space lag: real, but not something to threshold around

A P-controlled servo that is *moving* sits behind its goal whether or not it is
touching anything. Stepping the jaw 0.006 rad per frame is 3.9 counts of command
travel, so the tracking error reads several counts in free space, and only the
excess over that lag is load.

Thresholding the raw value stopped the close almost immediately — **1 pick in
15**, jaw halting at 1–6 counts with 0.00 N of contact. Subtracting a measured
baseline first is better but still fails (**0 picks in 20**); see *Contact
detection* below for why, and for what replaced it.

The confound itself is not a simulation artifact. It exists on hardware, and it
cannot be avoided there by reading a force sensor, because there isn't one.

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

## Contact detection: use measured motion, not tracking error

Regulating the grip needs a contact point. The obvious detector — threshold the
tracking error — does not work, for reasons visible in a single logged close:

```
k    jaw_cmd   jaw_act   delta   grip
0     0.5400    0.5488    -6.0    0.00     <- servo still accelerating
9     0.4500    0.4756   -17.0    0.00     <- steady free-space lag
33    0.2100    0.2388   -19.0    0.61     <- brushing, not gripping
51    0.0300    0.0607   -20.0    6.15
54   -0.0000    0.0588   -38.0   92.44     <- stalled; actual jaw frozen
57   -0.0300    0.0587   -58.0  133.44
```

Three traps in that trace:

1. The free-space error is not zero but a steady **-17 count** lag, purely from
   command travel.
2. It is not steady at the start. Over the first eight frames the servo is
   accelerating and reads -6 rising to -17, so a baseline measured there
   *underestimates* the lag and any small margin above it fires immediately.
3. Brushing the object moves the error only to -19..-21 at under 6 N — inside
   the free-space band.

Thresholding `|delta|` with a 3-count margin therefore triggered in free space,
and the subsequent squeeze started from nowhere near the object: **0 picks in
20**, with the achieved error only -1..-6 counts.

The same trace contains a far cleaner signal. A free jaw follows the command at
0.0100 rad/frame; a blocked one moves 0.0001 rad/frame. Detecting the **stall**
in measured position needs no baseline and no calibration, and `Present_Position`
over time is equally available on hardware.

Grip force is then commanded as a fixed **overshoot in encoder counts** past the
detected contact point. That decouples "where is the object" from "how hard to
squeeze", and only the second has to be precise — and it is set by an integer,
which is exactly where the encoder quantum should enter.

## The rigid-block task does not need force (negative control)

Three experts differing *only* in the signal that sets the squeeze, 40 randomised
episodes each:

| expert | grip signal | pick | place | grip force |
|---|---|---|---|---|
| `clamp` | fixed angle | 31/40 | **31/40** | 301 N |
| `force` | `delta`, quantised | 31/40 | 28/40 | 51 N |
| `oracle` | true contact force | 29/40 | 25/40 | 37 N |

`clamp → oracle` is **-6/40**: force feedback buys nothing here and costs a
little, because a gentler grip drops the block more often while nothing
penalises crushing it. At n=40 that is roughly 1.5 standard errors — no evidence
of benefit, not evidence of harm.

This is the negative control working as intended. Rigid-block pick-and-place
cannot be used to study force resolution, and now that is measured rather than
assumed. A grip *window* — drop below it, crush above it — is required, and the
`oracle` expert gives the upper bound against which any such task should be
screened before spending training time on it.

## A task that needs force, and a resolution cliff that moves with the margin

The rigid-block task cannot test force resolution (previous section). Adding a
**crush limit** — the episode is lost if grip force ever exceeds it — supplies
the missing upper bound, so success requires landing inside a window rather than
squeezing as hard as possible.

That flips the negative control into a positive one. 30 episodes per cell:

| task | clamp | force (`delta`) | oracle (true N) |
|---|---|---|---|
| rigid block | **23/30** | 19/30 | 18/30 |
| crush 120 N | **0/30** | **17/30** | 16/30 |

The open-loop clamp goes from best to *zero*: at a median peak grip of 356 N it
destroys the block every time. The quantised `delta` proxy substitutes fully for
a true force sensor.

### Resolution

Sweeping the observation quantum against the crush limit, force-regulated expert,
`placed / 30`:

| crush limit | q=1 | q=4 | q=8 | q=16 | q=32 | clamp | oracle |
|---|---|---|---|---|---|---|---|
| 100 N | 11 | 11 | 9 | **1** | 0 | 0 | 12 |
| 120 N | 17 | 17 | 16 | 10 | **0** | 0 | 14 |
| 160 N | 20 | 20 | 19 | 17 | **11** | 0 | 17 |

Three things worth separating:

1. **Coarsening the channel destroys the task**, and the collapse is a cliff
   rather than a slope: flat from q=1 to q=8, then a fall.
2. **The cliff moves with the margin.** At a 100 N limit, q=16 leaves 1/30; at
   160 N the same quantum leaves 17/30. An effect that appeared at every crush
   limit alike would be an artifact of removing input bits — this one tracks the
   physical headroom, which is the signature of a genuine resolution limit.
3. **At the real encoder's resolution (q=1) the proxy matches the oracle** —
   11 vs 12, 17 vs 14, 20 vs 17. On this task the tracking-error channel is as
   good as a force sensor, and the open-loop clamp is worthless.

### Where the quantitative prediction stands

Force changes by ~2.4 N per encoder count and typical peak grip is ~91 N, giving
a predicted failure quantum of `(crush - 91) / 2.4`. Taking the cliff as the
quantum where success halves:

| crush | margin | predicted | observed |
|---|---|---|---|
| 100 N | 9 N | 3.8 | ~11.5 |
| 120 N | 29 N | 12.1 | ~18.4 |
| 160 N | 69 N | 28.8 | ~34 |

Both rise monotonically and the gap narrows as the margin grows, so the
mechanism is right, but the constant is not: the prediction is off by 3.1x, 1.5x
and 1.2x. `TYPICAL_PEAK_N` is a single median stand-in for a distribution that
actually spans 19-56 N at contact alone, which is the obvious suspect. The law
should be re-derived from the peak-grip *distribution* before it is claimed as
quantitative.

### What actually limits the grip

Contact detection, not the encoder. Entry force at the moment of detection is
19-56 N with a median of 41 (n=28), while the encoder affords 2.4 N per count.
The dominant error is therefore ~15x the quantisation step, which is why the
resolution effect only appears once the quantum is coarsened well past 1 count.
A finer approach near contact would tighten the entry distribution and move the
whole family of cliffs toward finer quanta.
