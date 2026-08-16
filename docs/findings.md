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

## Phase 1: what the tracking-error signal actually measures

An external audit correctly stopped the controller tournament: task success
rates cannot establish what a signal measures, and the claim was drifting
between "delta is a force sensor" and "delta-feedback helps control". The claim
is now frozen as the narrower one -- **servo tracking error is a useful
low-cost feedback signal for crush-aware gripping in a bounded operating
regime** -- and the boundary of that regime is measured, not asserted
(`scripts/characterize_delta.py`, oracle force as ground truth, no task in the
loop).

### The signal has two regimes, and only one of them is a sensor

**Sliding (before the object is seated against the fixed jaw).** This gripper
actuates one jaw, so closing first slides the object across the table. Force
stays at friction level (0-4 N) regardless of jaw travel, and a calibration
staircase started at first touch returns garbage: slopes from -1.8 to +1.0
N/count across cells. Delta carries no grip-force information here.

**Seated.** Once the object is pinched against the fixed pad, the same
staircase gives, across 20 cells spanning object widths, lateral offsets and
step sizes:

| quantity | value |
|---|---|
| slope | 1.75 - 2.21 N/count, median 2.13 (1.3x spread) |
| pooled linear fit | **2.00 N/count** |
| held-out RMSE (unseen widths) | **4.3 N** over a 4-78 N range (~6% FS) |
| hysteresis (load vs unload) | 0.6 - 4.6 N, typically ~1.3 N |

### The failure envelope

* **Motion.** With an EMPTY jaw, arm transport alone drives delta to
  p95 = 17 counts, max = 54 counts on fast (1 s) moves -- the same magnitude
  as a firm grip -- collapsing to ~2 counts on slow moves. Any force reading
  taken during fast motion is meaningless without compensation. This bounds
  the operating regime to quasi-static reads.
* **Onset ordering.** Where the object can slide, delta responds 20-90 frames
  *before* the instantaneous normal force crosses 0.5 N, because it integrates
  the friction load on the servo while the contact force flickers. Where the
  object cannot slide, delta trails force onset by 0-4 frames (~0-130 ms).
  Delta is an interaction detector before it is a force gauge.
* **Seating is a precondition.** The mapping above exists only in the seated
  regime, so any consumer of the signal must establish seating first -- which
  the stall detector does from the same channel.

### Why the tournament thrashed, in one sentence

Six controller bugs were each real, but all of them -- entry spikes, overshoot,
transport peaks, relief runaway -- were downstream of one unmeasured fact: the
pinch is nearly rigid (~2 N per encoder count against a 100 N crush budget
gives a 50-count window, but transients traverse it in single frames at any
commanded speed above a creep). Characterise the plant first; the audit was
right that no amount of task-level iteration substitutes for it.

## Phase 1b: validity and transition audit

Three follow-ups demanded by audit before any task benchmark is run. Detector
thresholds were frozen in advance (the expert's constants); nothing was tuned
on any split. Full data in `results/phase1b.json`.

### A. Seating detection, evaluated as a classifier

Ground truth is independent of delta: the block is seated when it is in
simultaneous contact with a fixed-jaw pad and a moving-jaw pad. 90 closing
passes over widths x yaw errors (0/8/20 deg) x closing speeds x quanta:

| detector | split | precision | recall | latency med | latency p95 |
|---|---|---|---|---|---|
| stall (windowed position) | held-out | 1.00 | **1.00** | 0.10 s | 0.53 s |
| jump (single-frame delta) | held-out | 1.00 | 0.67 | 0.10 s | 0.23 s |

The stall detector is validated; the jump detector misses a third of seatings
and is retired as a primary.

**Stated limitation:** every one of the 90 passes ended seated -- even 20 deg
of yaw error produced no jams or escapes from a centred hover in this scene --
so the negative class is EMPTY and the false-positive rate against
jammed/misaligned objects is unmeasured. Precision 1.00 is on a
negatives-free distribution. Measuring it needs adversarial initial poses
(deliberate lateral offsets past the pad edge, tilted objects), noted for
Phase 2's protocol.

### B. Held-out force error in control-relevant statistics

The frozen pooled calibration (2.00 N/count) scored on held-out widths:

| cell | bias | RMSE | p95 abs | worst abs |
|---|---|---|---|---|
| 8.0 mm, centred | -2.2 N | 2.5 N | 4.1 N | 4.8 N |
| 8.0 mm, 3 mm offset | -1.5 N | 2.0 N | 3.5 N | 5.6 N |
| 9.0 mm, centred | **+5.4 N** | 5.5 N | 8.1 N | 9.0 N |
| 9.0 mm, 3 mm offset | **+5.6 N** | 5.8 N | 8.1 N | **9.2 N** |

Worst-case error is ~2x the RMSE, and the bias is width-dependent (the global
linear fit reads high on wider objects). The single-number error budget for a
global calibration is **+-9.2 N worst-case**; per-width calibration would
roughly halve it. Any Phase 2 window narrower than ~20 N is therefore not
safely regulable with the global fit.

### C. In-grasp transport confound (the decisive one)

A block seated at 36 N, jaw command FROZEN, the actual transport trajectory
run at three speeds, everything logged synchronously:

| transport | true force range | delta deviation p95 | corr(delta, F) | slip |
|---|---|---|---|---|
| 1.0 s | **0 - 59 N** | 17.9 counts (**35.9 N**) | +0.25 | 3.6 mm |
| 1.8 s | 0 - 63 N | 18.0 counts (36.0 N) | -0.27 | 3.1 mm |
| 3.7 s | 36 - 44 N | 3.9 counts (7.7 N) | n/a (F ~const) | 0.3 mm |

Two separate facts, cleanly split:

1. **The plant itself is violent at speed.** With the jaw frozen, the TRUE
   grip force swings 0-63 N during a 1-1.8 s transport -- momentary contact
   loss to nearly double the seated force. This is a property of the arm and
   contact, present with a perfect sensor; it is what the controller phase was
   fighting.
2. **The sensor is motion-blind at speed.** Delta departs from its calibrated
   value by up to 36 N (p95) and its correlation with true force collapses to
   |r| < 0.3. At the 3.7 s transport, deviation is 7.7 N p95 and the
   calibration approximately holds. The residual is essentially uncorrelated
   with arm acceleration magnitude (|r| <= 0.25), so a simple acceleration
   feedforward will not rescue the fast regime.

The earlier onset-ordering spread, in seconds: delta leads a 0.5 N force
threshold by 0.7-3.0 s during sliding, the spread tracking closing speed and
push distance -- it is a friction/obstruction signal, not force anticipation,
and no fixed delta threshold marks the transition across speeds.

### The operating envelope, as measured

Delta may be read as grip force ONLY when all of the following hold:

* seating confirmed by the stall detector (validated held-out at
  precision/recall 1.00/1.00, latency p95 0.53 s; FP rate vs jams unmeasured);
* the jaw stationary for at least 4 control frames (0.13 s);
* arm transport at the slow tier (>= ~3.7 s for the tested trajectory), where
  the dynamic deviation is 7.7 N p95 -- at <= 1.8 s it is 36 N and the reading
  is meaningless;
* object width within the calibrated 7.6-9.5 mm, load within 4-78 N;
* an error budget of +-9.2 N worst-case (global fit) held against whatever
  force window the task defines.

Outside the envelope the controller must hold, slow down, or abort -- not
force-regulate from delta.

## Phase 1c: adversarial negatives and the dynamic boundary

Configuration v1 (stock pads, stock scene) throughout; nothing about the
contact mechanics was changed. Detector thresholds unchanged from Phase 1b.

### The stall detector against constructed negatives

Eleven states including empty closure, objects outside the gap on both sides,
corner contact, 45-degree yaw, over-width, under- and over-height:

| verdict | states |
|---|---|
| true positive (7), latency 0.07-0.10 s | seated_control, outside_fixed, outside_moving, corner_contact, too_short, too_tall, narrow |
| true negative (2) | empty, mostly_missing |
| **false positive on a jam** | yaw_45deg -- block spun past 15 deg, detector still fired |
| **false positive, early** | too_wide -- fingertip lands on top and stalls the jaw before any two-pad seat |

So the honest numbers replace Phase 1b's negatives-free 1.00: on this
adversarial set, recall for true seats stays 1.00 (7/7, including degenerate
placements that nonetheless seated) and empty/missing closures never trigger,
but **grasp-ready precision is 7/9 = 0.78**: the stall detector detects
OBSTRUCTION, and a jammed or fingertip-topped object obstructs exactly like a
seated one. Position history alone cannot distinguish them. A post-detection
seat check (a small commanded squeeze whose delta response must match the
calibration slope) is the obvious discriminator and is future work, stated
rather than assumed.

### The dynamic boundary (pre-declared sweep and criteria)

Durations 0.7-5.0 s over the fixed trajectory, jaw frozen at a ~36 N seat.
Criteria declared before running: C1 no contact loss (min force > 5 N),
C2 slip < 1 mm, C3 delta residual p95 <= 4 counts, C4 force excursion <= 15 N.

| duration | min force | excursion | slip | resid p95 | passes |
|---|---|---|---|---|---|
| 0.7 - 2.5 s | 0.2-0.3 N | 36 N | 1.6-3.6 mm | ~18 counts | no |
| 3.0 s | 0.3 N | 36 N | 0.68 mm | 5.5 | no (C1) |
| **3.7 s** | 36.3 N | 8.0 N | 0.26 mm | 3.9 | **yes** |
| 5.0 s | 36.2 N | 8.0 N | 0.17 mm | 3.9 | yes |

The boundary is sharp and sits between 3.0 and 3.7 s: below it the block
momentarily LOSES CONTACT entirely (min force 0.3 N) on every run -- the
dominant failure is C1, not sensing -- and above it every criterion clears at
once. **Operating tier: 3.7 s** for this trajectory (~0.18 m path), published
as the envelope's transport condition.

### Envelope, final form (configuration v1)

Delta supports seated, quasi-static, crush-aware gripping when: seating is
stall-detected AND the jam/topping ambiguity is accepted or screened (0.78
grasp-ready precision adversarially); the jaw has been stationary >= 0.13 s;
transport follows the 3.7 s tier; width 7.6-9.5 mm; load 4-78 N; against a
+-9.2 N worst-case global-fit budget. Anything faster, unseated, or wider is
outside the claim.

## Phase 2: the physical safe-grip window (no feedback)

Pre-declared protocol (`scripts/phase2_window.py`): 8.5 mm block, oracle used
only to SET the seated force, jaw then frozen, the validated 3.7 s trajectory
plus a 1 s hold as the fixed disturbance, retention = block displacement in
the tool frame under 5 mm, 5 reps per level with +-2 mm pose jitter, a level
passes at >= 4/5. Configuration v1, unchanged since characterisation.

| seated force | retained | worst slip |
|---|---|---|
| 36 N | 5/5 | 0.28 mm |
| 28 N | 5/5 | 0.49 mm |
| **20 N** | **5/5** | 3.85 mm |
| 14 N | 1/5 | 78 mm |
| 10 N | 1/5 | 77 mm |
| 7 N and below | 0/5 | ~85-95 mm |

**F_min = 20 N**, with a sharp boundary between 14 and 20 N. As stated in
advance, this sits nearly two orders of magnitude above the ~0.25 N static
friction prediction: retention under this protocol is governed by the 8 N
transport force excursion and torque about the 1 mm pinch line, not by static
friction against gravity.

The windows against candidate crush levels, versus the >= 20 N acceptance gate
(twice the 9.2 N worst-case estimation budget):

| crush candidate | W = crush - F_min | usable with global calibration |
|---|---|---|
| 60 N | 40 N | yes |
| 80 N | 60 N | yes |
| 100 N | 80 N | yes |
| 120 N | 100 N | yes |

Explicitly: F_min = 20 N; for F_max in {60, 80, 100, 120} N the window is
W(F_max) = F_max - 20 N, i.e. {40, 60, 80, 100} N. Every candidate clears the
gate; the task is physically feasible as built and no mechanical redesign
(which would be configuration v2, requiring full re-characterisation) is
needed.

On the 3.85 mm slip at F_min: the retention criterion here is "retained"
(tool-frame displacement < 5 mm), deliberately weaker than the dynamic-boundary
protocol's < 1 mm slip criterion -- the boundary sweep certifies where the
SENSOR reading stays valid, while this protocol measures where the GRASP
physically survives. 20 N is therefore a retained-with-residual-slip floor,
not a no-slip grasp; a no-slip floor under the same protocol would sit nearer
28 N (0.49 mm).

## The claim, final form

**Servo tracking error supports seated, low-dynamic, crush-aware gripping
within a validated operating envelope** ("quasi-static" is reserved for the
sensor-calibration and hold phases, where the jaw is stationary; the 3.7 s
transport tier is validated bounded-dynamics transport, not a demonstrated
quasi-static regime) -- seating stall-detected (recall 1.00,
grasp-ready precision 0.78 against adversarial jams), jaw stationary >= 0.13 s,
the 3.7 s transport tier, widths 7.6-9.5 mm, loads 4-78 N, +-9.2 N worst-case
estimation budget, and a measured safe-grip window of 40-100 N for crush
thresholds 60-120 N.

Nothing broader is supported: delta is not a general force sensor (it is
non-informative while the object slides, and unusable during fast transport,
where the true grip force itself swings 0-63 N with a frozen jaw). The
clamp/delta/oracle comparison is now interpretable under this protocol and is
the next authorised step.
