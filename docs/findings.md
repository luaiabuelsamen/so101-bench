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

Three follow-ups demanded by review before any task benchmark is run. Detector
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
clamp/delta/oracle comparison is now interpretable under this protocol.

## Final tournament: clamp matches all methods, and why that is the finding

Frozen protocol (`scripts/tournament_final.py`): configuration v1, the 3.7 s
tier, shared seating/closure/trajectory/timeouts, guard-banded targets
F_target = (20 + F_max - 17.2)/2, delta and oracle with ZERO tuned parameters,
clamp's one knob tuned on a 10-instance dev set (the tuning budget favoured
the baseline), 30 paired held-out instances per threshold.

| F_max | clamp | delta | oracle |
|---|---|---|---|
| 60 N | 29/30 [0.83,0.99] | 29/30 [0.83,0.99] | 26/30 [0.70,0.95] |
| 80 N | 30/30 | 30/30 | 30/30 |
| 100 N | 30/30 | 30/30 | 30/30 |
| 120 N | 30/30 | 30/30 | 30/30 |

Paired delta-vs-clamp: 118 of 120 instances TIED (1 win each at 60 N). No
significant difference anywhere; all intervals overlap.

Per the interpretation table frozen before the run, this is the "clamp matches
all methods" row: **within the validated envelope and the measured windows
(W >= 40 N), continuous force feedback provides no measured benefit over
stall-seated clamping.** That is reported as the result, not retuned away.

### Why, and what it actually says about the channel

The protocol-required shared seating logic changes what "clamp" means. Stall-based
seating stops where the object is, so seat-then-2-counts lands at a
width-independent ~30-39 N grip (p95 38.8 N across all widths) -- inside every
window. The "no-feedback" baseline is therefore already using the
proprioceptive channel, ONCE, as a discrete contact-detection event. The
earlier dramatic clamp collapse (0/30, 356 N) was a clamp WITHOUT seating:
what that experiment demonstrated was the value of stopping at the object, not
of continuously regulating force against it.

Restated: **on this task, inside this envelope, the value of the tracking-error
channel is concentrated in the seating event, not in continuous force
regulation.** This is consistent with every upstream measurement -- the corpus
finding that deployed datasets carry ~1.5 distinguishable force levels (enough
for contact/no-contact and little else), and the Phase 1 finding that delta is
an interaction detector before it is a force gauge.

Secondary observations, small numbers, none significant: oracle recorded 3
crushes at F_max = 60 N (its true-force tracking chases transport transients
near the tight target); delta at the higher targets held the firmest transport
grip (min force 41-52 N where clamp and oracle both saw momentary contact
loss); delta's in-transport estimation error was 3.5-5.5 N p95, consistent
with the static budget.

### The boundary of the null

The guard-band formula empties as F_max approaches F_min + 17.2 = 37.2 N, so
thresholds tight enough to separate the conditions (where a 4-9 N estimation
error should start costing crushes) sit at F_max ~ 40-55 N -- mostly outside
the interval the frozen protocol permits. Whether feedback matters THERE is a
pre-registerable extension, not a rescue of this run: this run's answer stands
as measured.

## Learning: the channel regulates learned grip force, and resolution decides it

Three ACT policies (lerobot 0.3.4, resnet18, chunk 30), identical 200-episode
demonstration set, identical architecture and training budget; the ONLY
difference is `observation.state`:

    base   s[t]                    -- stock ACT (never sees the previous action)
    delta  [s[t], a[t-1]-s[t]]     -- the characterised channel, native quantum
    q16    same, quantised to 16 counts (offline for training, online at eval)

Closed-loop, 30 episodes per tier (crush latch as in the scripted study):

| tier | base | q16 | delta |
|---|---|---|---|
| success, no limit | 4/30 | 5/30 | 2/30 |
| **crushes @ 120 N** | **14** | **14** | **5** |
| **crushes @ 60 N** | **21** | **21** | **12** |

Two findings, one null, all pre-registered in direction:

1. **The channel halves over-force events in a cloned policy.** On video the
   mechanism is unmistakable: the base policy squeezes blind -- 1219 N on one
   episode, 28 N on the next, the regression-to-the-mean of close depth over
   block widths -- while the delta policy touches at 0-36 N peak. For ACT the
   input is not a derived feature (the architecture never sees a[t-1]), so
   this is the channel supplying genuinely new information at the input.
2. **Sixteen-count resolution erases the effect exactly** -- q16's crush counts
   match base digit for digit (14/14, 21/21). The learned-policy mirror of the
   scripted resolution cliff, at the same quantum, by an independent method.
3. **No input fixes task success** (2-5/30 everywhere): both informed and
   blind policies stall at the grasp-to-lift commitment. This is the
   demonstrated coverage gap -- the demos' trajectory manifold holds 92% of
   its variance in 2 PCs and contains no recovery behaviour, so any deviation
   at the contact boundary is out of distribution. A perturbed-collection
   pass (expert corrections after injected kicks) is the identified fix and is
   future work, not folded into this result.

Reference bar, measured: the scripted expert with a noisy pose estimate
(sigma = 5/10/20 mm, modelling a perception front-end) places 20/14/4 of 30 --
so the deployable scripted system sits at 47-67%, which none of the BC
policies approach. On this task, at this data scale, script-plus-perception
remains the stronger engineering; the learning contribution is the mechanism
evidence above.

## The guard wrapper: safety as architecture, on real learned policies

The same three checkpoints, identical paired episodes, raw versus wrapped in
the ~30-line jaw guard (stall-detected seating + a 4-count squeeze cap; only
bus-observable signals, no retraining, no object knowledge). Crush counts per
30 episodes:

| arm | tier | raw | guarded |
|---|---|---|---|
| base | 120 N | 13 | **1** |
| base | 60 N | 15 | **1** |
| delta | 120 N | 6 | **0** |
| delta | 60 N | 13 | **0** |
| delta_q16 | 120 N | 16 | **0** |
| delta_q16 | 60 N | 22 | **1** |

**Totals across all crush tiers: 85/180 raw, 3/180 guarded -- 96% of crush
events eliminated** on identical checkpoints and episodes. Median peak force
drops from 33-131 N (arm-dependent) to 13-26 N wrapped. The channel and the
guard stack: delta's learned gentleness halves what base does, and the guard
zeroes the remainder.

Two honest caveats, stated with the result:

* **Task success stays at the floor either way** (0-2/30 wrapped): the guard
  bounds force, it does not teach the grasp-to-lift commitment the
  demonstrations never covered.
* **The guard removes successes a policy earned by over-gripping.** The q16
  arm lifted 7/30 at the no-limit tier by squeezing at a 131 N median; capped
  at seat+4 counts those successes vanish. A force bound is universal -- it
  also binds force a sloppy policy was exploiting as a crutch.

The closing statement of the simulation study: **on a position-servo gripper
with no force sensor, gentleness can be learned from the tracking-error
channel at native resolution, and safety is better made architectural -- a
bus-observable guard that transfers to any policy, at a cost of 30 lines.**

## Autopsy: why BC sits at 5-17%, with causal attribution (results/autopsy/)

Teacher forcing: both arms predict ~97% (relative) on demo states in EVERY
phase -- the networks learned the demonstrations. Closed loop, base's decoded
plan at its own stalls contains 6 counts of motion against the demo lift's
471: no intention, not weak execution. Three data recipes moved every stage
except base's commitment (lift|seat 16 -> 24 -> 22%).

The counterfactual seals causality: at delta-policy stalls, zeroing the delta
input collapses the planned lift 482 -> 35 counts (14x); restoring a
demo-typical value raises it to 566. **The commitment decision is gated on
the channel inside the trained network.** Grasp state is unobservable from
joints + 96 px images; the tracking-error channel is the observation that
makes it visible. Covariate shift is real but secondary and largely cured by
the v2/v3 data recipes where it applies.

Consequence for the thesis: the channel is not merely helpful for learned
policies -- on this stack it is the enabling observation for contact
commitment, shown by intervention on the trained model, not correlation.

## Scale run (H100): the ladder tops out at 57-60%, and the control decides the framing

600 depth-diverse + kicked demonstrations, 224 px, 50k steps, batch 64 — the
standard recipe this project had been under-running by ~6x. Four containers,
~$26 of credits, evals run locally (n=100 per cell, Wilson CIs):

| policy | no-limit success | pooled over 2 seeds | 120 N tier |
|---|---|---|---|
| delta | 63 / 51 | **57%** | 34 / 11 (43-59 crushed) |
| base_hist (raw a[t-1], no subtraction) | 60 / 59 | **60%** | 18 / 10 (61-68 crushed) |

The success ladder end to end: 5 -> 17 -> 21 -> **57-63%**, now inside the
deployable scripted baseline band (47-67% = expert + realistic pose noise) and
approaching the privileged ceiling (83%). Picks reached ~85-90%; the residual
leak is transport retention, as the funnels predicted.

**The base_hist control: indistinguishable from delta at this n.** Pooled
60% vs 57%, but per-seed the cells are 60/59 and 63/51 -- the within-policy
seed spread (12 points) is four times the between-policy gap, and two seeds
cannot separate a 3-point difference. The honest statement: at scale, raw
action history and the explicit tracking-error form perform equivalently
within our statistical resolution; the causality question (information vs
representation) is NOT resolved by this cell and would need ~5+ seeds per arm.
What stands regardless, from the small-scale ladder: policies without any form
of action-history input plateau at 16-24% lift-commitment across three data
recipes, while channel-bearing policies reach 47-76% -- the *information* is
necessary; which *form* it takes appears not to matter at scale and was only
observed (not isolated) to matter at low compute.

**Competence arrived gripping hard:** every scaled policy crushes heavily at
the 120 N tier (43-68/100) -- the depth-diverse demos teach grips up to
~100 N and the policies imitate the distribution. Safety therefore stays
architectural; the guard-wrapper integration on the scaled checkpoint is the
final experiment.

## Phase 3 Task 1 -- corpus study: the pre-registered prediction is falsified, informatively (2026-08-16)

Frozen protocol in `docs/phase3.md` (committed before the run); one script
(`scripts/corpus_delta_outcome.py`), one figure
(`figures/paper/fig1_corpus.png`). Sample: 16 public SO-100/101 teleop
datasets scored of the spec's 20 -- the 555-repo candidate pool plus frozen
gates (>=20 episodes with a close event, median one close per episode, both
outcome classes >=5) plus intermittent Hub connectivity from the bench
support no more; documented, not padded.

**Pre-registered:** successes carry HIGHER delta-at-seat; Cohen's d > 0.5 in
>=10/20 datasets. **Observed: 2/16.** Pooled within-dataset-z d = -0.39
(546 episodes, 283 proxy-successes). Per-dataset: 6/16 significantly
negative (bootstrap 95% CI below 0), 2/16 significantly positive (above 0),
8 null. The prediction is dead; the channel is not: |d| > 0.5 in 9/16
datasets. The sign, not the information, was wrong.

**What the traces say happened.** Teleop operators over-command every close
(the leader lever has no hard stop), so the seated flag fires in ~100% of
episodes in 14/16 datasets -- empty jaws stall at the mechanical stop just
like full ones, and delta-at-seat cannot report object presence. What it
does report: successful grasps are STEREOTYPED -- in `ball_in_cup` every
clean episode rests at the same ~7-count gap (the channel reading the same
grasp identically; the low-variance side of Cohen's d, which is why two
task-repetitive datasets blow out to d ~ -16/-20) -- while failure episodes
contain deep-jam frames where the operator forces 40-80 counts against a
stalled jaw. In the wild, high delta at grasp time is a STRUGGLE signal,
not a success signal.

**What dies:** the strongest retroactivity claim -- "delta at grasp predicts
episode outcome across the public corpus" -- and with it any plan to mine
the corpus for success labels via this channel alone. **What survives:**
(i) the channel is recoverable from every position-only log and carries
outcome-relevant signal (9/16 at |d|>0.5) whose sign is task-dependent;
(ii) the struggle/anomaly reading aligns with the enforcement use -- the
same high-delta events the corpus associates with failure are what the
guard caps; (iii) the design claim (Task 2) never depended on this
prediction and stands on the sim causal chain. Known limits, frozen with
the protocol: the success proxy is unvalidated against ground truth (no
labels exist in the corpus); multi-grasp tasks are excluded by design;
n=16 not 20.

## The guard, paired (2026-08-16, post Rule 1) -- wrapper-alters-plant, three ways

Rule 1 (docs/phase3.md) arrived after four phantom-seat modes were excavated
one GPU run at a time; applying it retroactively took two CPU-minutes per
question and settled everything the GPU runs could not.

**The pairing** (`scripts/eval_expert_guarded.py`, seed 3000, identical
protocol to the policy A/B): the closed-loop force-mode teacher through
guard v4 (net-stall OR lag-excess detection; 12-count ~= +24 N force budget
past seat; 4 counts/frame closing rate limit) scores **25/30 placed vs
28/30 raw** at crush-off, grip 39 N both -- the guard costs a competent
controller ~3 episodes in 30 (the deleted-success accounting, now measured
on a working reference). At 120 N: 23 vs 26. **The guard is compatible with
competent closed-loop control.**

**Integration note (not a finding; demoted per Rule 1c review):** the
open-loop clamp expert collapses 30/30 -> 0/30 under the rate limiter
because its fixed-duration grasp window expires before the slowed jaw
arrives -- which is what a rate limiter does to open-loop timing, i.e.
definitional, not a boundary. Kept as an integration checklist item:
controllers wrapped by the guard must be closed-loop on contact, and any
internal stall heuristics must use feasible (not commanded) travel.

**Unimplemented feature (not a boundary; demoted per Rule 1c review):**
at the 60 N tier the teacher fails raw AND guarded (3/30) on contact-entry
transients. The standard fix is a soft-start / approach-velocity clamp
before contact -- routine in industrial grippers -- which guard v4's
closing-rate limit approximates but does not yet specialise near contact.
Filed as a guard feature ticket, not written as a limit of action filters.

**The policy ticket (NOT a finding):** the scaled delta policy under the
v3 guard scored 0/30 while gripping ~40 N. Filed as a defect per Rule 1:
the pairing proves the guarded task feasible; the counterfactual
(scratchpad/counterfactual_commit.py: demo-typical -32-count delta injected
at latch while the guard held ~40 N) FAILED to restore lifts (3/30 picked,
1/30 placed ~= baseline), killing the "policy waits for its learned 77 N
commitment feeling" hypothesis. Open. Next actions on the ticket: rerun
the A/B against guard v4 (the guard under which the reference was proven);
if still ~0, probe the state channel (the clamped jaw POSITION in s[t],
not the delta feeling, may be the out-of-distribution input).

Fix 2 from review (train through the guard) is now wired:
`PickScene.jaw_guard_factory` filters every `hold()` at the choke point
where actions are recorded, so guard-collected demos carry applied
(capped) actions by construction.

## Fix 2 at small budget: floor-equal twins (2026-08-17, one seed each)

Trained-through-guard delta (demos_v3g: the v3 recipe collected with guard
v4 filtering hold(), 200 eps) vs its exact unguarded twin (first 200 eps of
demos_v3), both 12k steps / 96 px / seed 0, both evaluated raw and guarded
at seed 3000: **raw 2/30 vs 3/30, guarded 0/30 vs 0/30.** The pair is
floor-equal -- and floor is the correct prior for this budget class (the
pre-scale ladder's place rates were 5-21% across every recipe; the 43-60%
figures belong to the 600-demo/50k-step/224px budget and are not a valid
reference here, a comparison this entry corrects).

Verdict, Rule-1-compliant: shielded-imitation collection neither helped nor
hurt where the baseline itself is floor -- the fix-2 question is
UNANSWERABLE at this budget and defers to (a) the Task 2 grid, whose D/E/F
arms exist precisely to move the small-data floor and which now carry the
trained-through-guard column, or (b) the H100 budget class on guarded data.

Logged as observation, not claim (n=1 seed): the guarded-data policy did
not inherit its demonstrations' force bound -- trained on <=40 N grips it
still crushed 15/30 raw at the 60 N tier (twin: 10/30) and gripped 52 N
median (twin: 45 N). Demonstration-level force bounds do not bound the
clone; enforcement stays architectural, consistent with the scaled-policy
result.

## Gate 1 closed: instrumentation audit and baseline reproduction (2026-08-17)

Ordered by the external review after a floor-level twin was misread against
an H100-class reference. Every cell measured, none argued:

| question | instrument | result |
|---|---|---|
| are the eval episode streams equal? | harness-free teacher, both seeds | 24-28/30 both; seed-3000 not harder |
| do the two harnesses agree at floor? | twin ckpts, both harnesses | 2-4/30 everywhere |
| do they agree at a non-floor point? | H100 ckpt, same seed 3000, both | **15/30 vs 13/30 -- match** |
| did training/env code change? | diff vs historical tree | only inactive-by-default flags; expert.py untouched |
| does the 21/100 cell reproduce? | full-280 retrain, seeds 0,1 | **11/100 and 31/100 -- brackets 21** |

Verdict: no regression anywhere; the historical 21% was a single unseeded
draw from a distribution whose per-seed spread at this budget is ~20 points
at n=100 -- larger than the 12 points documented at H100 scale, and never
measured at small budget until now. The "5x baseline collapse" decomposed
into: a reference-class error (H100 rows quoted against small-budget
cells), a 200-vs-280-episode subset difference, and this seed variance.

Grid-design consequence, binding for Task 2: with sigma_seed ~ 10 points,
3 seeds x n=100 resolves arm differences of roughly >=15 points and nothing
finer. Grid claims are restricted accordingly; nothing is claimed at n=30.

Seed 2 completes the spread: **{11, 25, 31}/100** (mean 22.3, sigma ~10.3);
the historical 21 sits inside. These three runs double as Task 2 grid arm C
(delta_input) x 3 seeds -- the grid's channel row is done, and its scale
note stands: differences under ~15 points are below this budget's
resolution. Grid dataset declared as FULL demos_v3 (280 episodes): the
instruction's "200-episode demos_v3" miscounted the set, and 280 is the
config whose baseline is reproduced.

**Retroactive scope note (2026-08-17, after sigma_seed was measured):** with
per-seed spread ~10 points at n=100 (and larger at smaller n), every
two-seed comparison in this repo is downgraded to "not resolved": the
H100 base_hist-vs-delta cell (already softened) and any tournament-era
"information not representation" phrasing are readings of noise at that
resolution. Claims stand only where the gap exceeds the measured spread
(e.g., channel-bearing vs channel-free arms, 16-24% vs 47-76% conditional
lift -- a 30+ point gap).

## Task 2 main grid complete (2026-08-18): the design result, at resolution

Six observation designs, identical everything else (280 demos, 12k steps,
96 px, 3 seeds x n=100; sigma_seed ~ 10 -> only >=15-point differences are
claimed). Figure: figures/paper/fig2_grid.png.

| arm | input | seeds | mean |
|---|---|---|---|
| A base | s[t] | 0, 15, 4 | 6.3 |
| B base_hist | s + a[t-1] | 4, 15, 15 | 11.3 |
| C delta | s + (a[t-1]-s) | 11, 25, 31 | 22.3 |
| D resid | s only; delta as aux target | 8, 12, 6 | 8.7 |
| E excess | s + (delta - lag baseline) | 36, 32, 21 | **29.7** |
| F token | s + latched seat bit | 29, 33, 18 | **26.7** |

Pre-registered predictions (docs/task2_arms.md), their fates:
- "D/E/F >= B if the channel's value is information": E-B = +18.4
  (RESOLVED), F-B = +15.4 (at the floor, resolved marginally), D-B = -2.6
  (FAILED). The split is the finding: **the channel must be observed, not
  merely predicted** -- an auxiliary delta-prediction head adds nothing
  (D ~= A), while the same information at the input moves the floor.
- "E sharpest if the motion confound is the small-data blocker": E is the
  top arm. Subtracting the free-motion lag baseline (self-labeled, frozen
  pre-training) is the best-performing form -- consistent with the
  characterization's motion-confound finding.
- B ~= C at scale carried down: C-B = 11.0, UNRESOLVED at 3 seeds, exactly
  the seeds45 question, along with E-C = +7.4 (whether the designed form
  beats raw delta is suggestive, not claimed).

Resolved sentences the paper may carry: a designed form of the channel
(E, F) beats raw action history at small data; the channel as input beats
the channel as training signal (E,C >> D); channel-bearing inputs beat
channel-free (E-A = 23.4, C-A = 16.0). Texture logged, no claim: F picks
at the highest rates in the grid (76-84 gripper-successes/100) and drops
heavily -- the seat token drives commitment while retention lags.

Cost note: dry + main = 9 H100 cells ~= $27 actual vs $23 estimated.
Guarded column running locally (Cg s0 = 14 vs C s0 = 11).

## Grid at 5 seeds (2026-08-18, seeds45 phase, ~$36): the verdicts firm up

A {0,15,4,3,9} 6.2 | B {4,15,15,10,25} 13.8 | C {11,25,31,12,30} 21.8 |
D {8,12,6,1,6} 6.6 | E {36,32,21,17,27} 26.6 | F {29,33,18,9,20} 21.8.
Welch t at n=5 seeds x 100 episodes:

RESOLVED: E-A +20.4 (t=4.7), C-A +15.6 (t=3.1), F-A +15.6 (t=3.1),
**E-B +12.8 (t=2.6, p~0.03)** -- the design headline survives the seed
expansion. D-A +0.4 (t=0.1): the aux-target arm is DEAD EQUAL to base at
five seeds; predicting the channel instead of observing it contributes
nothing. NOT RESOLVED: C-B +8.0 (t=1.4) -- raw delta does not separably
beat raw history at this budget; F-B +8.0 (t=1.5) -- F's 3-seed marginal
call collapsed exactly as its at-the-floor flag anticipated; E-C +4.8
(t=0.9) -- whether E's edge is the subtraction or delta at all is not
attributable here.

The paper's three resolved sentences: (1) channel-bearing observation
beats channel-free; (2) only the confound-subtracted form resolvably beats
raw action history; (3) the channel must be observed, not predicted.
Pre-registration discipline paid twice in one table: F's marginal claim
was flagged marginal and fell; E's was flagged resolved and held.

## Roadmap C.1 -- offset sweep (2026-08-18, CPU): the corpus result is
alignment-robust, and the true alignment is k~3-4

Sweeping delta_k = a[t-k] - s[t] over k=0..5 on the 16 scored datasets:
free-motion |delta| is minimized at **k*=3 for 10/16 and k*=4 for 4/16**
(~100-130 ms command-to-state at 30 fps) -- the protocol's k=1 convention
was not the physical alignment. But the Sec 4.3 conclusion does not care:
**median Cohen's-d span across the whole sweep is 0.12** -- signs and
magnitudes hold for 14/16 datasets (the two movers: one k-insensitive
flat-motion set drifting -0.85 to -0.40, and the degenerate low-variance
stacking set whose |d| inflates meaninglessly). The bench latency
measurement (roadmap A) pins k physically; retroactive training on real
logs (C.2) should build its lag baseline at the measured k, not k=1.

## Roadmap E.1 -- compensator ladder, offline (2026-08-18, CPU)

Free-motion predictors of delta, fit on the episode-split train portion of
jaw-open frames, held-out RMSE per joint: the paper's scalar velocity model
(M1, K*r) is within 0.1-4% of an intercept model, a direction+accel linear
model, and a 4-tap FIR on every joint (jaw: best richer model improves on
M1 by 1.9% of residual). Per the roadmap's decision rule: richer
compensators saturate -> linear was enough; arm E' is not warranted.
Caveat recorded: the comparison domain is free-motion frames (the fit
domain), where the jaw's rate is small -- the ladder ranks model families,
and all families tie, which is the evidence sought; it does not measure
closed-loop policy impact, which is arm E itself.

## Rev-3 desk items (2026-08-18 evening)

- **K-hat threshold sensitivity (NEW, P2): closed.** Refit at free-motion
  criterion percentiles 10/20/30/40/50: per-joint K moves <= 0.09 (jaw
  0.80-0.89, joint 0 0.78-0.79). The criterion is not a tuned knob. (Note:
  the smoke-test K values quoted earlier came from a 12-episode subset;
  full-set values are the canonical ones.)
- **k-sweep timing figure (NEW, P1): built** from the C.1 data ->
  figures/paper/fig_ksweep.png (residual minimum at k*~3-4; d flat, span
  0.12).
- **Compensator ladder status sync:** linear family ran (saturates at K*r,
  jaw +1.9%); the spec's small-MLP variant still to add.
- **Venue dates verified:** CoRL 2026 main deadline passed (May 25/28);
  conference Nov 9-12 Austin, workshop day Nov 9 -> CoRL-26 workshop CFPs
  are the imminent rule-6 target; "CoRL main track" = CoRL 2027 (~May).
  Two-track plan: workshop ship first, rev-3 A+B+D program aims at 2027.

## Guard detector is inert on the gentle teacher; "guarded" datasets are
replications (2026-08-19, ~01:00, Rule-1 chase of Eg s0 = 7)

Evidence chain: Eg s0 trained normally (loss 0.059) with sane K_hat;
demos_v3g280's hold-phase |delta| (median 30) and command floors are
IDENTICAL to demos_v3 -> no capping occurred; live instrumented episode:
guard armed, seat NEVER latched on the force-mode teacher (its fine pass
moves ~0.3 counts/frame, under both the net-close and rate gates; persist
compounds it). Re-reading the old pairing table confirms the guard was
inert there too (39.6 N raw vs 38.9 N "guarded"): the "3-in-30
interference" was seed noise around zero enforcement.

Consequences: (1) demos_v3g-* are plain re-collections -> the "guarded
column" is a COLLECTION-REPLICATION column; Ag/Bg/Cg/Dg matching their
twins is a replication result. (2) Trained-through-guard remains UNTESTED
(needs a detector variant sensitive to gentle closes). (3) The paper's
guard section corrected: enforcement claims stand where the detector
fires (policies 99->40 N; clamp 360->10-28 N; crush A/Bs); the
teacher-pairing row now reported as measuring wiring overhead only, the
threshold insensitivity stated as a property with its ambiguity named.
(4) Floor-twin result relabeled replication; conclusion unchanged.
(5) OPEN AND URGENT: Eg s0 = 7 is now a fresh-collection replication draw
of the headline arm, ~2 sigma below E's mean. Eg s1/s2 decide whether E
carries a collection-fragility caveat. Nothing written until they land.

Main grid, corpus study, and channel characterization are untouched: the
guard appears nowhere in those pipelines.

## Eg complete: the mixed branch (2026-08-19 ~06:50)

Eg = {7, 12, 27}, mean 15.3, vs E = {36, 32, 21, 17, 27}, mean 26.6.
Welch t = 1.6 -> UNRESOLVED; s2 = 27 sits squarely in E's range, killing
the fragility-catastrophe branch. Replication column vs originals:
A +0.8, B +3.9, C -8.5, D +0.4, E -11.3 -- nothing resolves individually,
AND the fresh collection's point ordering at 3 seeds (Bg 17.7 > Eg 15.3 >
Cg 13.3) does not reproduce the original. Verdict written into the paper
(App C + Limitations): collection-level resampling is a variance
component ABOVE sigma_seed that the headline does not yet average over;
E-B stands as published on its declared dataset; roadmap D upgraded --
the 20-seed expansion must SPAN BOTH collections. The fresh dataset
itself survived three forensic checks (delta stats, command floors, rate
distributions identical); the teacher's own closing speed (4 counts/step)
sits below the rate limiter, so the "guarded" collection was clean stock.

## Replication column complete (2026-08-19 12:19): Fg {11,18,7} = 12.0

Full table (original 5-seed mean -> fresh-collection 3-seed mean):
A 6.2->7.0, B 13.8->17.7, C 21.8->13.3, D 6.6->7.0, E 26.6->15.3,
F 21.8->12.0. All three channel arms fell (8.5-11.3), all three
channel-free arms rose slightly; nothing resolves individually (max
|t|~1.6). Fourth forensic check on the collections (per-episode depth
diversity: spread 25.0 counts BOTH, std 9.2 vs 9.0) -- collections are
marginally indistinguishable across four independent statistics, so the
pattern is recorded as UNATTRIBUTED collection-level variance; note also
regression-to-the-mean (original channel arms' seeds 0-2 drew high; their
seeds 3-4 sit near the replication values). Paper states it at exactly
this strength (App C + Limitations). D's design requirement stands: seed
expansion must span both collections.
