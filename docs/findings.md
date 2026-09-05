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

## Roadmap H complete (2026-08-19): shift probe favors regulation over script

Best-local-seed checkpoints, n=50/condition, orderings only (frozen
protocol in scripts/eval_shift.py, committed pre-run). Success/50 across
{nominal, mass x0.5, mass x1.5, friction x0.5}: A 7/7/7/7 (14 drops every
time -- outcomes byte-identical under physics shifts: visual-script
signature); B 9/9/10/9; C 14/19/15/18; E 14/14/17/13. Pre-registered
question answered: the channel arms' advantage PERSISTS under object-
property shifts (no collapse toward A; orderings preserved), the behavior
signature of contact regulation. crush_60N: all arms 0/50 (37-40 crushes)
-- no differentiation; enforcement remains architectural. E-vs-C not
probed (best-seed n=50 below resolution; not claimed).

## Oracle ceiling arm complete (2026-08-22): E is at the ceiling

Privileged TRUE grip force as input (fixed 40 N scale), same demos_v3,
3 seeds: O = {15, 33, 37}, mean 28.3 +- 11.7. O-E = +1.7 (t = 0.23):
indistinguishable. O-A = +22.1 (t = 3.05): the ceiling is real and far
from base. Reading: the lag excess recovers essentially all of what a
perfect contact observation delivers on this task at this budget -- the
residual is close to a sufficient statistic; D's null sharpens (even the
true signal helps only as an INPUT). Caveat: 3 seeds, one collection.

## Guard Pareto accounting (2026-08-23): the guarded configuration never succeeds

Desk work, no new runs (roadmap :219-220). scripts/guard_pareto.py pairs the
guarded/unguarded rows already in results/guard_ab.json (3 checkpoints x 3
crush tiers x 30 paired episodes) and reports the columns the roadmap asked
for. Independent recomputation CONFIRMS the paper's headline: 85 -> 3 crushes
of 180 across the two crush-capable tiers, 96% reduction.

Two accounting issues surfaced, neither a wrong number:

(1) Guarded success is 0/30 in ALL NINE cells. Totals across all three tiers:
12/270 unguarded -> 0/270 guarded. App C discloses one slice of this (the
quantized arm's 7/30 at a 131 N median) but not the aggregate; five further
successes vanish in cells the text does not mention (delta 2 at no-crush, 2 at
120 N; delta_q16 1 at 120 N). Under the project's own null rule (0/N is a
defect until proven a boundary) a clean 9-for-9 zero is the pattern that rule
exists to catch. Mitigating context, which belongs next to it: the unguarded
policies are themselves weak here (12/270 = 4.4%), and the base arm scores 0
guarded AND unguarded -- so this is "the guard removes the few successes that
existed", not "the guard destroys a working policy". Not yet distinguishable
from a boundary; needs the pairing protocol before it can be called either.

(2) Mixed denominators in one sentence. "85 to 3 of 180" is computed over the
crush-capable tiers only (6 rows x 30). The peak-force range quoted in the same
breath, "33--131 N to 13--26 N", is computed over ALL nine cells -- the 131 is
the delta_q16 no-crush row, a tier where crushing is impossible. Restricted to
the same 180 episodes the crush claim uses, the range is 33--92 N -> 13--26 N.
Both ranges are individually correct; the sentence reads as if one denominator.
Appears in main.tex sec:guard and again in App C. Reviewer bait; costless to fix.

False interventions (guard deletes a success in the no-crush tier, where it had
nothing to prevent): 9 of the 12 lost successes. Deletion counts are net per
cell, so they are lower bounds -- the stored rows carry aggregate counts, not
episode-level pairing.

## Real teleop, 50 episodes (2026-09-03): Present_Load saturates through contact

First real-hardware demonstration set on the SO-101 pair: 50 teleop episodes of
adapter -> centre of tape roll, 21,476 frames, 155 MB. Recording captures
observation.state as 6 positions PLUS 6 Present_Load, so delta and the
reviewer's baseline are logged simultaneously with no extra work.

The contact signature is present in every episode: all 50 have frames where the
jaw command keeps closing while the jaw does not move (median 24 such frames,
range 14-44). |delta| on the jaw peaks at a median of 16.6 deg.

The result worth having: **Present_Load pins at 500 in 16.5% of frames, and all
50/50 episodes touch saturation.** It is not spread evenly -- it coincides with
contact, where |delta| averages 12.35 deg against 2.09 deg elsewhere. Inside
those frames the register reports one constant number while delta still spans
8.01-31.01 deg, i.e. 23 deg of range the baseline cannot express. Where load is
not clipped the two track tightly (corr -0.853 unsaturated, -0.922 overall),
which is the affine relation of the static bench data reappearing on real data.

This sharpens the Appendix D argument from redundancy to range. The static data
(scripts/appendix_d_channels.py) says load cannot beat delta because it IS delta.
This says load additionally stops being delta exactly at the forces a grasp
produces. Reviewer's baseline answered from two directions.

CAVEAT, unresolved and load-bearing: 500 is almost certainly the servo's
CONFIGURED torque limit (addr 48), not the register's full scale -- Feetech
document Present_Load as full-scale 1000 = 100% duty. If so the honest claim is
"under this arm's torque limit", a deployment fact, not a ceiling of the
register. Read addr 48 on the follower before this goes in the paper; it is a
one-line register read and it decides how strongly the paragraph can be written.

Not claimed: nothing here is a policy result. These are demonstrations, not
evaluations, and no arm has been trained on them.

## The 500 ceiling is lerobot's, fleet-wide (2026-09-03)

Follow-up to the saturation entry above, and it upgrades that finding from a
property of our arm to a property of the stack.

Read of addr 48 (Torque_Limit) on the follower, read-only, arm idle:

    joints 1-5 (pan, lift, elbow, wrist_flex, wrist_roll) : 1000
    joint 6    (gripper)                                  :  500

Present_Load is full-scale 1000, so 500 is a configured cap, not the register's
ceiling -- and it is set on the gripper alone. lerobot writes it, unconditionally,
for every SO-100/SO-101 follower it connects
(robots/so_follower/so_follower.py:172-175, in configure(), called from
connect():110):

    Max_Torque_Limit   = 500   # 50% of max torque to avoid burnout
    Protection_Current = 250   # 50% of max current to avoid burnout
    Overload_Torque    = 25

Three consequences, in increasing order of how much they help the paper:

1. Our 16.5%-of-frames saturation is not a local misconfiguration to apologise
   for. It is what every SO-100/101 gripper does under this stack.
2. The SAME block halves Protection_Current, so the second reviewer baseline is
   capped by the same three lines. The two obvious alternatives to delta are
   both clipped, on the one joint where manipulation makes contact, by the
   library everyone records through.
3. delta requires no register and no limit. It is untouched by all three writes
   because it is arithmetic on numbers already in the log.

This is the strongest form of the reviewer's-baseline answer so far, and it is
ecosystem-wide rather than n=1: not "load happens to be worse on our bench" but
"the fleet's own default caps load and current through contact, and the channel
that survives is the one nobody is recording."

Honest limits. The current half is INFERRED from the configuration write, not
measured: our dataset logs position and load, not current. The corpus study's
datasets record neither, so this cannot be checked against them -- it is a claim
about the recording stack, not about corpus contents. And burnout protection is
a good reason for the cap; the point is not that lerobot is wrong, it is that a
sensible safety default silently costs the baseline its range.

Worth one confirmation before it ships: read addr 48 on a second SO-101 that has
been through lerobot connect, to show the value is written rather than inherited
from how this particular arm was flashed.

## The declared primary decomposes into two unresolved halves (2026-09-03)

Desk work, no new runs, no new data — arithmetic on the frozen 5-seed grid
cells in `results/grid_*_s{0..4}.json`. Surfaced while `projects-a5` was
auditing arm E's inputs; the two findings are one finding.

**What E actually consumes.** `train_act.py:118-131,190` computes

    rate[t]   = a[t-1] - a[t-2]                 (a COMMAND rate, not q̇)
    excess[t] = (a[t-1] - s[t]) - k_hat * rate[t]

so arm E's input depends on **s[t], a[t-1] and a[t-2]**, while B ([s, a[t-1]])
and C ([s, a[t-1]-s[t]]) depend only on s[t] and a[t-1]. k_hat is load-bearing,
not a rounding term: per-joint [0.786, 0.763, 0.837, 0.838, 0.744, 0.801] from
`checkpoints/modal_E_s0/excess/norm_stats.npz`, so ~80% of the rate is
subtracted. `roadmap.md:10-14` already recorded that the implementation uses the
command rate g[t−1] − g[t−2]; what nobody wrote down is the consequence — **E is
a two-action-frame arm and B is a one-action-frame arm**, so the paper's declared
primary comparison confounds the free-motion subtraction with an extra frame of
action history.

**The confound is already measured, and it is the smaller half.** C is exactly E
without the a[t-2] information, so the existing cells decompose the headline:

| comparison | value | Welch t (n=5) | what it isolates |
|---|---|---|---|
| **E − B** | **+12.8** | **2.61** | declared primary; confounded |
| E − C | +4.8 | 0.86 | the k_hat·rate correction, i.e. the a[t-2] frame |
| C − B | +8.0 | 1.44 | same information {s, a[t-1]}, different representation |

+4.8 and +8.0 sum to +12.8 exactly. So the one pair in the grid that clears the
project's resolution bar **is significant only as the sum of two components that
are each individually unresolved**, and neither component is separately claimable
at this seed budget. That sentence is owed to the reader regardless of what any
future cell returns, and it costs nothing to establish.

Read carefully, this calibrates the alarm rather than raising it. The extra-frame
contribution is bounded by E−C = +4.8 of the +12.8, and the larger half (C−B =
+8.0) is a clean representation test at matched information. The confound is real
and disclosable; it is not evidence that the headline is an artefact.

**Untouched by any of this:** E−A = +20.4 (t=4.7) and C−A = +15.6 (t=3.1). The
channel-versus-nothing result does not depend on the decomposition.

**The control the grid lacks.** B2 = [s[t], a[t-1], a[t-2]] — the same raw
information as E, presented raw. E − B2 is the value of the fitted subtraction
with information held fixed, which is what the title claims. Caveat to price
before reading the result: B2 is 18-dim where B, C and E are all 12-dim, so it is
information-matched but not width-matched, and at 280 demonstrations the extra
width can cut either way. E ≥ B2 would therefore be strong; E < B2 would be
ambiguous between representation and width.

## Grid-wide information audit: only one arm pair holds content fixed (2026-09-03)

Follow-on from the E/`a[t−2]` entry above. If the declared primary was
confounded by an information asymmetry, the right response is to audit every arm
rather than patch one comparison. Desk work on `scripts/train_act.py:118-232`
and `src/so101_bench/guard.py:78-117`; no new runs.

**What each arm's observation actually depends on:**

| arm | input | raw information consumed | mean |
|---|---|---|---|
| A base | s[t] | {s[t]} | 6.2 |
| D resid | s[t] (δ as aux *target*) | {s[t]} at input | 6.6 |
| B base_hist | [s, a[t−1]] | {s[t], a[t−1]} | 13.8 |
| C delta | [s, a[t−1]−s[t]] | {s[t], a[t−1]} | 21.8 |
| E excess | [s, δ − K̂·r] | {s[t], **a[t−1], a[t−2]**} | 26.6 |
| F token | [s, seat[t]·𝟙₆] | **{s[0..t], a[0..t−1]}** — see below | 21.8 |

**F consumes unbounded history, and by more than E does.** `train_act.py:158-173`
runs the frozen JawGuard detector causally over each episode; `guard.py:99,114`
sets `seat_jaw` under `if self.seat_jaw is None` and never resets it. The latch
is single-shot, so `seat[t]` is monotone non-decreasing within an episode and
encodes "a stall was detected at **any** t′ ≤ t". Detection itself is local
(`window=4`, `persist=3`), but the latch makes F's input a function of the whole
episode prefix. F is the longest-context arm in the grid by a wide margin.

**The consequence for the paper's framing.** The grid is presented as an
observation-*design* study — which *form* of the channel a small-data policy
needs, content held fixed. Sorting the arms by information consumed gives
A = D < B = C < E < F. Of all the pairs the paper draws design conclusions from,
**exactly one holds raw information fixed and varies only representation: C − B**
(both see {s[t], a[t−1]}). Its value is +8.0, t = 1.44 — **unresolved**. Every
other design comparison in the grid varies content as well as form.

**But the story is not simply "more information wins", and F is why.** F consumes
strictly more than E and does not beat it (21.8 vs 26.6), and F−B = +8.0 (t=1.5)
is no larger than C−B despite F's unbounded prefix. So performance does not track
information monotonically, which is evidence against reading the whole grid as an
information ladder. The honest summary is narrower and worse for the design
claim: the grid resolves *content* questions well (E−A = +20.4 t=4.7; C−A = +15.6
t=3.1; D−A = +0.4 t=0.1, the observe-vs-predict result, which is clean because D
and A are information-matched at the input) and resolves *form* questions
poorly, because it was not built to hold content fixed.

**What this does and does not change.**
- Untouched: channel-versus-channel-free, and the observe-vs-predict result
  (D ≈ A) — D's input is information-matched to A's, so that comparison is clean
  and it is the grid's strongest design finding.
- Weakened: any sentence of the form "form X of the channel beats form Y",
  because only C−B is a matched form test and it does not resolve.
- Arm B2 (`task2_arms.md`, frozen 2026-09-03) is the fix for E. There is no
  matched control for F in the grid or planned, and F's latch means building one
  would require a non-latched seat bit — worth noting, not worth a cell yet.

## The guard section of main.tex has not absorbed the 2026-08-23 Pareto entry

Re-verified `results/guard_ab.json` independently today (18 rows, 9 paired
cells). The numbers reproduce the 2026-08-23 entry exactly:

    successes  unguarded 12/270 -> guarded 0/270, and 0/30 in ALL NINE cells
    crushes    85 -> 3 of 180 on the six crush-capable rows (96%)
    peak force median, all 9 cells        : 33-131 N -> 13-26 N
    peak force median, the same 180 eps   : 33- 92 N -> 13-26 N

Two statements in `main.tex:438-457` still conflict with this, both flagged on
2026-08-23 and both still present:

1. **"it also deletes successes one policy earned by over-gripping."** It is
   two policies (delta and delta_q16), five cells, and *every* success in the
   experiment: 12/270 → 0/270, 0/30 in all nine cells. "One policy" is not a
   softening of the result, it is a different result. The mitigating context
   belongs in the same sentence and is enough on its own — the unguarded
   policies are weak here (12/270 = 4.4%) and base scores 0 guarded *and*
   unguarded, so this is "the guard removes the few successes that existed",
   not "the guard destroys a working policy". Say that; it survives review and
   the current phrasing does not.
2. **Mixed denominators.** "from 85 to 3 of 180 (96%)" is computed over the six
   crush-capable rows; "33--131 N to 13--26 N" in the same sentence is computed
   over all nine cells, and the 131 is the `delta_q16` no-crush row where
   crushing is impossible. On the same 180 episodes the range is 33--92 N. Both
   numbers are individually correct and the sentence reads as one denominator.
   The 2026-08-23 entry called this "reviewer bait; costless to fix".

Nothing here is a new measurement — it is the same data, recomputed, showing the
paper text drifting from the lab record over eleven days. Worth a standing check:
`scripts/guard_pareto.py` reproduces all of it, so the guard paragraph can be
verified against the data on every rebuild rather than by memory.

## Appendix D's sign test reports the p-value of an experiment that did not happen (2026-09-04)

Audit of App. D against `scripts/appendix_d_stats.py` and
`blindspot/data/channel_j2.npz`. One error, with a root cause in code.

**The claim.** main.tex: "$\delta$ attains the higher $R^2$ in four of five
poses, which under an exact paired sign test is $p = 0.06$ --- suggestive at
$n = 5$ poses and not significant."

**The count is right and the p-value is not.** δ beats `Present_Current` in
4/5 poses (pose 2 is the exception: R²_δ 0.473 vs R²_curr 0.503). The exact
paired sign test at 4/5 gives **p = 0.375 two-sided** (0.188 one-sided).
p ≈ 0.06 is the two-sided value for a clean **5/5** sweep — the smallest value
reachable at n = 5 — i.e. the p-value of an outcome that did not occur.

**Root cause, `appendix_d_stats.py:124-127` (before today's fix).** The script
printed the observed win count and a constant `2**-n_pose * 2` on the same
line, with the conditional buried in the format string as `"two-sided if all"`:

    print(f"delta beats Present_Current in {wins}/{n_pose} poses "
          f"(exact paired sign test p = {2**-n_pose * 2:.3f} two-sided if all)")

Whoever transcribed it into the tex took the number and dropped the "if all".
The script now computes the exact test for the count actually observed and
prints the clean-sweep value separately as an explicitly labelled reference.

**What changes and what does not.** The paper's *conclusion* was already right
and is unaffected — "not significant… we report the ordering and decline to
claim it". What must change is the number and one adjective: at p = 0.375 the
ordering is not "suggestive", it is uninformative. This is the first error found
today that moves a stated statistic in the flattering direction while the
surrounding prose stays honest, which is the combination hardest to catch by
reading.

**Verified correct in the same pass, so the appendix is not under general
suspicion:** `Present_Load = −4.70 (±0.10)·δ + 0.1`, R² 0.976 [0.969, 0.982],
n = 60 — reproduces exactly. Per-pose δ/load agreement "identical to 3 decimals
in 3 of 5 poses, max |difference| 0.017" — reproduces exactly (Table 4's
0.261/0.279 rounds to 0.018, but the unrounded max is 0.017; the text is
right). Mass-resolution figures 17 g at pose 3 and 41 g at pose 1 recompute
from replicate/slope as 17.0 and 41.2; the three poor poses give 161, 170 and
232 g against the stated "160--230".

**Category note.** This is a fourth failure class, distinct from the three
already logged: not drift, not a wrong claim on a right number, not a right
number with no generator — a **right generator whose output was transcribed
past its own conditional**. `check_paper_numbers.py` would not catch it, because
the script's printed line was itself ambiguous. The fix is upstream: scripts
should not print a statistic for a case other than the one observed.

## Sec. 3.2 envelope audit: clean (2026-09-04)

Recorded because a verified section is worth as much as a corrected one, and so
neither session re-audits it. All three constants in `main.tex:209-227` reproduce
exactly from `results/delta_characterization.json` (`pooled_fit`):

| claim in Sec. 3.2 | stored value | verdict |
|---|---|---|
| κ = 2.00 N/count | `slope_n_per_count` = 1.99933 | exact |
| held-out RMSE 4.3 N | `test_rmse_n` = 4.3103 | exact |
| over 4–78 N | `force_range_n` = [4.0026, 78.1549] | exact |

Fit is 468 train / 312 held-out points. The `±9.2 N` budget and `F_min = 20 N`
that appear in `hardware/staircase_cal.py`'s docstring are project-internal
envelope figures and are **not** claimed anywhere in main.tex, so there is
nothing to reconcile there.

**One completeness note, not an error.** The pooled fit carries
`bias_n = −3.568`, and Sec. 3.2 reports only the slope. The intercept is ~83% of
the held-out RMSE and implies δ = 0 maps to −3.6 N, which is unphysical as a
force reading. It does not affect any claim the paper makes — κ is used only for
the quantization design rule (quantum < M/κ), which depends on the slope alone —
but a reader converting δ to newtons with κ and no intercept is off by a constant
3.6 N. Worth one clause in the sentence that introduces κ.

With this, the sections audited against their generating data are: the grid
(decomposition + information audit), the guard (App. C), the corpus paragraph,
App. D static and dynamic, and Sec. 3.2. Unaudited: Sec. 4.1's cliff/validity
numbers, and the Method's problem formulation.

**Newton-provenance check (same pass, prompted by `projects-a5`): also clean.**
The worry was that App. D now contains real hardware measurements, so a reader
returning to a body full of newtons might not carry the "simulated" qualifier
with them. Every newton quantity in `main.tex` was traced:

- Sec. 3.2 (L214): "On our simulated plant… These constants describe the modeled
  plant" — explicit, twice, before the numbers.
- Sec. 4.4 guard (L443): "all results are simulated" — explicit.
- App. C (L641): "≈ +24 N on the simulated plant" opens the paragraph the
  remaining guard forces (40, 99–127, 360, 10–28, 39, 33–92, 13–26, 33–131 N)
  sit in.
- App. D "What is still not measured" (L815): "No load cell has been applied, so
  the newton scale is unmeasured and **the simulator's 2.0 N/count remains the
  provenance of every force figure in the body**."

The last one is the strongest possible placement — the hardware appendix itself
tells the reader the body's newtons are simulator-derived, which is exactly where
a reader in hardware mode will be. No gap found; no change needed.

## Sec. 4.1 audit: the M/κ design rule is stated as a threshold the data does not support (2026-09-04)

Last unaudited load-bearing section. The *mechanism* claims hold; the
*quantitative* claim does not, and `findings.md:270-280` already said so in
August without the paper absorbing it.

**What the paper claims.** Sec. 3.2: "a safe-grip window of force margin $M$ is
learnable through the channel only if the observed quantum is finer than
$M/\kappa$". Sec. 4.1: "Coarsening the observed channel collapses force-aware
grasping once the quantum exceeds $M/\kappa$." Both state $M/\kappa$ as *the*
threshold.

**What the sweep shows.** Taking $M = \text{crush} - 91$ N (the peak-grip
stand-in the sweep uses) and reading the resolution table:

| crush | M | M/κ (κ=2.4) | M/κ (κ=2.00) | quanta above threshold retaining ≥50% of q=1 |
|---|---|---|---|---|
| 100 N | 9 N | 3.8 | 4.5 | q=4 (100%), q=8 (82%) |
| 120 N | 29 N | 12.1 | 14.5 | q=16 (59%) |
| 160 N | 69 N | 28.8 | 34.5 | q=32 (55%) |

Under the pre-registered κ = 2.4 the stated rule is violated in **3 of 3** crush
conditions; under Sec. 3.2's κ = 2.00 in **2 of 3**. At a 100 N crush the channel
is at 100% of baseline at q=4 and 82% at q=8, both above the threshold that
predicts it should not be learnable at all. The rule is systematically
**conservative** — it fires before the channel actually degrades — which is safe
as a design guideline and wrong as a description of where the cliff is.

**Two constants for one quantity.** `scripts/resolution_sweep.py:38` hardcodes
`NEWTONS_PER_COUNT = 2.4` and its docstring labels it a PRE-REGISTERED
PREDICTION; Sec. 3.2 defines κ = 2.00 from the pooled characterization. The paper
writes the rule as $M/\kappa$ and defines κ = 2.00, but the experiment that tests
it used 2.4. Those must be reconciled *by stating which is which* — **not** by
adopting whichever fits better. 2.4 was frozen before the sweep ran, and refitting
a pre-registered constant after seeing the data is the exact move this project's
discipline exists to prevent. (For the record so nobody is tempted: κ = 2.00
happens to fit the 160 N case almost exactly, 34.5 predicted against ~34 observed.
That is not a reason to switch.)

**What survives, and it is most of it.** The cliff is real and sharp (flat q=1→8,
then a fall). It moves with the margin — q=16 leaves 1/30 at a 100 N limit and
17/30 at 160 N — which is the signature distinguishing a genuine resolution limit
from an artifact of deleting input bits, and it is the claim Sec. 4.1 most needs.
At the real encoder resolution the proxy matches the oracle (11 vs 12, 17 vs 14,
20 vs 17).

**Suggested repair, minimal.** Say the quantum must be *some fixed multiple*
finer than M/κ, or state the rule as necessary-but-not-tight: exceeding M/κ marks
where degradation begins, and the observed halving point runs 1.2–3.1× beyond it.
`findings.md:277-280` already reached this conclusion in August — "the mechanism
is right, but the constant is not… the law should be re-derived from the
peak-grip distribution before it is claimed as quantitative" — so the repair is to
propagate an existing lab-record verdict into two sentences, not to run anything.

## Bibliography audit: six entries carried invented given names (2026-09-04)

Prompted by the `hwang2018virtual` error, run as a full pass over all 28 entries
in `paper/refs.bib`, each checked against arXiv `citation_author` metadata or
Crossref's publisher-deposited record.

**Six entries had wrong author given names. In every case the surnames were
correct and only the given names were wrong** — the signature of names being
generated rather than looked up:

| entry | bib said | actually |
|---|---|---|
| `hwang2018virtual` | Hwang, **Seonghyeon**; Minami, **Yuta** | Hwang, **Yoonkyu**; Minami, **Yuki** |
| `dou2026neuralactuator` | Onyemelukwe, **Chidera**; Zhang, **Haoran**; "and others" | Onyemelukwe, **John U.**; Zhang, **Hangxing**; + 9 more authors |
| `oh2026factr2` | Oh, **Jason**; Liu, **Sirui**; Tao, **Ruoshi**; Han, **Yumeng** | Oh, **Steven**; Liu, **Jason Jingzhou**; Tao, **Tony**; Han, **Philip** |
| `liu2025factr` | Liu, **Sirui**; Li, **Jason Jingzhou**; Tao, **Ruoshi** | Liu, **Jason Jingzhou**; Li, **Yulong**; Tao, **Tony** |
| `yamane2025sensorless` | Yamane, **Kazuki**; Li, **Yicheng**; Konosu, **Yusuke**; Inami, **Masashi** | Yamane, **Koki**; Li, **Yunhan**; Konosu, **Masashi**; Inami, **Koki** |
| `torne2025pasttoken` | Liu, **Sirui** | Liu, **Yuejiang** |

All six corrected.

**The decisive evidence that these were generated, not mistyped:** "Liu, Sirui"
appears as a wrong given name in **three separate entries** — `oh2026factr2`,
`liu2025factr`, and `torne2025pasttoken` — none of which has an author by that
name. Several of the invented names are real people attached to the wrong
surname: the true first author of `liu2025factr` is Jason Jingzhou Liu, and the
bib assigns "Jason Jingzhou" to *Li* while giving *Liu* the invented "Sirui".
Tony Tao is real and appears as "Ruoshi Tao". A typo does not do this.

**Why it matters more than a formatting slip.** `oh2026factr2` is the paper's
central positioning reference — Sec. 2 says "Our lag excess is a deliberate
special case of FACTR 2's central idea" — so four wrong given names sit on the
citation the contribution is defined against. These are the errors most visible
to exactly the reviewer best placed to judge the work, and they cost nothing
intellectually.

**Verified correct, so nobody re-checks them:** `zhao2023act`, `chi2023diffusion`
(a Crossref title-match false positive in the first pass; the entry is exact
against arXiv), `yen2019virtual`, `deluca2005sensorless`,
`haddadin2017collisions`, `dehaan2019causal`, `wen2020copycat`,
`calandra2017feeling`, `kim2024openvla` (18 authors), `mandlekar2021robomimic`
and `alshiekh2018shielding` (both flagged by the script, both artifacts of LaTeX
accent escaping — `Mart{\'i}n-Mart{\'i}n` and `R{\"u}diger` are correct),
`hsu2024safetyfilter`, `brunke2022safelearning`, `wong2026beyond`,
`zeng2026revisiting`.

**Not errors, but decide before submission:** `black2024pi0`,
`shukor2025smolvla` and `nvidia2025groot` use `and others`, hiding 14, 4 and 33
authors respectively. Legitimate BibTeX, renders as "et al.", but most robotics
venues want the full list. `torne2025pasttoken` uses "Torne Villasevil" where
arXiv says "Torne"; PMLR's own record is `villasevil25a`, so the double surname
is defensible and was left.

**PAPER ACTION:** none — `refs.bib` is fixed. But no unverified given name should
enter this file again; the check is one arXiv or Crossref lookup per entry and it
is now scripted in this entry's method.

## The reading notes are NOT fabricated: spot-check of thread A (2026-09-04)

Follow-on from the bibliography audit. If given names in `refs.bib` were
generated rather than looked up, the next surface at risk is
`docs/reading_notes/thread_A_history_in_IL.md`, which carries ~40 specific
quoted numbers from 15 papers and is the input to the paper's Related Work. Its
header claims "Every URL below was actually fetched." That claim is testable.

**Two entries spot-checked against full text, 8 numbers, all exact.**

Entry 2, Wen et al. 2020 (arXiv:2010.14876, ar5iv full text):
- notes: "HalfCheetah −38 → 820" — source table: BC-SO −38 ± 36, BC-OH 820 ± 60. ✓
- notes: "Walker2D MSE 0.46e-2 vs 2.47e-2", described as the BC-OH policy being
  "~5× more predictable-from-history than the expert's" — source: BC-OH 0.46 ±
  0.02, expert 2.47 ± 0.07, column unit ×10⁻². Ratio 5.4×, and the direction is
  right (lower MSE = more predictable). ✓
- The abstract independently confirms the entry's qualitative framing, including
  "removes excess information about the previous expert action nuisance
  correlate, while retaining the information necessary to predict the next
  action", which the notes paraphrase accurately.

Entry 3, Wen et al. 2021 Keyframe-Focused (arXiv:2106.06452, ar5iv full text) —
all six numbers, correctly rounded and correctly attributed to method and
condition:

| notes | source table |
|---|---|
| CARLA: BC-OH 33.0 vs BC-SO 42.7 | 33.000 ± 4.190 / 42.667 ± 8.668 |
| keyframe-weighted 43.4 | Ours (step) 43.444 ± 0.786 |
| CARLA-w/o-speed: 9.2 / 25.7 / 36.8 | 9.222 / 25.667 / 36.778 |

**Verdict, with its limits stated.** The fabrication found in `refs.bib` was
confined to author given names and does **not** extend to the reading notes'
quantitative content. This is a spot-check — 2 of 15 entries, 8 of ~40 numbers —
so it is evidence, not proof, and it was deliberately taken from two different
papers rather than two claims in one entry. The notes' own "every URL was
actually fetched" claim is supported.

That distinction matters for triage: `refs.bib` metadata was typed from memory,
while the reading notes record things actually read. Anything else in the repo
should be sorted by which of those two it resembles.

**QUALIFICATION, same day, from `projects-a5` and verified here.** The rule above
is too strong as written. Thread B entry 4.5, on FACTR (arXiv:2502.17432), says
the curriculum improves "unseen-object generalization by 43% and policy
performance by 40% over force-as-input-without-curriculum". I fetched the
abstract independently: **43% is exact, including its comparison condition**
("significantly improves generalization to unseen objects by 43% compared to
baseline approaches without a curriculum"); **40% does not appear, and neither
does "policy performance"** — 0 occurrences of either, on /abs and /abs v1.

Two things make this the refs.bib signature rather than a transcription slip.
The fabricated 40% sits **in the same sentence as a correct 43%** — a plausible
figure generated alongside a real one. And the note's very next sentence says
"The abstract does not specify the actuators or the force source", so the writer
knew they were working from the abstract only and still stated a number it does
not contain. The header's "abs fetched" claim is true and protects nothing.

So the accurate rule is narrower than "the notes record things actually read":

> Where a reading note quotes a number, it is usually right and occasionally is
> not, and the failure mode is a fabricated figure adjacent to a real one in the
> same sentence. No header claim about fetching protects against it, because the
> URL genuinely was fetched.

Evidence that the notes are still much closer to source than `refs.bib` was, and
that this is a qualification rather than a reversal: the eight thread-A numbers
verified exactly, including rounding and condition attribution, and this same
note's author list — "Liu, Li, Shaw, Tao, Salakhutdinov & Pathak" — is correct in
both membership and order, where `refs.bib` had three of those given names wrong.

**Did not reach the paper.** `main.tex:148,150` cites FACTR and FACTR 2 only
qualitatively and quotes neither percentage, so there is no correction to make.

**PAPER ACTION:** any number originating in the reading threads must have its own
source check before entering `main.tex`; "it is in the notes" is not sufficient
provenance. Currently no such number is in the paper, so this is preventive.

## Thread D audit: one fabricated quotation, and a corroboration we are not using (2026-09-04)

Took threads C/D while `projects-a5` worked B. Checked thread D's five
source-quoting entries against arXiv full text or abstracts.

**Entry 12, FACTR 2 — the highest-stakes note in the corpus, and it verifies.**
This is the closest prior art and the paper's differentiation rests on its
reading. Checked against `arxiv.org/html/2606.12406v1`:

| note claims | source |
|---|---|
| "outperforms prior force-aware policies by over 17% in task progress" | verbatim ✓ |
| "NIST Belt 0.494→0.767" | table: NIST Belt C 0.494, PC 0.767 ✓ |
| "Motor current is readily provided on most robot arms, and is approximately related to motor torque" | verbatim ✓ |
| estimator input `[q, q̇, Δq^d]` over a history window | confirmed: they compare histories of `q`, `(q,q̇)`, and `(q,q̇,Δq_d)` at 10/25/50 steps ✓ |
| "AgileX Piper ($2.5k)" | **the price appears nowhere in the paper** — unsourced detail inside a sourced parenthetical |

**And the note undersells it — this is the useful part.** FACTR 2 *ablates* the
estimator's input modalities and reports: "Using (q, q̇, Δq_d) consistently
outperforms the other inputs, suggesting that **tracking error provides useful
information about controller effort and actuator response**." That is the closest
prior art independently confirming, by ablation, that the tracking error carries
information beyond joint positions and velocities — which is this paper's core
claim, arrived at from the other direction. Neither the note nor `main.tex` uses
it. It is stronger corroboration than anything else we have found today, and it
comes from the one paper a reviewer is most likely to weigh us against.

Related, and relevant to the `zeng2026revisiting` thread: they observe that "a
standard behavior cloning policy… typically receives only the current observation
rather than a long proprioceptive history such as the 50-step history used by
NEXT."

**Entry 13, Hang et al. 2018 — a fabricated quotation.** The note gives, in quotes,
"For robotic manipulation, proprioception is translated as the combination of joint
position and torque sensing." The abstract says: "we focus on grasping unknown
objects using proprioception (**the combination of joint position and torque
sensing**)." The *substance* is exact; the quoted wording is not. The note also
states "[Abstract fetched; full text not read]", so it presents as a quotation
something absent from the only text it claims to have read.

This is a new failure class, distinct from the FACTR 40%: not a fabricated number
but a **fabricated quotation with accurate meaning**. Lower severity — nothing
downstream is wrong — but it fails verification if anyone checks, and it is the
same underlying behaviour of generating plausible text adjacent to real material.

**Verified exact:** entry 11 (Current as Touch, arXiv:2607.03529) 3 of 3 quoted
passages; entry 14 (tendon proprioception, arXiv:2509.12969) 2 of 2.

**PAPER ACTION:** cite FACTR 2's input-modality ablation as independent
corroboration that tracking error carries information beyond `q` and `q̇`. It
belongs wherever Sec. 1 or Sec. 2 states the observability claim, and it partly
answers the standing objection that our learning evidence is simulated.

## Thread C audit: clean on the load-bearing numbers (2026-09-04)

Seven full-text-fetch entries. Checked the two whose numbers do the most work.

**Entry 13 (Nakamura et al., arXiv:2511.18606) — every number exact, and it is the
published analogue of our guard result.** Verified against the arXiv HTML:

- "735 trajectories" — source: `|D| = 735` ✓
- "80% vs 38%" — verbatim ✓
- Table 2 aggregate columns (Fail / Stall / Success):

| filter | Fail | Stall | Success |
|---|---|---|---|
| None | **62** | 0 | **38** |
| Least Restrictive | 0 | **62** | **38** |
| CBF No GP | 0 | 62 | 38 |
| LatentCBF (GP) | 0 | 20 | **80** |

The note's "unfiltered 62% failures / 38% success", "a least-restrictive filter
achieves 0% failures but converts every failure into a stall — success stays at
38%", and "their smooth CBF reaches 0% failures with 80% success" are all
literal table values. I initially flagged the 62% as a possible unsafe inference
from 100−38 given the paper separates Stall from Fail; that was wrong, and
checking the table rather than the prose is what settled it.

**Why this row matters to Sec. 4.4.** The Least Restrictive row — failures 62→0,
stalls 0→62, successes unchanged at 38 — is the published, real-hardware form of
the trade our guard exposes, and it is a *more favourable* form than ours: LR
preserves its policy's 38% while ours goes 12/270 → 0/270. Their paper treats
that as the baseline worth beating. Citing it puts our guard's number in the
literature's own frame rather than leaving it as an unexplained zero.

**Entry 14 (X-Safe, arXiv:2606.22278) — substantially verified.** "safety
constraints encompass self-collision and collision with quasi-static environment
objects" verbatim ✓; "not spilling liquid" as a named open problem ✓; the 0.02%
collision figure appears in the results table ✓; AgileX Aloha VLA evaluation
present ✓. One figure to re-check before use: the note says "~6% success
degradation on Franka (vs 20% for the reachability baseline RAIL)", and the table
row I located reads DP 76.0, RAIL 58.0, DP+X-Safe 70.0 — a 6-point degradation
for X-Safe (matches) but 18 points for RAIL, not 20, and that row may be a
different embodiment than the note's "Franka". Low stakes, nothing downstream.

## Quotation audit of main.tex: closed, all three verified (2026-09-04)

The fabricated-quotation class found in thread D (entry 13, Hang et al.) prompted
a check of every quoted string in the paper. `projects-a5` verified them; recorded
here because the result is the distinction that matters:

| quoted string | source | result |
|---|---|---|
| `zhao2023act` "is implicitly defined by the difference between" | ar5iv 2304.13705, full text | exact, correct attribution |
| `oh2026factr2` estimator-input ablation conclusion | arXiv html 2606.12406 | exact |
| `hwang2018virtual` identification requirement | — | paraphrase, not quoted |

**The fabricated-quotation class exists in the reading notes and does NOT appear
in `main.tex`.** That is the difference between a submission risk and a
housekeeping item, and today it is housekeeping.

**Tooling note, worth more than it looks.** `arxiv.org/abs` returns the abstract
only and will produce a false negative on any full-text quotation;
`arxiv.org/html` 404s for older papers; `ar5iv.labs.arxiv.org` is the reliable
full-text route. The ACT quotation would have been wrongly declared fabricated by
anyone who stopped at `/abs` — the same failure mode we spent the day guarding
against, arriving from the tooling side instead of the writing side.

**PAPER ACTION:** none outstanding. Gate line: every quoted string in `main.tex`
re-fetched before submission with date and source URL recorded; first pass done
2026-09-04, all three exact.

## B2 complete: the extra frame is not doing the work, at the point estimate (2026-09-04)

Arm B2 (`[s, a[t−1], a[t−2]]`, 18-dim), three seeds at the frozen grid protocol,
pre-registered in `task2_arms.md` before launch. Seeds 11 / 16 / 17, mean 14.67,
sd 3.21. Recomputed here independently of `projects-a5`; both implementations
agree to the decimal.

| comparison | Δ | Welch t | 95% two-level CI | width |
|---|---|---|---|---|
| E − B2 | **+11.9** | +3.03 | [+3.3, +20.7] | 17.4 |
| B2 − B | +0.9 | +0.22 | [−7.7, +9.0] | 16.7 |
| E − B | +12.8 | +2.61 | [+2.8, +22.6] | 19.8 |

**The frozen reading applies.** `task2_arms.md` registered E ≥ B2 as evidence for
the subtraction and E < B2 as ambiguous. E > B2, so the ambiguous branch does not
apply and the 18-vs-12-dim width caveat does not bite.

**What it establishes:** against a control seeing *exactly* what E sees —
s[t], a[t−1], a[t−2] — E retains +11.9 with an interval excluding zero. The
effect is located in the *form* of the channel, not in the extra frame.

**What it does not establish, twice over.**

1. +11.9 is below the **15-point resolution bar** frozen before the cells ran.
   The interval excludes zero and t = 3.03, but the rule exists precisely so a
   favourable result cannot argue past it afterwards. Not resolved.
2. **"The confound is bounded at ~1 point" overstates the data.** That reads the
   B2 − B *point estimate* (+0.9) as a bound. The interval is [−7.7, +9.0] — the
   extra frame's contribution could be as much as +9.0, which is most of E − B's
   +12.8. A null at this width bounds nothing tightly. The defensible claim is the
   direct matched comparison E − B2, which needs no decomposition; the
   decomposition into "+0.9 frame, +11.9 subtraction" should not be written.

**Design asymmetry, disclosed:** E has 5 seeds, B2 has 3. The bootstrap resamples
each at its own n, but the comparison is not seed-matched and no seed-matched
variant was pre-declared, so none was constructed after the fact.

**PAPER ACTION:** the abstract's "its information-matched control is absent from
this grid" and Limitations' "the matched two-step baseline is not in the present
grid" became false when this cell landed. Both must carry the result. State it as
E − B2 with its n, not as a decomposition, and state that it sits below the
grid's own resolution bar.

## The resolution bar is seed-scaled, and it predates the argument (2026-09-04)

`projects-a5` asked whether invoking n=5-versus-n=3 to license E−B while
conceding E−B2 is principled or is a standard chosen to fit. It is principled,
and the decisive fact is provenance rather than reasoning.

`scripts/fig_grid.py:65` computes `res = 15.0 * (3.0 / n_seeds) ** 0.5` and draws
that band on Figure 2. It was committed **2026-08-18** (`395ae67`, "grid brackets
+ seed-scaled resolution band") — seventeen days before this discussion, before
arm B2 existed, before today's audit. The 15-point figure in `task2_arms.md` is
itself declared "at 3 seeds", under a protocol of seeds {0,1,2}. So the scaling
is not invented to rescue a number; it is the rule already printed on the paper's
central figure, and √(3/n) is the correct scaling for a difference of means at
fixed σ_seed.

| comparison | Δ | binding n | resolution | verdict |
|---|---|---|---|---|
| E − A | +20.4 | 5 | 11.62 | **clears, by ~2×** |
| E − B | +12.8 | 5 | 11.62 | **clears** |
| E − B2 | +11.9 | 3 | 15.00 | below |
| C − B | +8.0 | 5 | 11.62 | below |
| E − C | +4.8 | 5 | 11.62 | below |

**Consequence: the abstract's original sentence was true and the walk-back was a
de-improvement.** "The only design to beat an action-history baseline at our
pre-declared seed resolution" is correct — at five seeds the bar is 11.62 and
E−B = 12.8 clears it. Commit `86750b7` softened this to "close to the grid's
declared resolution" on the mistaken premise that the bar is a flat 15.

**And the E−B2 concession survives the most favourable treatment of its mixed n.**
E has 5 seeds, B2 has 3, so the difference's SE is σ√(1/5+1/3) = 0.730σ against
0.816σ for a 3-vs-3 pair. Scaling the 15-point bar by that ratio gives 13.4;
E−B2 = 11.9 is below it either way. There is no reading of the mixed n under
which B2's margin clears, which is what removes the suspicion of standard-fitting
— the rule was applied conservatively in the one place it could have been bent.

**PAPER ACTION:** restore the abstract's resolution claim for E−B, citing the
seed-scaled bar rather than a flat 15, and state the scaling once where the bar
is introduced. Keep the E−B2 concession exactly as written.

## G complete: position history lands on C, and the strong claim does not survive (2026-09-04)

Arm G (`[s[t], s[t−8]]`, 12-dim, no `a[t−1]` in any form), three seeds at the
frozen grid protocol: 15 / 22 / 24, mean 20.33, sd 4.73. Recomputed independently
of `projects-a5`; both implementations agree.

| comparison | Δ | t | 95% CI | bar (binding n) | seeds needed |
|---|---|---|---|---|---|
| E − A | +20.4 | +4.68 | [+11.6, +29.0] | 11.62 (5) — **clears** | 2 |
| **G − A** | **+14.1** | **+3.73** | **[+5.9, +22.0]** | 15.00 (3) — below | 4 |
| E − B2 | +11.9 | +3.03 | [+3.3, +20.7] | 15.00 (3) — below | 5 |
| G − B | +6.5 | +1.48 | [−2.7, +15.6] | 15.00 (3) — below | 16 |
| E − G | +6.3 | +1.42 | [−3.2, +15.7] | 15.00 (3) — below | 18 |
| **G − C** | **−1.5** | **−0.29** | [−11.7, +8.9] | 15.00 (3) — below | 314 |
| B2 − B | +0.9 | +0.22 | [−7.6, +9.1] | 15.00 (3) — below | 899 |

**The two comparisons that matter were not the ones we designed G to test.**

`G − C = −1.5, t = −0.29`. An arm that observes only past *positions*, from which
the tracking error is **not computable in principle**, performs indistinguishably
from the raw tracking-error arm. `task2_arms.md` registered "Zeng predicts
G ≈ C; the observability thesis predicts C > G". At the point estimate, Zeng's
prediction is what occurred. The registered "G < C" is technically satisfied by
1.5 points, which is not a result.

`G − A = +14.1, t = 3.73, CI [+5.9, +22.0]`. Position history alone recovers
**69% of E's advantage over base**, and this comparison excludes zero more
strongly than either E − G (t = 1.42) or G − B (t = 1.48). It needs n = 4 to
clear its own bar and has 3.

**What this does to the thesis.** The strong form — *the servo tracking error is
the enabling observation for contact commitment* — is not supported by this grid.
What the grid supports is weaker and closer to `zeng2026revisiting`: **some form
of temporal information beyond `s[t]` is necessary**, and which form is not
resolved. E still leads every arm, and E − G = +6.3 favours the channel at the
point estimate, but at 18 seeds required it is the least resolved comparison in
the grid.

**What survives untouched:** E − A = +20.4 (t = 4.68), the only comparison that
clears its bar; D ≈ A, the observe-versus-predict result, which is
information-matched and unaffected; and E − B2 = +11.9, which retired the
`a[t−2]` confound.

**Registered readings, honoured.** B2's frozen asymmetry applied and gave
evidence for the subtraction. G's frozen prediction pair did not discriminate —
neither account is confirmed or refuted. The cell was built to bound the
long-context alternative and it does not bound it.

**Not claimable, and named here so it is not written by accident:** G − B = +6.5
("position history carries some of what the channel carries") is unresolved at
16 seeds required. `projects-a5` flagged this one; the two above are the ones
that run against us and neither of us had named them.

**PAPER ACTION:** E − G is now the experiment that decides the paper's central
claim, and it is ~18 seeds away (~24 h of Orin time at 1.6 h/cell). Until it
runs, Limitations must state that a position-history control matches the raw
channel arm at three seeds and that the channel-versus-context question is open.
The paper must not say position history fails to substitute.
