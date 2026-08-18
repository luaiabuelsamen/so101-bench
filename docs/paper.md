# Lag Excess: A Retroactive Contact Channel for Behavior Cloning on Low-Cost Arms

*Luai Abuelsamen. Draft v0.4, 2026-08-18. Workshop target: 4 pages.
Status: main grid final (5 seeds). Guarded column and hardware calibration
(Sec. 4.5) in progress. Full lab record with every n and CI in
[findings.md](findings.md).*

---

## Abstract

Behavior cloning on low-cost servo arms has made robot manipulation broadly
accessible. These platforms ship without force sensing, however, and the
standard stacks observe joint positions only, so learned policies close the
gripper open-loop against contact. For logs that record synchronized
joint-position commands and measured positions, the servo tracking error δ
is a retrospectively computable proxy for actuator loading that current
stacks discard. We present the **lag excess**, a free-motion-compensated
tracking residual computed from recorded positions alone by subtracting the
servo's following lag from δ. We (i) characterize the proxy's validity
envelope on a simulated plant (2.0 N per encoder count and held-out RMSE
4.3 N in the seated quasi-static regime, both properties of the modeled
plant), (ii) compare six observation designs at 280 demonstrations under a
pre-registered seed-resolution protocol, and (iii) test the proxy on 546
episodes across 16 public teleoperation datasets under a frozen protocol.
In simulation, the lag-excess policy succeeds in 26.6/100 episodes versus
13.8/100 for an action-history baseline (5 seeds × 100 episodes, 95% CI on
the difference [+2.8, +22.6]), while predicting δ as an auxiliary training
target matches the channel-free base (6.6 vs 6.2). On the public corpus,
high δ at grasp time is associated with failure under the frozen outcome
proxy, not with success. Code and data
at [URL].

---

![Teaser](../figures/paper/fig_teaser.png)

**Fig. 1.** Two policies on an identical scene with an identical data
budget of 280 demonstrations. Top row: the stock observation (joint
positions only) reaches, grips, and releases without lifting. Bottom row:
the same network with the lag-excess channel appended to its state vector
places the block. The trace under each row plots |δ| on the jaw joint over
the episode; the contact plateau visible in both traces is the signal the
top policy never receives. Both channels are computed from the same log
fields.

---

## 1. Introduction

Low-cost servo arms and open imitation-learning stacks have made
manipulation research broadly accessible: ACT on ALOHA-class hardware [1],
diffusion policies [3], and the LeRobot ecosystem [2] with its VLA
descendants [4, 5, 6, 7] all train behavior-cloned policies on
teleoperated demonstrations. Across these stacks the proprioceptive
observation is the same default: the current measured joint positions,
one timestep, nothing else (audit in Sec. 2).

This default fails at contact. The arms carry no force or tactile sensors,
so a position-only policy cannot distinguish a seated grasp from an empty
close, and the resulting failure modes are familiar to anyone who has
trained on this hardware: crushed objects, unseated grips, drops during
transport. The omission is not for lack of signal. The servos are
proportional position controllers, so when the jaw meets an object the
measured position stalls while the command deepens, and the gap between
them reflects the servo's deflection under load. The authors of ACT state
the physics directly: force "is implicitly defined by the difference
between" leader and follower positions [1], and a recent intervention
study confirms it, showing that removing this discrepancy signal from
ACT's action labels breaks force-critical tasks [27]. The stack keeps one
operand of that subtraction in the action labels and never provides the
difference as an observation; the LeRobot Feetech driver defines
`Present_Load` and `Present_Current` registers but reads only
`Present_Position` into the state vector, and the reference ACT
implementation raises on `n_obs_steps != 1` [2].

Beyond closing the loop at grasp time, the channel has a property no
current- or torque-based estimator shares: it is recoverable
retroactively. For any recorded dataset whose `action` field holds applied
joint-position commands synchronized with `observation.state` in a common
coordinate frame, δ = a[t−1] − s[t] can be computed after the fact; 16 of
the 20 public datasets we targeted met these conditions (Sec. 4.3). The
same signal also supports runtime enforcement, since a load bound
expressed in encoder counts can be checked from bus telemetry alone
(Sec. 4.4).

We introduce the lag excess, a free-motion-compensated tracking residual
serving as a contact observation for behavior cloning on position-servo
arms. The raw tracking error confounds contact load with ordinary motion
lag, so we fit each joint's free-motion following lag by least squares on
the jaw-open frames of the training demonstrations and subtract it; the
fit is label-free and frozen before policy training. The subtraction
suppresses the free-motion component of δ, leaving a residual that is
near zero during unloaded motion and grows with applied load when the jaw
is seated and quasi-static.

Precisely, with all signals in encoder counts at the recorded cadence
(stride-2 frames of a 30 Hz bus, i.e. 15 Hz): δ_j[t] = a_j[t−1] − s_j[t],
where a is the commanded and s the measured position — the k=1 alignment
convention; an offset sweep over k = 0..5 on the public corpus leaves
every Sec. 4.3 conclusion unchanged (median Cohen's-d span 0.12) while
estimating the physical alignment at k ≈ 3–4 recorded frames, which the
Sec. 4.5 latency measurement will pin. The commanded rate is the
first difference of commands, r_j[t] = a_j[t−1] − a_j[t−2] (zero at the
first two frames of an episode). The lag model is a single per-joint
scalar through the origin, K̂_j = Σ r δ / Σ r², fit over frames whose jaw
command exceeds its 30th percentile (jaw-open ⇒ contact-free in this
task; K̂ = 0.75–1.08 across joints on our training set), computed once on
the training demonstrations and frozen. The policy input is the
concatenation [s_norm, (δ − K̂·r − μ_δ)/σ_δ] with the same normalization
statistics saved beside the checkpoint and applied identically at
evaluation; at evaluation a[t−1] is the previously issued command, so the
feature is computable from exactly what the deployed stack already has.

To evaluate the channel we build a simulated STS3215-class plant with a
calibrated contact model, a pick-and-place task whose success requires
holding grip force inside a physical window (an oracle admission test
shows standard rigid-block pick-and-place cannot detect force sensing at
all), a six-arm observation-design grid trained under a pre-registered
seed-resolution protocol, and a frozen evaluation protocol for 16 public
SO-100/SO-101 teleoperation datasets.

On the simulated plant the channel is valid within a measurable envelope,
2.00 N per encoder count with held-out RMSE 4.3 N over 4-78 N, seated and
quasi-static only (Sec. 4.1); these constants are properties of the
modeled plant pending the hardware calibration of Sec. 4.5. At 280
demonstrations, observing the channel is the design choice that resolves:
the lag-excess arm reaches 26.6/100 versus 13.8/100 for action history
(95% CI on the difference [+2.8, +22.6]) and 6.2/100 for the base, while
an auxiliary δ-prediction head leaves the base unchanged (Sec. 4.2). On
the public corpus the pre-registered prediction fails in an informative
direction: high δ at grasp marks operator struggle, and its correlation
with failure supports the enforcement use of the channel (Sec. 4.3). A
40-line guard built on the same telemetry removes 96% of simulated crush
events across three learned policies without retraining (Sec. 4.4).

We summarize our contributions below:

1. Characterization of servo tracking error as a contact/load proxy, with
   a validity envelope and a resolution-vs-margin design rule (Sec. 3.2,
   4.1).
2. A six-design observation study at 280 demonstrations: the tested
   auxiliary-prediction baseline does not recover the direct-input gain,
   and the lag-subtracted form is the only tested design clearing the
   declared threshold against raw action history (Sec. 4.2).
3. A pre-registered study across 16 public teleoperation datasets
   associating high δ at grasp time with proxy-defined failure rather
   than success (Sec. 4.3).
4. A runtime jaw guard operating from bus telemetry alone, with paired
   deleted-success accounting (Sec. 4.4).

## 2. Related Work

**Sensorless force estimation.** Momentum-residual observers recover
external torque from proprioception on torque-controlled arms [12, 13],
and virtual force sensors extend the idea to low-cost platforms [11].
Hwang et al. [10] estimate torque from commanded and measured positions on
hobby servos via dynamic system identification, prior art for the raw
channel. Series-elastic actuation made deflection-as-force a design
principle [14]. The closest current work is NeuralActuator [15], which
learns per-actuator models for external-force perception on this servo
class and reports force-aware imitation policies beating position-only
control, and FACTR 2 [16, 17], which feeds estimated external torque into
behavior cloning on commodity arms; sensorless bilateral teleoperation
exploits the same physics online [18]. Our lag excess is a deliberate
special case of FACTR 2's central idea: where FACTR 2 learns a free-motion
baseline over current and position telemetry and treats deviations from it
as external torque, we fit a linear free-motion lag model on positions
alone. What the restriction buys is retroactivity: every method above
consumes live current or load telemetry, an instrumented calibration
stage, or both, while the residual studied here is computable from
recorded position-command logs after the fact. Closest in spirit, Wong et
al. [27] show by intervention that ACT's force competence rides on the
leader-follower discrepancy implicit in its action labels; they manipulate
the signal through the labels, whereas we place it explicitly in the
observation and ask which form of it a small-data policy needs.

**Observation design in imitation learning.** The observation specs of the
dominant open stacks are strikingly uniform: ACT [1], Diffusion Policy
[3], OpenVLA [4], pi0 [5], SmolVLA [6], and GR00T N1 [7] all condition on
measured joint positions at one or two timesteps, and none states a
rationale for excluding actuator-error signals. The literature's argument
against richer observations is causal confusion: past actions invite
copycat shortcuts [8, 9], and past-token prediction is studied as a
mitigation [26]. Our grid places both effects in one table: raw action
history helps but does not resolve against forms that isolate the physical
latent (Sec. 4.2).

**Grasp monitoring and runtime safety.** Industrial grippers ship grasp
verification from actuator feedback as firmware: Robotiq's object-detection
register fires on commanded-vs-actual position discrepancy under a current
limit [22], and SCHUNK advertises workpiece-loss detection from an
integrated encoder [23]; tactile alternatives require added sensing [24].
Safety filters for learned policies in the academic literature (shielding,
control barrier functions, predictive filters [19, 20, 21]) consume a
dynamics model, a state estimate, or exteroception. The industrial
firmware above shows that commanded-vs-achieved telemetry suffices for
grasp monitoring; the guard of Sec. 4.4 ports exactly that tradition to
wrap learned policies, a position we have not found occupied in the safety
filter literature.

## 3. Method

### 3.1 Problem formulation

Let q[t] denote the measured joint positions of a servo arm in integer
encoder counts and g[t] the commanded goal positions, both logged at every
control step; in LeRobot format these are `observation.state` and
`action`. A behavior-cloned policy π maps an observation o[t] to the next
command. The tracking error is

    δ[t] = g[t−1] − q[t].

Each servo runs a proportional position loop, so joint torque obeys
τ ≈ K_p · δ and, at the jaw, applied grip force obeys F ≈ κ · δ_jaw for a
calibration constant κ in newtons per count. The lag excess is

    e[t] = δ[t] − K̂ · r[t],    r[t] = g[t−1] − g[t−2],

where r[t] is the one-step command rate and K̂ is a per-joint coefficient
fit by least squares through the origin on the free-motion frames of the
training demonstrations. All quantities are in integer encoder counts at
the demonstration rate of 15 Hz (demonstrations are stride-2 subsamples of
a 30 Hz control loop). Free-motion frames are selected by a label-free
criterion: frames whose jaw command exceeds its 30th percentile over the
training set, since an open jaw is the only contact-free guarantee this
task offers. δ and r are defined as zero at episode boundaries where no
previous command exists. The fit is deterministic given the demonstrations
and is computed before policy training. Two conditions make δ meaningful:
the logged action must be the command actually applied to the servo rather
than an unfiltered intention, and the command-to-state timing must be
known and stable. Both hold by construction in our simulated plant; on
real logs they are empirical properties of the recording stack, which we
revisit in Sec. 4.3 and 5. We study which function of the logged pair
(q, g) a policy trained on N = 280 demonstrations should observe.

### 3.2 The channel and its envelope

On our simulated plant the jaw calibration is κ = 2.00 N per count, linear
while the jaw is seated on an object and quasi-static, with held-out RMSE
4.3 N over a 4-78 N range. These constants describe the modeled plant: the
simulator implements the proportional servo loop of Sec. 3.1, so its
δ-to-force map is clean by construction, and real STS3215 hardware adds
stiction, backlash, deadband, saturation, and thermal drift that only the
calibration of Sec. 4.5 can measure. What simulation can characterize is
the structure of the proxy. Outside the seated quasi-static regime the
signal is not a load reading: during free motion δ reflects following lag
(which e[t] removes), and during fast transport inertial deflection
dominates. Fig. 2 shows the mechanism on a single grasp.

The channel is quantized at the encoder, which yields a design rule. A
task whose safe-grip window spans a force margin M is learnable through
the channel only if the observed quantum is finer than the window it must
resolve, i.e. quantum < M / κ. Empirically this failure is a cliff rather
than a slope, and the cliff location moves with the margin (Sec. 4.1).

![Mechanism](../figures/paper/fig_mech.png)

**Fig. 2.** The channel on one servo and one grasp. The commanded position
(dashed) and measured position (solid) track closely through the free
close, separated only by the small following lag. At first contact
(marked) the measured position stalls while the command continues to
deepen; the growing gap is the load reading, κ ≈ 2 N per count on the
simulated plant. The
detector's seat event (marked) fires when net travel stalls while lag
excess grows. The lower panel plots δ and the lag excess e over the same
frames; e is near zero until contact by construction.

### 3.3 System, task, and data

All experiments except Sec. 4.3 and 4.5 run on a simulated STS3215-class
7-DoF arm with a calibrated contact model. The task is pick-and-place of
an object whose retention requires grip force above a lift threshold and
whose integrity fails above a crush threshold, so success requires holding
force inside a physical window. A privileged-force oracle passes an
admission test on this task and, as a negative control, shows that
standard rigid-block pick-and-place cannot distinguish force-aware from
force-blind control at all; results on the rigid task would say nothing
about the channel. Demonstrations come from a scripted closed-loop teacher
with privileged access to simulated grip force (280 episodes), collected
through the guard of Sec. 4.4 so that recorded actions are applied
commands rather than intentions; the observation arms of Sec. 3.4 never
see the privileged force, only functions of (q, g). Policies are ACT [1]
trained for 12k steps at 96 px, identical across arms; at this resolution
we cannot rule out that the cameras carry some deformation cue, but the
base arm's floor performance indicates any such cue is insufficient alone.
Seed variance was
measured before any comparison: σ_seed ≈ 10 points at n = 100 evaluation
episodes, so 5-seed comparisons resolve gaps of roughly 11 points and we
claim nothing below that resolution.

### 3.4 Observation designs

Six arms share the training recipe of Sec. 3.3 and differ only in the
proprioceptive observation.

**Base (A).** Measured positions s[t] only: the stock LeRobot observation.
**History (B).** s[t] plus the previous action a[t−1]: the minimal change
that makes δ recoverable in principle, and the arm the copycat literature
[8, 9] warns about. **Delta (C).** s[t] plus raw δ[t]: the channel with
its motion confound intact. **Auxiliary target (D).** s[t] only, with δ[t]
predicted as an auxiliary training target through a small head at loss
weight 0.1: the same information as supervision rather than input.
**Lag excess (E).** s[t] plus e[t]: the channel with free-motion lag
subtracted (Sec. 3.1). **Seat token (F).** s[t] plus a latched binary seat
bit, produced by running the guard's frozen stall detector (Sec. 4.4)
causally over each episode's (s[t], a[t−1]) jaw pairs: the channel
compressed to one bit.

Implementation details shared across arms: joint states and actions are
standardized by dataset statistics; δ, e, and the aux target are
normalized by a fixed scale of 8 counts with zero mean, so no channel
statistic leaks from evaluation; appended channels concatenate onto the
state vector, leaving the network otherwise unchanged.

## 4. Experiments

We evaluate: *(i) over what envelope is δ a valid contact/load proxy;
(ii) which observation design exploits it at small data; (iii) does the
channel predict grasp outcomes on the public corpus?*

### 4.1 Channel behavior on the simulated plant

The calibration of Sec. 3.2 holds in the seated quasi-static regime
(RMSE 4.3 N over 4-78 N, held out). We state the scope plainly: because
the simulator implements the servo loop that generates δ, these numbers
validate the proxy against our own plant model, not against hardware;
their value is in mapping where the proxy's structure breaks, which
transfers as a set of hypotheses for Sec. 4.5. A stall detector on net
travel and lag
excess localizes seat events, and constructed adversarial negatives
(free-space reversals, transport shakes) do not fire it within the
declared envelope. Coarsening the observed channel collapses force-aware
grasping once the quantum exceeds M / κ, and rerunning the sweep at a
different margin moves the cliff accordingly: the resolution requirement
is a property of the task margin, a design rule rather than a benchmark
number.

### 4.2 Observation design at 280 demonstrations

Table 1 and Fig. 3 give the grid: 6 arms × 5 seeds × 100 evaluation
episodes, identical otherwise. Arm definitions and directional predictions
were frozen before implementation, with E vs. B declared the headline
comparison; the frozen documents, per-seed outcomes, and all runs
including the 3-seed phase are in the released repository. Alongside Welch's
t across seed means we report 95% CIs from a hierarchical bootstrap that
resamples seeds and then episodes within seeds.

| arm | observation | success/100 (mean ± std) | vs A | vs B |
|---|---|---|---|---|
| A base | s[t] | 6.2 ± 5.9 | | |
| B history | s + a[t−1] | 13.8 ± 7.7 | +7.6 (t=1.8, n.s.) | |
| C delta | s + δ | 21.8 ± 9.7 | +15.6 (t=3.1) | +8.0 (t=1.4, n.s.) |
| D aux-target | s; δ predicted | 6.6 ± 4.0 | +0.4 (t=0.1, n.s.) | −7.2 (n.s.) |
| E lag excess | s + e | **26.6 ± 7.8** | **+20.4 (t=4.7)** | **+12.8 (t=2.6)** |
| F seat token | s + seat bit | 21.8 ± 9.5 | +15.6 (t=3.1) | +8.0 (t=1.5, n.s.) |

**Table 1.** Placement success per 100 episodes, mean ± std over 5 seeds
(500 evaluation episodes per arm). Welch's t across seed means; n.s.
marks differences below the σ_seed resolution of Sec. 3.3. Best in bold.

Three results resolve. First, channel-bearing observation beats
channel-free observation in every input form (C, E, F vs A: t = 3.1-4.7;
bootstrap 95% CIs [+5.6, +25.2], [+11.6, +29.0], [+6.0, +25.2]). Second,
the tested auxiliary-prediction design does not recover the direct-input
gain: arm D trains the identical encoder to predict δ
through a weight-0.1 head and lands equal to base (6.6 vs 6.2, CI on the
difference [−6.2, +6.4]), while the same information as an input (C, E)
triples success. Other auxiliary weightings and architectures could in
principle close this gap; none was tuned here. Third, in the declared
headline comparison, only the lag-excess form beats the action-history
baseline at resolution (E−B = +12.8, t = 2.6, CI [+2.8, +22.6]); raw δ's
+8.0 over history does not resolve (CI [−2.8, +18.6]), consistent with
the motion confound of Sec. 3.2. Differences F−B (CI [−2.6, +18.6]) and
E−C (CI [−6.2, +15.8]) remain open at this budget, so whether E's edge
comes from the subtraction or from δ at all is not attributable here. The
frozen predictions called E's margin resolved and F's marginal; the
5-seed expansion confirmed both calls, collapsing F's 3-seed margin over
B and preserving E's. We note the caveats these numbers carry: five seed
means per arm, a headline difference of similar magnitude to the declared
resolution, and multiple secondary comparisons for which we apply no
correction beyond designating the primary one. Within those limits, the
consistent ordering supports one design statement: at small data on this
force-critical task, placing the free-motion-compensated residual in the
observation vector is the change that moves performance.

![Observation-design grid](../figures/paper/fig2_grid.png)

**Fig. 3.** The observation-design grid. Bars are 5-seed means, dots are
individual seeds (100 episodes each). The gray band spans the
below-resolution zone around base given σ_seed ≈ 10; differences inside
the band are not claimed. The lag-excess arm (E) is the only design whose
margin over the history baseline (B) exceeds seed resolution.

### 4.3 Ecosystem study: a pre-registered negative

If δ is computable from every recorded episode, does it predict grasp
outcomes corpus-wide? The protocol was frozen before the run: seat
detection by the stall detector of Sec. 4.1, δ at seat as unsigned
tracking error, success proxy defined as the gripper staying closed
through transport, and the prediction that successes carry higher δ with
Cohen's d > 0.5 in at least 10 of 20 datasets. Of the 20 targeted
SO-100/SO-101 datasets, 16 passed the frozen inclusion gates with
connectivity, giving 546 episodes (283 proxy successes).

**The prediction fails: 2/16 datasets.** Pooled within-dataset d = −0.39.
The channel is informative with the opposite sign: |d| > 0.5 in 9/16
datasets, and in 6/16 the effect is inverted with a bootstrap 95% CI below
zero (Fig. 4). The traces explain the physics. Teleop leader devices have
no hard stop, so operators over-command every close; the seat flag fires
in nearly every episode of 14/16 datasets because empty jaws stall at the
mechanical limit exactly as loaded ones do. Successful grasps are
stereotyped low-δ closes (in one dataset every clean episode rests at the
same 7-count gap), while failure episodes carry 40-80-count frames of an
operator forcing a jammed grasp. On the public corpus, high δ at grasp
time is associated with failure under the frozen proxy definition. This
rules out mining the corpus for success
labels through this channel alone, and it supports the enforcement use:
the high-δ events the corpus associates with failure are the events the
guard of Sec. 4.4 caps.

This result also bounds Sec. 4.2's transfer to retroactive training on
real teleop data, and we state the bound rather than bracket it. When the
seat event fires in essentially every episode, a latched seat bit (arm F)
carries no information, and raw δ at close time reads commanded depth as
much as contact; the components that remain informative on such logs are
δ magnitude and its dynamics, which is what the inverted effect sizes
measure. Whether the lag-excess form retains its simulated advantage when
trained on over-commanded human demonstrations is an open empirical
question, not a corollary of Sec. 4.2. Two further caveats attach to the
corpus numbers themselves: the recorded `action` in these datasets is the
leader position, which approximates but need not equal the applied
follower command, and per-dataset action-to-state timing offsets are
uncharacterized (Sec. 3.1); an integer-offset sweep of δ[t] = g[t−k] −
q[t] is the direct check and is future work.

![Corpus forest plot](../figures/paper/fig1_corpus.png)

**Fig. 4.** Per-dataset Cohen's d of δ-at-seat between proxy successes and
failures, with bootstrap 95% CIs, for the 16 datasets passing the frozen
gates (candidate pool, gates, and attrition documented in the repo).
Positive d was the pre-registered direction. Two datasets exceed d = 0.5
as predicted; six are significantly inverted; two task-repetitive
datasets reach d ≈ −16 and −20 because their stereotyped successes have
near-zero δ variance.

### 4.4 Runtime guard

The same telemetry supports enforcement. A 40-line jaw guard combines
net-travel stall and lag-excess seat detection, a squeeze budget of 12
counts (≈ +24 N on the simulated plant) past seat, and a closing-rate
limit, all computed from bus signals with no dynamics model, no
retraining, and no object knowledge. All results in this subsection are
simulated; hardware validation of the guard is part of Sec. 4.5. Paired
against a competent reference, the scripted closed-loop teacher places
25/30 through the guard versus 28/30 raw at a 39 N grip (23 vs 26 at the
120 N tier), so the guard costs a competent controller about 3 episodes
in 30. On three learned checkpoints over identical paired episodes,
the guard removes 85/180 → 3/180 crush events (96%) across crush tiers
(Fig. 5), with median peak force dropping from 33-131 N to 13-26 N. The
guard also deletes successes a policy earned by over-gripping: one
quantized arm lifted 7/30 episodes at a 131 N median, and those successes
vanish under the cap. Integration requires two properties: demonstrations
must record applied commands rather than intentions (Sec. 3.3), and any
wrapped controller must be closed-loop on contact, since the rate limiter
breaks open-loop grasp timing by construction. Collecting demonstrations
through the guard is cost-neutral within seed resolution on the arms
measured so far.

![Guard crush counts](../figures/paper/fig2_guard.png)

**Fig. 5.** Crush events per 30 paired episodes for three learned
checkpoints (base, delta, and a 16-count-quantized delta arm) at two crush
tiers, raw (orange) versus guarded (blue). Identical checkpoints and
episode seeds; the guard acts only through bus telemetry at runtime.
Totals: 85/180 raw, 3/180 guarded.

### 4.5 Hardware calibration

[PENDING. δ versus `Present_Load`, `Present_Current`, and a load cell on a
physical STS3215, staircase protocol at three speeds; comparison rows: the
simulator's 2.0 N/count, Hwang et al. [10], NeuralActuator [15]. Until
this lands, every newton above inherits the simulator's contact model; a
negative transfer result would itself be the first measurement of the
channel's sim-to-real gap.]

## 5. Limitations

All learning and guard results are simulation, on one gripper and one
object family, and every quantity in newtons inherits the simulator's
servo and contact model until Sec. 4.5 lands. The task is constructed to
require force-band control, so the grid measures observation design on a
force-critical task, not on manipulation in general. The lag-excess model
is a first-order velocity correction; richer free-motion predictors
(acceleration terms, direction-dependent friction, learned models as in
FACTR 2 [16]) may compensate better and are untested. The seed budget
resolves gaps of about 11 points at n = 100, so C−B (+8.0) and E−C (+4.8)
remain open. On real logs, δ additionally depends on action semantics and
command-to-state timing that our corpus study does not characterize
(Sec. 4.3). The corpus success proxy is unvalidated against ground truth,
since no outcome labels exist in the data, and multi-grasp tasks were
excluded by design. The channel is valid only seated and quasi-static; it
is not a load reading during transport. The guarded-training column is
partially complete at this draft.

## 6. Conclusion

Position-command logs contain a retrospectively computable contact proxy:
the servo tracking error, valid within a quasi-static envelope that our
simulated plant makes measurable. At 280 demonstrations in that setting,
policies must observe this channel, and predicting it as the tested
auxiliary target recovers none of the gain; the free-motion-compensated
form is the only design that beats raw action history at seed resolution. Across 16 public datasets the channel reads
operator struggle rather than success, which forecloses free outcome
labels and motivates enforcement. A 40-line telemetry-only guard removes
96% of simulated crush events on unmodified policies. The claims that
remain open are exactly the ones hardware decides: the calibration of
Sec. 4.5, retroactive training on real teleop logs, and a copycat-style
probe of when history helps versus hurts in behavior cloning.

## References

[1] T. Zhao, V. Kumar, S. Levine, C. Finn. Learning Fine-Grained Bimanual
Manipulation with Low-Cost Hardware (ACT/ALOHA). RSS 2023.
[2] R. Cadene et al. LeRobot: State-of-the-art AI for real-world robotics.
Hugging Face, github.com/huggingface/lerobot (source audited May 2026).
[3] C. Chi et al. Diffusion Policy: Visuomotor Policy Learning via Action
Diffusion. RSS 2023 / IJRR.
[4] M. J. Kim et al. OpenVLA: An Open-Source Vision-Language-Action Model.
CoRL 2024.
[5] K. Black et al. pi0: A Vision-Language-Action Flow Model for General
Robot Control. Physical Intelligence, arXiv:2410.24164, 2024.
[6] M. Shukor et al. SmolVLA: A Vision-Language-Action Model for
Affordable and Efficient Robotics. arXiv:2506.01844, 2025.
[7] NVIDIA. GR00T N1: An Open Foundation Model for Generalist Humanoid
Robots. arXiv:2503.14734, 2025.
[8] P. de Haan, D. Jayaraman, S. Levine. Causal Confusion in Imitation
Learning. NeurIPS 2019.
[9] C. Wen, J. Lin, T. Darrell, D. Jayaraman, Y. Gao. Fighting Copycat
Agents in Behavioral Cloning from Observation Histories. NeurIPS 2020.
[10] S. Hwang, M. Minami, M. Ishikawa. Virtual Torque Sensor for Low-Cost
RC Servo Motors Based on Dynamic System Identification Utilizing
Parametric Constraints. Sensors 18(11):3856, 2018.
[11] C.-L. Yen, P.-C. Tang, C.-Y. Lin, et al. Development of a Virtual
Force Sensor for a Low-Cost Collaborative Robot and Applications to Safety
Control. Sensors 19(11):2603, 2019.
[12] A. De Luca, R. Mattone. Sensorless Robot Collision Detection and
Hybrid Force/Motion Control. ICRA 2005.
[13] S. Haddadin, A. De Luca, A. Albu-Schäffer. Robot Collisions: A Survey
on Detection, Isolation, and Identification. IEEE T-RO 33(6), 2017.
[14] M. Williamson. Series Elastic Actuators. MIT MS thesis, 1995.
[15] Z. Dou, C. Onyemelukwe, H. Zhang, et al. NeuralActuator: Neural
Actuation Modeling for Robot Dynamics and External Force Perception.
RSS 2026. arXiv:2607.11734.
[16] J. Oh, S. Liu, R. Tao, et al. FACTR 2: Learning External Force
Sensing for Commodity Robot Arms Improves Policy Learning.
arXiv:2606.12406, 2026.
[17] S. Liu, J. Li, K. Shaw, et al. FACTR: Force-Attending Curriculum
Training for Contact-Rich Policy Learning. arXiv:2502.17432, 2025.
[18] K. Yamane et al. Design and Experimental Validation of Sensorless
4-Channel Bilateral Teleoperation for Low-Cost Manipulators.
arXiv:2507.06174, 2025.
[19] M. Alshiekh, R. Bloem, R. Ehlers, et al. Safe Reinforcement Learning
via Shielding. AAAI 2018.
[20] K.-C. Hsu, H. Hu, J. Fisac. The Safety Filter: A Unified View of
Safety-Critical Control in Autonomous Systems. Annu. Rev. Control Robot.
Auton. Syst., 2024.
[21] L. Brunke et al. Safe Learning in Robotics: From Learning-Based
Control to Safe Reinforcement Learning. Annu. Rev. Control Robot. Auton.
Syst., 2022.
[22] Robotiq. 2F-85 / 2F-140 Instruction Manual, Control section: gOBJ
object-detection register.
[23] SCHUNK. EGK electric parallel gripper: workpiece-loss detection.
Product documentation.
[24] R. Calandra et al. The Feeling of Success: Does Touch Sensing Help
Predict Grasp Outcomes? CoRL 2017.
[25] A. Mandlekar et al. What Matters in Learning from Offline Human
Demonstrations for Robot Manipulation (robomimic). CoRL 2021.
[26] M. Torne Villasevil, A. Tang, S. Liu, C. Finn. Learning Long-Context
Diffusion Policies via Past-Token Prediction. CoRL 2025.
[27] K. H. Wong, L. Liu, F. Dayoub. Beyond Implicit Force: Evaluating
Explicit Force-Torque Proxies in Action Chunking with Transformers.
arXiv:2607.14578, 2026.
