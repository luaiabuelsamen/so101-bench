# The Observation the Fleet Already Logs: Tracking Error as a Force Channel at Small Data

*Paper-one draft, workshop-sized (4 pages, 3 figures). Scope set 2026-08-18
after external review: a small-data observability result on one servo
family, an honest ecosystem negative, and infrastructure — nothing larger.
Full lab record: findings.md. Prior framing ("Enforce, Don't Learn")
retired; the guard is one paragraph of infrastructure here.*

## Claim (one sentence)

On SO-100/101-class arms, the position-only logs the ecosystem already
records contain a force observation — `delta = action[t−1] − state[t]`, the
servo loop's rejection of its command — and at small data the policy must
**observe** it: predicting the channel as an auxiliary target does nothing
(D 8.7 ≈ base 6.3/100), while the same information at the input, in its
confound-subtracted form, quadruples success over raw action history
(E 29.7 vs B 11.3; 3 seeds × n=100, claims restricted to gaps above the
measured σ_seed resolution).

*Lead structure per review: D-vs-A is the mechanistic sentence (predicting
≠ observing); E-over-B is the headline number; C−B and E−C are stated at
their resolution pending seeds 4–5 (funded, running).*

## 1. The channel and its envelope (foundation)

Verbatim from ACT §IV: force "is implicitly defined by the difference
between" leader and follower positions — the field names the channel in its
action labels and observes only one operand. Stock LeRobot reads
`Present_Position` alone; its own register table defines `Present_Load`.
Characterization on the simulated STS3215 plant: 2.00 N/count seated and
quasi-static (held-out RMSE 4.3 N over 4–78 N), non-informative while
sliding, resolution requirement tracking the physical safety margin (the
cliff figure). Neighbours conceded up front: NeuralActuator (RSS'26) owns
force-aware-BC-beats-position-only on this servo class with live telemetry
and a rig; FACTR 2 owns estimated-torque-into-BC via current telemetry.
The strip claimed here and nowhere else: **no currents, no calibration rig,
recoverable retroactively from recorded position-only logs** — an
already-frozen corpus of thousands of datasets.

## 2. The small-data grid (central figure)

Six observation designs, identical everything else (280 demos, 12k steps,
96 px, 3 seeds × n=100; σ_seed ≈ 10 points measured first — nothing under
15 points is claimed):

[GRID TABLE — A base | B +a[t−1] | C +delta | D delta-as-aux-target |
E delta-minus-lag-baseline | F seat-event token; plus trained-through-guard
column. Current: A {0,15,4}, B {4,15,·}, C {11,25,31}.]

Pre-registered predictions and their fates go here verbatim, including the
flat-grid outcome if it lands. Supporting causal evidence carried from the
ladder: channel-free policies plateau at 16–24% lift-commitment across
three data recipes (14× input counterfactual); channel-bearing reach 47–76%.

## 3. The ecosystem negative (corpus, full strength)

Pre-registered: delta-at-seat separates grasp outcomes at d > 0.5 in ≥10
of 20 public datasets. Observed: 2/16, pooled d = −0.39 (546 episodes) —
with 9/16 at |d| > 0.5 and six significantly *inverted*. Teleop operators
over-command every close; empty jaws stall at the mechanical stop exactly
like full ones; successes are stereotyped low-delta closes; failures carry
40–80-count jam frames. **In the wild the channel is a struggle signal,
not a success signal** (Fig: forest plot). The strongest retroactivity
claim dies here by its own test; what survives is
recoverable-and-informative with task-dependent sign — an anomaly/safety
channel, not a label mine.

## 4. Infrastructure (one paragraph)

A ~40-line jaw guard — stall/lag-excess seat detection, force-budget cap,
closing-rate limit, all from bus telemetry — passes the paired-reference
test (scripted teacher 25/30 through it vs 28/30 raw; interference = 3
episodes) and converts the scaled policy's crush rate 27→3 at the 60 N
tier. This is engineered plumbing in the Robotiq tradition, reported as
infrastructure with its integration lessons (feed back applied commands;
wrapped controllers must be closed-loop on contact), not as a contribution.

## 5. Hardware calibration

[CAL FIGURE — N vs counts on a real STS3215: delta vs Present_Load vs
Present_Current vs load cell, three speeds; comparison rows: sim 2.0
N/count, Hwang 2018, NeuralActuator reported error. The one figure that
touches reality; transfers or replaces every sim constant above.]

## 6. Methodology carried (secondary, brief)

Oracle-first benchmark screening (a rigid-block control shows standard
pick-place cannot detect force sensing at all); paired working references
before any null is reported; σ_seed measured before any arm comparison;
deleted-success accounting for safety layers. Offered as checklist, not
framework.

## Limitations

One gripper, one object family, sim except §5; 3 seeds at n=100 resolves
only ≥15-point gaps; the success proxy in §3 is unvalidated against ground
truth; force constants inherit the simulator's contact model until §5
lands.

## The wall this paper found (one line, closes the paper)

What must a policy observe to grasp reliably at 200 demonstrations on a
$500 arm? This paper shows position-only observation is not enough and one
recoverable channel moves the floor; the next one attacks the question
directly.
