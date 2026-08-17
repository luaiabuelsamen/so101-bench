# The gap statement

*v2, after external review of v1. One claim, one page. Sources:
`docs/reading_notes/` (58 papers); the two load-bearing quotes below were
re-verified against the primary sources directly, not the reading agents'
notes.*

## The gap, in one sentence

**The position-only logs the field already ships contain a sufficient force
observation — here is the proof, the envelope, and the ecosystem-wide
measurement.**

Everything in this repo is a section of that sentence: the calibration is
"sufficient... observation" made quantitative, the BC ladder and autopsy are
the proof, the resolution cliff is the envelope, the corpus study is the
ecosystem-wide measurement, and the guard is the corollary (the same
observation enforced rather than learned).

## Defaults, not representation

The anticipated kill-shot: "this is a POMDP with a one-step sufficient
statistic; set `n_obs_steps=2` and any model recovers it — your own
base_hist result proves it." Correct — and that is the claim. Nothing here
is about architectures or a new representation; it is that **every stock
stack observes one operand of a subtraction the field itself names.** The
two operands, verified at the source:

- ACT, §IV (verbatim, checked against the paper): *"It is important to use
  the leader joint positions instead of the follower's, because the amount of
  force applied is implicitly defined by the difference between them, through
  the low-level PID controller."* The force channel is deliberately kept in
  the action labels — and never provided as an observation; the reference
  implementation raises on `n_obs_steps != 1`.
- LeRobot (verified in source, May-2026 tree): the Feetech table defines
  `Present_Load` (addr 60) and `Present_Current` (addr 69); upstream
  `get_observation()` reads only `Present_Position`. (Our local checkout
  carries an uncommitted patch adding `.load` to the observation — that diff
  is the PR this project intends to ship.)

A design default of the ecosystem, not a taxonomy cell. The "fourth regime"
IL-theory framing of v1 is real but is **the next paper**, not this one; its
instrument (a Wen-style copycat probe: does the policy track `a[t]` or
`a[t-1]` at the frames where the expert breaks jaw autocorrelation) is
written and smoke-tested (`scripts/probe_copycat.py`), and parks until then.

## Neighbours, and the one defensible strip

- **NeuralActuator (RSS 2026, Outstanding Systems Paper; code + data
  released July 2026).** Not merely an accuracy neighbour: it already
  reports hardware BC comparisons where a force-aware policy beats
  position-only control. "Force input improves BC on this servo class" is
  therefore **owned, by an award paper**. What it needs and we don't: live
  load/current telemetry and an instrumented calibration rig. Our strip is
  exactly *no currents, no calibration, retroactive on the recorded corpus*
  — and nothing else.
- **FACTR 2 (2026):** estimated external torque into BC on commodity arms —
  same strip left open (estimator needs current telemetry + calibration;
  nothing retroactive). Steal its free-motion self-labeling trick.
- **Hwang 2018:** prior art for raw (command, position) → torque on hobby
  servos; cite, then supersede with the validity envelope.
- **Shields (De Luca/Haddadin residuals; Simplex/RTA/shielding/CBF):** every
  published enforcing filter consumes a model, a state estimate, perception,
  or an F/T sensor; none covers grasped-object crush; none audits deleted
  successes. The guard keeps this unclaimed position; LeRobot's only shipped
  guard (`max_relative_target`) is command-side, contact-blind, off by
  default.

**The window closes from both sides.** NeuralActuator's released SO-101
teleop stack logs current, and any upstream merge of a load-reading
observation (our own drafted patch included) ends the era of position-only
logs. The moat is the *existing* corpus — thousands of datasets whose only
force channel, forever, is delta. The retroactive claim does not expire;
the priority to state it first does.

## The week, in the order the thesis demands

1. **Corpus study** (`scripts/corpus_delta_outcome.py`, unparked — the kill
   test). Delta at first-close vs regrasp across public SO-101 datasets.
   Cheapest experiment we have (no training), and the one the headline
   sentence lives or dies on. If delta at contact does not separate outcomes
   in the wild, the framing dies this week, at CPU prices, as it should.
2. **Channel-content substitution grid** {base, +a[t−1], +delta, +privileged
   force} × {contact task, reach-only control}: does the gain track the
   physical latent (vanishes under privileged force) or the history channel
   (persists)?
3. **Hardware calibration** (user's bench session; script ready): delta vs
   `Present_Load` vs `Present_Current` vs load cell, with Hwang and
   NeuralActuator as explicit comparison rows.

Retired from v1: four parallel "gaps" (collapsed to the sentence above);
the taxonomy claim as this paper's frame; any accuracy competition with
NeuralActuator; the copycat probe as this week's work.
