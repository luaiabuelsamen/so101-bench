# Here's what's new — the gap statement

*Synthesis of the four-thread reading pass (58 papers read/verified;
`docs/reading_notes/`). One page. Written 16 Aug 2026, before any further
building.*

## What the literature already has

**History in IL (A):** the canon never says history hurts — Wen's own copycat
paper shows history-BC beating single-frame in 5/6 of its environments; the
shortcut is a *tax*, with a measured signature (validation loss improves,
rollouts degrade, action-predictability exceeds the expert's). Three failure
regimes are named: shortcut mimicry (Wen/de Haan), latching on unobserved
contexts (Swamy, NeurIPS 2022), and episodic under-use (Past-Token
Prediction). **Sensorless force (B):** torque-from-(command, position) on RC
servos exists (Hwang 2018); the 2025–26 wave (NeuralActuator — 0.36–0.73 N
MAE on our very servo; FACTR 2 — force-from-currents+tracking-error into BC,
+17% on commodity arms) is converging on "force is latent in shipped
hardware." **Shields (C):** enforce-don't-learn is canonical
(Simplex→RTA→shielding), but every published enforcing filter consumes
models, state estimates, perception, or F/T sensing; none addresses gripper
crush; none audits the successes it deletes. **The stacks (D):** all seven
dominant open policies observe at most current joint positions; ACT *states*
that force "is implicitly defined by the difference between" leader and
follower positions and then observes only one operand; LeRobot reads
`Present_Position` while its own table defines `Present_Load`; Robotiq ships
grasp-loss detection computed from exactly sign(goal−present)+current.

## The gap, precisely

1. **An unnamed fourth regime in the history-in-IL taxonomy:** history as a
   *one-step physical measurement of a fast exogenous latent* — the servo
   loop's rejection of the command IS a force reading. Every published
   "history necessary" construction is an occlusion, a hidden velocity, or an
   episodic fact; none is an actuator latent. Our result sits in this cell,
   and by de Haan's own diagnostic (teacher-forcing unchanged, rollouts 10×
   up — the mirror of the confound signature) it is not the shortcut regime.
2. **Retroactivity, claimed by no one:** the channel is recoverable from the
   position-only logs every public LeRobot dataset already ships. NeuralActuator
   needs the load register; FACTR 2 needs current telemetry and calibration;
   neither exists in the corpus. This claim has a closing window.
3. **A shield that consumes only bus telemetry** — no model, no state
   estimator, no sensor — enforcing a constraint (grasp crush) that no
   published filter family covers, with paired accounting of deleted
   successes, which no published shield reports.
4. **The connection itself:** our point exists in the literature as two
   disconnected halves — ACT's sentence and FACTR 2's estimator — and nobody
   has joined them: *the difference the field names and then discards is a
   sufficient observation, retroactively, at zero hardware cost.*

## The dangerous neighbours, and the line against each

- **NeuralActuator** (RSS'26): better accuracy, same servo — but requires
  live load telemetry + an instrumented training rig; cannot touch the corpus.
- **FACTR 2**: owns "force input improves BC on commodity arms" — but via a
  trained estimator on current telemetry; never engages the copycat
  literature; not retroactive. (Steal: its free-motion self-labeling trick.)
- **Haddadin/De Luca residual line**: needs identified dynamics; targets
  link/human collision, not grasped-object crush (their own footnote flags
  fragile objects and moves on); never wraps a learned policy.
- **Hwang 2018**: the raw idea's prior art; cite and supersede with the
  calibrated envelope + the design rule.

## What this retires in our own claims

Accuracy competitiveness (NeuralActuator wins on numbers); the guard as a
*concept* (thresholded residual reactions are 2005); "the field ignores
history without justification" (ACT cites causal confusion; DP measured
history hurting — the field's actual error is conflating *history* with
*load*).

## The experiments the gap implies, ranked

1. **Wen's predictability probe on our existing checkpoints** (immediate,
   cheap): is base_hist copycatting? Our regime claim predicts no.
2. **Channel-content substitution grid**: {base, +a[t−1], +delta,
   +privileged force, +both} × {contact task, reach-only control}. Thesis
   predicts a[t−1]'s gain vanishes under privileged force; latching predicts
   it persists. This is the paper's decisive figure.
3. **Corpus regrasp study** (parked script): the retroactivity claim made
   empirical, ecosystem-wide.
4. **Hardware calibration** (blocking, script ready): delta vs Present_Load
   vs Present_Current vs load cell — with Hwang and NeuralActuator now the
   explicit comparison rows.

## The sentence, survived and sharpened

The reading pass confirms the thesis is early-consensus (the field is
arriving), which converts it from a bet into a race: the defensible, unclaimed
ground is **retroactive observability of actuator latents in the existing
corpus, the honest validity envelope, the margin-tracking design rule, and
telemetry-only enforcement** — and the deadline is set by how fast the
NeuralActuator/FACTR line reaches the same corpus insight.
