# Reading program (rule 2: read before build — 30–50 papers before the next line of code)

Goal for the week: place the sentence in the literature and find the gap that
generates the next question. Every entry ends with "what it means for the
thesis." Verified seeds below; the rest get filled during the reading pass —
no citation goes into a paper unread.

## Thread A — Observation histories in imitation learning (THE tension)

Our `base_hist` result — raw `a[t-1]` in the observation lifting BC from ~5%
to ~60% — sits in *direct tension* with this literature, which says history
inputs cause shortcut failures:

- **de Haan, Jayaraman, Levine — "Causal Confusion in Imitation Learning",
  NeurIPS 2019.** More information can yield *worse* BC performance; the nuisance
  correlate the policy latches onto is often its own past action.
  [paper](https://pure.uva.nl/ws/files/54674966/NeurIPS_2019_causal_confusion_in_imitation_learning_Paper.pdf) ·
  [site](https://sites.google.com/view/causal-confusion)
- **Wen, Lin, Darrell, Jayaraman, Gao — "Fighting Copycat Agents in
  Behavioral Cloning from Observation Histories", NeurIPS 2020.** Names the
  copycat shortcut precisely: when expert actions autocorrelate and the past
  action is recoverable, the imitator predicts the *previous* action.
  [arXiv 2010.14876](https://arxiv.org/abs/2010.14876)

**The gap question this thread hands us:** why did action history *help* here
when the field's canonical results say it hurts? Candidate answer worth
testing: history hurts when it only carries the shortcut, and helps when it
carries a *task-necessary latent* (grasp/load state) that the rest of the
observation omits — i.e., the copycat trade-off flips sign with the
observability deficit. Our 14× counterfactual + the ≤24% no-history plateau
are evidence on the "helps" side; a copycat-style probe on our checkpoints
(is the policy predicting a[t-1]?) is the missing measurement. That question
— *when does history help vs hurt in BC* — is thesis-grade and nobody has our
causal instrumentation for it.

## Thread B — Sensorless force/torque estimation (the mechanism's home field)

- Momentum-residual observers for collision detection (De Luca line; Haddadin) —
  [DLR overview](https://elib.dlr.de/129060/1/root.pdf)
- **NeuralActuator (2026)** — neural per-actuator models for external-force
  perception on low-cost servos (OpenManipulator-X): the closest current
  work. [project](https://frank-zy-dou.github.io/projects/NeuralActuator/index.html) ·
  [arXiv 2607.11734](https://arxiv.org/html/2607.11734v1)
- Virtual force/torque sensors on low-cost and RC-class actuators —
  [Sensors 2019](https://doi.org/10.3390/s19112603) ·
  [virtual torque sensor for RC servos](https://www.researchgate.net/publication/328910852)
- Sensorless bilateral teleop for low-cost manipulators —
  [arXiv 2507.06174](https://arxiv.org/pdf/2507.06174)
- Learning-based force estimation baselines (dVRK study) —
  [arXiv 2405.07453](https://arxiv.org/pdf/2405.07453)

**Differentiators to defend after reading:** (i) retroactivity — delta is
recoverable from the *existing public corpus*, which no current/torque method
is; (ii) the guard needs no model at all; (iii) resolution-vs-margin as a
design rule. If reading kills any of these, the paper says so.

## Thread C — Runtime safety for learned policies

- Alshiekh et al., "Safe RL via Shielding", AAAI 2018 —
  [dblp](https://dblp.org/rec/conf/aaai/AlshiekhBEKNT18.html)
- Ames et al., control barrier functions (tutorial line)
- Brunke et al., "Safe Learning in Robotics" survey —
  [arXiv 2108.06266](https://arxiv.org/pdf/2108.06266)

**To establish:** whether any shield in this line operates from bus telemetry
alone, without a dynamics model or state-space constraint. If none does, the
guard's positioning writes itself.

## Thread D — to be filled during the pass (10–15 papers each)

- Partial observability / privileged-teacher distillation in manipulation
- The LeRobot/ALOHA/ACT/diffusion-policy stack papers (what observation specs
  they chose and why — the omission we're pointing at)
- Grasp-outcome detection / slip detection from proprioception

## Protocol

One pass, ~1 page of notes per 3–4 papers, each note ending in "for/against/
orthogonal to the sentence, and what it changes." Output at week's end: one
page — *here's what's new* — which becomes the related-work section and
decides the next build (corpus study stays parked until then).
