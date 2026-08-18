# Study roadmap after review (2026-08-18, rev 3)

Post-review program for the lag-excess paper ([paper.md](paper.md)), ranked
by what each experiment decides, not by cost. Free fixes from both review
rounds are applied in paper.md v0.4: terminology (contact/load proxy),
scoped retroactivity, Wong et al. + FACTR 2 positioning, Robotiq fix,
bootstrap CIs with E-vs-B as declared primary, seat-saturation paragraph,
method spec for e[t], and scoped causal claims ("observed, not merely
predicted, for the auxiliary design tested").

**Correction found while writing the method spec:** the paper's equation
said e[t] = δ − K̂·q̇; the implementation (`scripts/train_act.py`) uses the
one-step COMMAND rate r[t] = g[t−1] − g[t−2], least squares through the
origin, free-motion frames = jaw command above its 30th percentile, all at
15 Hz in encoder counts. Paper now matches code. Keep them matched.

## Experiments

### A. Bench calibration — first; everything else inherits from it

Load cell vs δ vs `Present_Load` vs `Present_Current` on the physical
STS3215, staircase protocol at three speeds (script ready per
[phase3.md](phase3.md)). Additions from review:

- **Command-to-state latency distribution** on the real bus: settles the
  δ[t] = g[t−k] − q[t] alignment semantics for every downstream result.
- **Within-session drift** (thermal): repeat a reference staircase at
  session start/middle/end.
- Hysteresis (up vs down staircase), direction dependence, deadband, and
  at least one off-axis/contact-location condition.
- Repeated trials; a monotonicity plot and an honest failure envelope beat
  a perfect calibration constant.

**Decides:** whether any newton claim exists; whether the envelope
structure (regimes, cliff) transfers.

**Kill criterion (write down before the session):** if δ vs load cell is
non-monotonic in the seated regime, or drifts by more than the task margin
within a session, the paper repositions as simulation-methods + corpus
forensics. That outcome is a finding, not a failure.

**Cost:** one bench day.

### B. Real collect-and-rollout — the conversion experiment

Same bench window as A. Collect 100-300 demos on a force-critical task
with a physical crush surrogate (objective success/damage indicator).
Train stock-obs vs +e paired, roll out paired. Run the guard physically on
the same episodes with paired deleted-success accounting.

**Decides:** whether the paper is hardware-relevant at all. A without B
leaves the learning results simulated.

**Cost:** a bench week + ~$10-20 training.

### C. Retroactive training on public data — the honest offline version

Rollout of corpus-trained policies is impossible without the source
robots/scenes, so the honest form is:

1. **Offset sweep** g[t−k] − q[t] over integer k on the already-downloaded
   corpus data (CPU, free). Characterizes per-dataset timing; validates or
   bounds the Sec. 4.3 numbers.
2. Paired policies (stock vs +e) trained on 1-2 public SO-101 datasets,
   compared on held-out action prediction restricted to contact-phase
   frames.
3. **Stretch:** if a dataset's embodiment matches our arm and the scene is
   rebuildable, roll out a corpus-trained policy for real.

**Decides:** whether the retroactivity claim has teeth on real
over-commanded logs, given the seat saturation found in Sec. 4.3.

**Cost:** days; small GPU.

### D. E−C seed expansion — is the *subtraction* the contribution?

The scientifically central open pair. If E−C resolves, lag compensation is
the finding; if not, raw δ suffices and the story simplifies.

Power math at σ_seed ≈ 10: resolving the observed E−C = +4.8 needs ~35-40
seeds/arm; resolving C−B = +8.0 needs ~20/arm. **Reviewer suggestions of
"+5 seeds" are underpowered:** 5→10 seeds shrinks resolution only from ~11
to ~8 points, which can decide C−B at the margin but not E−C. More
episodes per seed does NOT substitute: variance is seed-to-seed training
variance (arm A spans 0-15), not eval noise. Plan: run B, C, E to 20 seeds
(~$40-60/arm at ~$2-3/run), extend toward 40 only if the CI is
tantalizing.

**Also decides the title.** The paper is named for the subtraction while
E−C is unresolved: the name bets on this experiment. Decision rule: if E−C
does not resolve after expansion, retitle around the channel (delta /
contact channel), not the compensated form.

**Cost:** ~$200-300 total.

### E. Compensator ladder — feature engineering or physical latent?

Evaluate free-motion predictors OFFLINE first (held-out no-contact
frames): current linear command-rate model; linear in rate + acceleration
+ direction; short causal FIR/AR; small MLP. Train only the offline winner
as arm E′.

- Richer compensators saturate at E → the minimal linear model is enough
  (compelling minimalism).
- E′ beats E → current paper undersells the method.

**Cost:** mostly CPU + 1-2 GPU cells.

### F. Privileged-force ceiling arm — one cell, bounds headroom

Oracle grip-force as observation input (from the original gap.md plan).

- Oracle ≈ E → the residual is close to a sufficient statistic.
- Oracle ≫ E → the form is lossy; sharpens interpretation of D's null.

**Cost:** ~$15.

### G. Data-scale sweep — scopes the "small data" claim

A/B/E at 280 vs ~1000 demos: does the channel advantage persist, shrink,
or vanish? Without this, "at small data" is an assumed regime boundary,
not a measured one.

**Cost:** ~$30-50.

### H. Object-property shift eval — eval-only, near-free

Evaluate EXISTING grid checkpoints under shifted object mass, stiffness,
friction, and crush threshold. No training cost.

**Decides:** whether the E policy learned contact regulation or one
scripted visual sequence. Upgraded from reviewer-"optional" because it is
eval-only and directly answers the "task bakes in the conclusion"
objection.

**Cost:** CPU/GPU eval hours only.

## Desk work (no hardware, no GPU)

- **Reproducibility bundle**: simulator parameters, task specification,
  all six observation transforms, per-seed success results, exact
  evaluation seeds, force-model code, bootstrap code, public-corpus
  inclusion/exclusion ledger, plotting scripts. Most exists in the repo;
  needs packaging + a README map. For double-blind venues: anonymized
  archive or submission-hosted supplement carrying the frozen protocol
  docs and their timestamps.
- **Venue check**: reviewer cited RSS 2026 Lab-to-Production workshop
  (encourages real-robot evidence, requires limitations section,
  double-blind). Unverified; verify CFP + deadline if targeting it.
- **Figure work**: eyeball fig_mech against its rewritten caption; §4
  figure count if page budget ever binds.

## Sequencing

- A + B share the bench window; first.
- C.1 (offset sweep) and H run now on CPU, no dependencies.
- D + F are fire-and-forget GPU jobs, parallel with bench work.
- E after C says whether real-log compensation matters.
- G last; its interpretation depends on D.
- Guarded-column runs already in progress (Ag/Bg/Cg/Dg in `results/`)
  complete the cost-neutrality claim independently.

## Positioning ladder (for whenever a deadline appears)

1. Now: "a controlled simulation study of a retrospective contact feature,
   with hardware calibration in progress."
2. After A: "a position-command-only contact proxy whose hardware behavior
   is measured and whose policy utility is demonstrated in controlled
   simulation."
3. After A + B: "a measured contact channel with a real-arm policy and
   guard result."

Deadline logic only: a clean, scoped simulation paper beats an incomplete
hardware claim. Absent a deadline, the study drives (A then B), not the
positioning.

## Rev 3: CoRL mock-review merge (2026-08-18)

Target is CoRL (8-page body), not a workshop. The mock review's verdict:
borderline as-is; the path to accept runs through hardware. Its priority
list maps onto A-H as follows; genuinely new items are marked NEW.

**P0 (their "do now") = A + B + D.** Bench calibration (A), one real
force-critical BC experiment (B), stronger E-vs-B/C power (D; their "more
seeds" advice stands but see the power math above; 20/arm on B, C, E
first). The review confirms evaluation pairing is already exhausted:
`train_act.py` rolls out every arm and seed on the identical episode
sequence (env seed 1000), so variance is training-seed variance.

**P1:**
- Force-oracle upper bound = F. Also contextualizes 26.6/100 against a
  ceiling (their point: without it, nobody knows if 26.6 is progress).
- NEW **k-sweep timing figure**: the offset sweep is already run (k=0..5,
  d-span 0.12, alignment k~3-4). Turn the sentence into a figure:
  effect size vs k, sim latency and corpus-inferred latency marked.
  Desk work; data exists.
- NEW **delta-variant ablation cells**: |delta|, sign(delta),
  delta-dynamics as additional arms; with F (seat bit) these answer "is
  the contribution force information or contact-state information?" F=C
  at 21.8 makes this a live question. ~$2-3/cell.
- Quantization x margin sweep as a major figure = the cliff experiment
  (fig1_cliff exists; extend to a 2D sweep if cheap). Elevate the
  "resolution-margin principle": quantum < M/kappa.
- NEW **real-vs-sim tracking-error traces** (part of bench session A):
  does the simulator reproduce observed delta behavior before any policy
  transfer claim.

**P2:**
- Compensator ladder = E (linear vs richer free-motion models).
- NEW **K-hat threshold sensitivity**: refit at 10/20/30/40/50th
  percentile free-motion criterion, show K stability. CPU, hours.
- NEW **cross-task / cross-speed K generalization**: fit K on task A,
  apply on task B. CPU.
- NEW **behavioral trace analysis**: at-contact traces of jaw command,
  position, delta, e, and policy action for base vs delta vs lag-excess
  policies; makes Fig 1 mechanistic. Existing checkpoints.
- NEW **camera-occlusion control**: a cell where the jaw/deformation is
  not visible, closing the "the cameras carry the signal" attack.
- NEW **guard Pareto accounting**: report crushes, successes, deletions,
  false interventions, peak force as one table (data in results/).
- H (object-property shift eval) stays; the review missed it but it
  answers their task-circularity concern for free.

**Paper changes already implemented in v0.6 main.tex** (convergent with
the review): retitle away from Lag Excess (now "A Retroactive Contact
Channel..."), lag excess demoted to recommended instantiation, E-vs-C
non-attribution stated with the seed-power math, E-vs-B declared primary
with the rest exploratory, abstract says "simulated force-critical task"
up front, corpus framed as informative negative/boundary, intro no longer
claims action semantics "met" (gates only), FACTR-2 special-case
positioning, six-servo correction, anonymized submission mode, method
details in App A, guard demoted to secondary with details in App C.

**Paper changes still open:** difference table in related work (methods x
{extra sensor, online, retroactive, works on position-only logs});
"SIMULATION" labels stamped on force figures (figure scripts);
"informative but not semantically invariant" sentence for the corpus
section; corpus subsection retitle to "boundary condition"; restore cliff
figure to body when the 2D sweep lands; behavioral panel under Fig 1.

**Sequencing unchanged:** the review's own bottom line matches rev 1:
hardware first (A then B), power second (D), everything else after. Final
framing waits on which experiments land, per the review's closing advice.

## Submission hygiene (when repo goes public)

Tag the freeze commits so pre-registration timestamps are findable without
hashes in the prose: `prereg-grid` → d4077d5 (task2 arm definitions),
`prereg-corpus` → 7b7a818 (phase 3 / corpus protocol).
