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

**CORRECTION 2026-09-03 — C.2's declared metric cannot decide C.2's question.**
Step 2 above specifies "held-out action prediction restricted to contact-phase
frames" as the comparison. That metric is inert on this stack, and the project
already has the evidence: `findings.md:697` (autopsy) reports teacher forcing at
**~97% relative for BOTH arms in EVERY phase** — the networks learn the
demonstrations equally well with and without the channel. The autopsy's whole
point is that the discriminating signal lives in **closed-loop divergence**, not
in prediction accuracy: base predicts the demos at 97% while its decoded plan at
its own stalls contains 6 counts of motion against the demo lift's 471.

So a paired stock-vs-+e comparison on held-out action prediction is predicted to
return "no difference" regardless of which arm is right, and a null from it
carries no information. Do not plan a bench week around it.

What would actually decide C.2, in increasing order of cost:

1. **The counterfactual probe, not the prediction metric.** The autopsy's zeroing
   intervention (delta-policy stalls: zeroing the channel collapses the planned
   lift 482 → 35 counts, 14x; restoring a demo-typical value gives 566) is a
   closed-loop measurement that runs on a trained checkpoint without a simulator
   rollout. Applied to a corpus-trained policy at corpus contact frames, it asks
   whether the channel gates commitment *inside the network* — which is the
   retroactivity claim's actual content. This is the cheap replacement and it
   reuses `scripts/probe_counterfactual_commit.py`.
2. **Wen's action-predictability probe** (thread A entry 2): an MLP predicting
   `a[t]` from past actions alone, compared against the expert's own
   predictability. Discriminates shortcut-copying from channel use, and unlike
   teacher-forced accuracy it is *designed* to be sensitive where TF is flat.
3. Rollout on a rebuildable corpus scene — the step-3 stretch, unchanged and
   still the only thing that closes it properly.

Item 1 is desk work on existing code and should be run before C is scheduled at
all; if it is flat too, C's premise is what is wrong, not its metric.

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

---

## Submission gate: reconcile findings.md against main.tex (added 2026-09-04)

**Why this exists.** On 2026-09-03/04 the paper was found to be behind its own
lab record three separate times, all three correct in `findings.md` since August
and never propagated:

| lab-record verdict | where it sat | how long |
|---|---|---|
| guard deletes *every* success (12/270 → 0/270), not "one policy"; mixed denominators "costless to fix" | `findings.md` 2026-08-23 | 11 days |
| corpus counts are 10/16 and 7/16, not 9/16 and 6/16 | frozen `corpus_delta_outcome.json`, 2026-08-16 | 18 days |
| the M/κ constant "should be re-derived… before it is claimed as quantitative" | `findings.md` 2026-08-18 | 17 days |

All three were found by accident, in the course of auditing something else.

**Why `check_paper_numbers.py` cannot cover it.** That script closes one
direction only — the tex quoting a number the results contradict. It is blind to
a lab-record entry that says *"the constant is not right"* when the tex states no
number at all, which is exactly the quantization case. It is equally blind to a
correct number carrying a wrong claim (the guard's "one policy" sat beside a
correct 7/30). Those need a reader.

**The gate.** Before submission, one pass reading `findings.md` against
`main.tex`, entry by entry, asking of each: does the paper state this, contradict
it, or ignore it? Not a habit — a checklist item with an owner and a date, because
three misses in one day is a systematic gap and the two mechanical checkers we
have both miss it by construction.

**Convention going forward.** A `findings.md` entry whose conclusion the paper
must carry ends with an explicit line so the pass has something to grep for:

    **PAPER ACTION:** <what main.tex must say or stop saying>

Back-tagging the ~40 existing entries is a bounded one-time job and is the
cheapest way to make the gate mechanical rather than attentive.

### Bibliography: verified 2026-09-04, re-run if entries are added

Six of 28 entries carried invented author given names — surnames correct,
given names wrong, with "Liu, Sirui" appearing in three unrelated entries that
have no such author. All corrected; full table in `findings.md`. The affected
entries included `oh2026factr2`, the paper's central positioning reference.

**Standing rule:** every entry added to `refs.bib` is exported from arXiv or
Crossref, never typed. The check is one lookup per entry against arXiv's
`citation_author` metadata or Crossref's publisher-deposited record, and it
takes under a minute for the whole file.

Two open style decisions, not errors: `black2024pi0`, `shukor2025smolvla` and
`nvidia2025groot` use `and others` (hiding 14, 4 and 33 authors) — legitimate
BibTeX, but most robotics venues want full lists.

### Read `wong2026beyond` in full before submission

Positioned from the abstract only (`related_table.md`, cell 4). It is the
closest published work to this paper — same channel, real hardware, four
force-critical tasks — so abstract-only is enough to position against and not
enough to be sure the paper does not restate something in their discussion.
MDPI-style access was not the blocker here; nobody has read it.

### Open item found by the first pass (2026-09-04)

`findings.md:1202` records an unmet precondition that has not propagated:
*"Worth one confirmation before it ships: read addr 48 on a second SO-101 that
has been through lerobot connect, to show the value is written rather than
inherited from how this particular arm was flashed."*

App. D §"The ceiling belongs to the stack, not to this arm" concludes it is
"a property of the ecosystem rather than of one bench" on a register read from
**one** arm. The code-path argument carries most of the weight and is strong, but
the measured leg is n=1 and the paper does not say so.

**A better test than the one the entry proposes.** A second *follower* only
replicates. The leader discriminates:

- `so_follower.py:172-175` writes `Max_Torque_Limit=500` (plus
  `Protection_Current=250`, `Overload_Torque=25`) — **gripper only**.
- `so_leader.py:127-131` `configure()` calls `disable_torque()`,
  `configure_motors()`, and sets `Operating_Mode` — **no torque-limit write**.

Both arms came from the same vendor and flash; only the follower goes through the
path that writes 500. So follower gripper = 500 with leader gripper = 1000 proves
the value is *written by the stack*, not inherited. That is the claim App. D
actually makes, and it costs one register read.

**Footgun, and the reason this is not already done.** Connecting to the leader
with a *follower* class would write 500 to it and destroy the evidence
permanently — the test is unrepeatable on that arm once contaminated. Read addr 48
over a raw bus handle; never via `SOFollower.connect()`. Until it is run, App. D
should disclose that the register read is n=1 and that the code path is the
load-bearing evidence.
