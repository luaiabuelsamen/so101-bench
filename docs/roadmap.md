# Study roadmap after review (2026-08-18)

Post-review experiment program for the lag-excess paper ([paper.md](paper.md)),
ranked by what each experiment decides, not by cost. Free fixes (terminology,
scoping, citations, CIs, seat-saturation paragraph) are already applied in
paper.md v0.4. This file holds the expensive tier.

## A. Bench calibration — first; everything else inherits from it

Load cell vs δ vs `Present_Load` vs `Present_Current` on the physical
STS3215, staircase protocol at three speeds (script ready per
[phase3.md](phase3.md)). Two additions from review:

- **Command-to-state latency distribution** on the real bus: settles the
  δ[t] = g[t−k] − q[t] alignment semantics for every downstream result.
- **Within-session drift** (thermal): repeat a reference staircase at
  session start/middle/end.

Also measure: hysteresis (up vs down staircase), direction dependence,
deadband, and contact-location dependence if the rig allows.

**Decides:** whether any newton claim exists; whether the envelope
structure (regimes, cliff) transfers.

**Kill criterion (write down before the session):** if δ vs load cell is
non-monotonic in the seated regime, or drifts by more than the task margin
within a session, the paper repositions as simulation-methods + corpus
forensics. That outcome is a finding, not a failure.

**Cost:** one bench day.

## B. Real collect-and-rollout — the conversion experiment

Same bench window as A. Collect 100-300 demos on a force-critical task
with a physical crush surrogate (objective success/damage indicator).
Train stock-obs vs +e paired, roll out paired. Run the guard physically on
the same episodes with paired deleted-success accounting.

**Decides:** whether the paper is hardware-relevant at all. A without B
leaves the learning results simulated.

**Cost:** a bench week + ~$10-20 training.

## C. Retroactive training on public data — the honest offline version

Review 2 proposed "retrain on public data"; rollout is impossible without
the source robots/scenes, so the honest form is:

1. **Offset sweep** g[t−k] − q[t] over integer k on the already-downloaded
   corpus data (CPU, free). Characterizes per-dataset timing; validates or
   bounds the Sec. 4.3 numbers.
2. Paired policies (stock vs +e) trained on 1-2 public SO-101 datasets,
   compared on held-out action prediction restricted to contact-phase
   frames.
3. **Stretch:** if a dataset's embodiment matches our arm and the scene is
   rebuildable, roll out a corpus-trained policy for real. Headline result
   if it works.

**Decides:** whether the retroactivity claim has teeth on real
over-commanded logs, given the seat saturation found in Sec. 4.3.

**Cost:** days; small GPU.

## D. E−C seed expansion — is the *subtraction* the contribution?

The scientifically central open pair. If E−C resolves, lag compensation is
the finding; if not, raw δ suffices and the story simplifies.

Power math at σ_seed ≈ 10: resolving the observed E−C = +4.8 needs ~35-40
seeds/arm; resolving C−B = +8.0 needs ~20/arm. More episodes per seed does
NOT substitute: variance is seed-to-seed training variance (arm A spans
0-15), not eval noise. Plan: run B, C, E to 20 seeds (~$40-60/arm at
~$2-3/run), extend only if the CI is tantalizing.

**Cost:** ~$200-300 total.

## E. Compensator ladder — feature engineering or physical latent?

Review 1's model-family list, done efficiently: evaluate free-motion
predictors OFFLINE first (held-out no-contact frames): ARX/FIR;
linear in q̇, q̈, direction; small MLP. Train only the offline winner as
arm E′.

- Richer compensators saturate at E → linear was enough.
- E′ beats E → current paper undersells the idea.

**Cost:** mostly CPU + 1-2 GPU cells.

## F. Privileged-force ceiling arm — one cell, bounds headroom

Oracle grip-force as input (from the original gap.md plan).

- Oracle ≈ E → the residual is close to a sufficient statistic (strong
  sentence).
- Oracle ≫ E → the form is lossy; sharpens interpretation of D's null.

**Cost:** ~$15.

## G. Data-scale sweep — scopes the "small data" claim

A/B/E at 280 vs ~1000 demos: does the channel advantage persist, shrink,
or vanish? Without this, "at small data" is an assumed regime boundary,
not a measured one.

**Cost:** ~$30-50.

## Sequencing

- A + B share the bench window; first.
- C.1 (offset sweep) runs today on CPU, no dependencies.
- D + F are fire-and-forget GPU jobs, parallel with bench work.
- E after C says whether real-log compensation matters.
- G last; its interpretation depends on D.
- Guarded-column runs already in progress (Ag/Bg/Cg/Dg in `results/`)
  complete the cost-neutrality claim independently.

## Submission hygiene (when repo goes public)

Tag the freeze commits so pre-registration timestamps are findable without
hashes in the prose: `prereg-grid` → d4077d5 (task2 arm definitions),
`prereg-corpus` → 7b7a818 (phase 3 / corpus protocol).
