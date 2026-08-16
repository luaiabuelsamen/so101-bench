# Autopsy: why BC sits at 5-17% under an 83% expert

Checkpoints examined: act_v2/{base,delta} (best available at autopsy time).
Rungs 1-3 at full n; rung 4 at n=3 because the delta policy produced only 3
stalls in 20 rollouts (base: 9) -- the scarcity is itself evidence.

## Evidence

**R1 Teacher forcing (2400 demo frames/arm, per phase).** On demo states both
policies predict well in EVERY phase: arm first-step L1 9-15 counts against a
471-count lift motion (~97% relative accuracy); hold-phase jaw error 5-7
counts. No phase-specific representation failure exists on-distribution.

**R2 OOD curves (20 rollouts/arm vs 40k-frame KDTree, demo-replay baseline
p95 = 0.52).** base crosses the baseline p95 at frame ~177; delta's median
never crosses (0.36-0.44). The delta-informed policy stays closer to the demo
manifold in closed loop.

**R3 Stall-chunk inspection.** At its own stalled states, base plans NOTHING:
median chunk amplitude 6 counts (range 4-9) against the demo lift's 471. It is
not a weak attempt; there is no lift intention. delta's rare stalls contain
full-amplitude lift plans (median 482).

**R4 Counterfactual delta swap (n=3, delta policy).** Same stalled state:
as-is 482 counts of planned lift; delta input zeroed -> **35** (14x collapse);
delta set to the demo-typical hold value -> 566. The lift intention is
causally driven by the delta feature.

**Cross-check from the training rounds (coordinator update folded in).** Three
successive data recipes moved every stage EXCEPT base's commitment:
lift|seat = 16% (plain) -> 24% (kicks) -> 22% (depth-diverse), while the same
recipes moved seating 63 -> 81 and delta's place 5 -> 17. Data fixes what data
can reach; base's commitment never moved.

## Ranked causes

1. **Base lacks any observable commitment feature.** (R3: zero intention at
   stalls; R1: intention exists on demo states; three data recipes flat at
   ~16-24% lift|seat; R4: the delta input causally generates the lift plan.)
   From state+96px images alone, "gripped and ready" is not separable from
   "near block, not ready" at deployment states. Not a data problem.
2. **Compounding covariate shift.** (R1 excellent on-distribution vs R2
   divergence; recipes that widened coverage did move their targeted stages:
   seat 38->72 with kicks, 63->81 with depth diversity.) Curable by data --
   and largely already cured for the stages it governs.
3. **Delta's residual losses are execution-grade, not conceptual**: retention
   through transport (place|lift 42%) and occasional re-planned lifts.
   Standard scale levers apply (image resolution, demos, steps).

## Top two interventions, with predictions

1. **Commit to the channel and scale the winning recipe on it**: delta arm +
   v3-style data at ~600 episodes + 224px + 50k steps (H100). Commitment is
   already solved by the input (R4); shift and precision are solved by scale.
   Predicted: place 17% -> 35-45%.
2. **If a no-delta policy is ever required**: the only plausible route to an
   implicit commitment feature is temporal context (n_obs_steps > 1), since
   single-frame base provably cannot express it at these states -- but the
   channel exists on every SO-10x for free, so this is a fallback, not a
   recommendation. Do not spend further data recipes on base; three have
   returned ~0 on the commitment edge.

The one-line verdict: **BC is bad here because the decisive latent variable
(grasp state) is unobservable to the baseline policy; the tracking-error
channel makes it observable, and everything else is ordinary scale.**
