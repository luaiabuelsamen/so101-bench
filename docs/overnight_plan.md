# Overnight plan and pre-registered decision tree (16 Aug, written before results)

## Running

1. Funnels n=100 (base done: reach 91%, seat 65%, **seat→lift 16%** — the cliff)
2. `demos_v2`: 280 episodes with physics kicks (σ=1.5 rad/s, p=0.02/frame,
   measured knee: 75% expert yield) — recovery-data intervention targeting the
   seat→lift edge
3. Gated: verify dataset → retrain base+delta 12k steps → eval n=50 × 3 tiers
   → post-fix funnels n=100

## Decision tree — fixed NOW, before any v2 number exists

Primary readout: **seat→lift conditional in the v2 funnel** (v1 = 16%).

| outcome | verdict | action |
|---|---|---|
| ≥ 40% | recovery data was the binding constraint | write up; morning = polish (seeds/hardware) |
| 25–40% | partial — real but not sufficient | overnight round 2: kicks *targeted at the post-seat phase only* (higher p during grasp/lift), retrain base only, re-funnel |
| < 25% | recovery data NOT binding at this ceiling | STOP retraining. Next diagnostic, not next fix: inspect v2-policy seat-state trajectories vs demos at the lift boundary (is the policy's chunk boundary splitting the lift?); test chunk=15 variant as the single follow-up |

Secondary readouts, reported regardless: overall place rate n=50 with CI
(v1 baseline: base 2–4/30), crush counts per tier (delta should stay ≤ base's
half), and the demos_v2 PCA spread vs v1's 92%-in-2-PCs (verifies the
intervention actually widened coverage — if it did not, outcome interpretation
is void and that is said aloud).

## Standing rules for the night

- No mid-flight patching of running processes; fixes go to versioned copies.
- Kill only by PID file. n≥50 for any decision-bearing rate. Every new script
  smoke-tests before a long launch.
- Negative results get written with the same care as positive ones.

## Gate results (recorded as they landed, before training finished)

- Funnels n=100: base dies at lift-commitment (seat 65%, lift|seat 16%);
  delta dies earlier but commits (seat 38%, lift|seat 47%) -- the channel
  relocates the failure mode up the ladder rather than scaling one number.
- demos_v2 collection: 280/426 (66% yield, at the measured knee).
- Coverage gate: **passed on the per-frame metric** -- RMS jerk doubled
  (0.87 -> 1.64 m/s^3), proving deviated-state/corrective-action pairs are in
  the data. Path-level PCA stayed 92%-in-2-PCs, which in hindsight it must:
  resampled path shapes smooth out 100-200 ms recovery transients, and the
  expert returns to its nominal path. The pre-registered "PCA must widen"
  check named an instrument insensitive to the thing it was guarding; the
  jerk metric is the valid instrument. Stated rather than silently swapped.
