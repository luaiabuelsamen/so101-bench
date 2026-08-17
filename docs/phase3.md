# Phase 3 — instruction (received 2026-08-16, verbatim; operational freezes below)

Thesis (every task below argues for or against this, nothing else is in scope):
  Position-only logs shipped by the SO-100/101 ecosystem contain a sufficient
  force observation (delta = action[t-1] - state[t]). Stock stacks omit it.
  Making it observable, with a design that works at small data, is the claim.

Rule 0: analysis is bounded, design is central, hardware runs in parallel.
No task produces a script and calls it a result. A result is a figure with a
claim under it, n and CI, and a written prediction made before the run.

## Task 1 — Corpus figure (bounded: 1 week, 1 script, 1 figure, then STOP)
- Pull 20 public SO-100/101 LeRobot datasets. Compute delta per frame from
  action/state only. Detect gripper close events with the frozen stall detector.
- Figure 1: delta at seat vs episode outcome (success proxy: gripper stays
  closed and moves; define once, do not iterate).
- Prediction written before running: delta at seat separates outcomes at
  effect size > 0.5 in at least half the datasets. If it doesn't, write the
  negative and continue anyway — the design task stands on the sim result.
- No further corpus analysis after Figure 1 unless it kills a claim.

## Task 2 — Design (central, weeks 2–5)
Ablation grid, all trained on the same 200-episode demos_v3 at 12k steps,
small-data regime, sim, 3 seeds each, n=100 eval, crush tiers -1/120/60:
  A  base           s[t]
  B  base_hist      [s[t], a[t-1]]
  C  delta_input    [s[t], delta]
  D  residual_head  encoder predicts delta as auxiliary target, fused
  E  load_prior     per-servo delta→load map pretrained on free-motion
                    self-labeling, frozen, output fed to policy
  F  event_token    stall/seat detector output as a discrete token gating the chunk
Prediction before running: at small data, D/E/F beat B; at scale (Task 2b,
one seed, 50k steps) they converge. If B matches D/E/F at small data, the
design claim is dead and the paper is the observability claim only. Say so.
Task 2b: rerun best of D/E/F vs B at 600 eps / 50k steps, one seed.
Guard wrapper is a fixed baseline row across the grid, not a new experiment.

## Task 3 — Hardware (parallel, from week 2, not deferred)
- Load cell on one real STS3215. Staircase of goal position at 3 speeds.
  Log delta, Present_Load, Present_Current, load cell.
- Figure: N vs counts. Comparison rows: our sim 2.0 N/count, Hwang 2018,
  NeuralActuator reported error. If real slope/saturation differ from sim,
  update sim actuator, rerun Task 2 best config, report the shift.
- Then: best Task 2 policy on the real arm, guard raw vs wrapped, 20 paired
  episodes, video.

## Task 4 — Writeup (week 6, mandatory regardless of results)
4 pages, 3 figures: corpus, ablation grid, hardware calibration.
One sentence of claim at the top. Negative results at full strength.
Target: workshop. Send to two humans before submitting.

Cadence: every Friday one paragraph — what was predicted, what happened,
what dies. Anything not on this list needs a sentence for why it serves the
thesis, or it isn't started.

Rule 1 (added 2026-08-16 after the guard saga): no null result enters
findings.md without (a) a paired working reference under identical protocol,
(b) that reference failing across the claimed boundary, (c) a citation for
the nearest prior method and why it doesn't cover this case. Otherwise it's
a bug, filed as one. A 0/N is a defect until proven a boundary. In practice:
before a learned policy's failure gets a sentence, run the scripted expert
under the identical protocol, and ask what a gripper engineer (Robotiq:
seat by stall, hold by position offset, monitor slip — it was already in
thread D's notes) would have built before running anything.

---

# Task 1 operational freezes (written 2026-08-16, BEFORE the run; not to be iterated)

**Close event** (gripper ACTION trace, hysteresis): close = command drops
below lo+0.30·range and stays ≥5 frames; reopen = climbs above lo+0.60·range
sustained ≥5 frames. Single-grasp-task gate: the median episode in a dataset
must close exactly once, else the dataset is excluded as multi-grasp by
design (chess/sorting/assembly), where any pick-outcome proxy is undefined.

**Seat / stabilization frame T** (offline form of the frozen stall detector):
first frame after close onset — and after closing has visibly begun
(measured jaw moved ≥3 counts from onset, or command ≥2 counts beyond
measured) — where the measured jaw moves <1 count for 4 consecutive frames.
Seated flag = command–measured gap ≥ 2 counts at T (stalled away from
command) vs reached-command.

**delta at seat** = mean over frames [T, T+8] of |a[t−1] − s[t]| on the
gripper dim, in encoder counts (per-dataset quantum from the 555-dataset
study, empirical fallback). Magnitude, because a seated SO-101 jaw rests
ABOVE its deeper command (signed gap is negative) while a clean miss tracks
to zero gap; the unsigned error is the force reading.

**Success proxy** (define once): success = (no reopen event between T and
the final 10% of the episode) AND (transport: max over t>T of
‖s[t,:5] − s[T,:5]‖₂ ≥ 60 counts). Known noise, accepted and frozen: a drop
after a good grasp with late re-grasp counts as failure with possibly high
delta; a task completed without transport counts as failure. The proxy is
not iterated after this line.

**Pre-registered prediction**: successes sit at HIGHER delta-at-seat;
Cohen's d > 0.5 in ≥ 10 of 20 datasets. Secondary: pooled AUC > 0.5 after
within-dataset z-scoring. Negative result gets written at full strength and
Phase 3 continues on the sim result.

**Figure 1**: forest plot — per-dataset Cohen's d with bootstrap 95% CI,
reference lines at 0 and 0.5, marker area ∝ episodes; pooled d at bottom.

**Friday paragraphs** land in `docs/friday.md` starting 2026-08-21.
