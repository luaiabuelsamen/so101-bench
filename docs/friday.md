# Friday paragraphs

One paragraph per week: what was predicted, what happened, what dies.
(Phase 3 cadence; first entry written early, 2026-08-16, with the run.)

## Week of 2026-08-17 (entry for Friday 2026-08-21)

The week's first finding is about our instruments, not the robot: every
result to date was measured on one of two eval harnesses (train_act's
`evaluate()` and eval_guard's rollout) that had never been cross-calibrated,
plus a third code path for the teacher — and a reference-class error (H100
rows quoted as the small-budget baseline) made a floor-level twin experiment
read as a 5x regression. The audit that followed: episode streams are
equivalent (harness-free teacher: 24–28/30 on both eval seeds); the two
harnesses agree where checked (twin checkpoints: 2–4/30 under both); the
training and env code is behaviorally identical to the historical tree
(diff: only inactive-by-default flags); and the historical 21/100 small-
budget cell reproduces at 11/100 on seed 0 — inside the pipeline's
documented 12-point seed spread but only marginally (p≈0.08), so seeds 1–2
are running before any baseline sentence is written. Remaining calibration
debt: the two harnesses have not yet been compared at a non-floor operating
point (H100 checkpoint, both harnesses, same seed — queued behind the
seeds). Until Gate 1 closes, the Task 2 grid does not start.

Also this week — the corpus figure (Task 1, done): predicted delta-at-seat
separates outcomes at d > 0.5 in ≥half of 20 public datasets; happened 2/16
(pooled d = −0.39, 546 episodes), with 9/16 at |d| > 0.5 and six
significantly *inverted* — teleop over-commands every close, so empty jaws
stall like full ones; in the wild the channel is a struggle signal, not a
success signal. Dies: the strong retroactivity claim; the corpus section
becomes recoverable-and-informative with task-dependent sign, i.e. the
enforcement story. And the guard: Rule 1 settled in two CPU-minutes what
three hypotheses and several GPU runs had not (teacher 25/30 through the
guard vs 28/30 raw — the guard is fine; the scaled policy's 0/30 under it
is a filed ticket). Demoted on review, per Rule 1c: "open-loop controllers
break behind a rate limiter" (definitional) and "entry transients defeat
action filters" (an unimplemented soft-start feature, not a limit). Dies
also: the guard-saga workflow itself — detector modes excavated at GPU
prices that chapter-3 controls or the Robotiq note already in thread D
supply at boot. Shielded-imitation collection is built and its small-budget
twin test came back floor-equal (2 vs 3/30) — unanswerable at a budget
where nothing works; the question moves into the grid's trained-through-
guard column.
