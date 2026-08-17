# Friday paragraphs

One paragraph per week: what was predicted, what happened, what dies.
(Phase 3 cadence; first entry written early, 2026-08-16, with the run.)

## Week of 2026-08-17 (entry for Friday 2026-08-21)

Predicted: delta-at-seat separates grasp outcomes at d > 0.5 in at least
half of 20 public datasets. Happened: 2/16 (pooled d = −0.39, 546
episodes) — but 9/16 show |d| > 0.5, six of them significantly *inverted*:
teleop operators over-command every close, so empty jaws stall like full
ones, successes are stereotyped low-delta closes, and failures carry
deep-jam high-delta frames. In the wild the channel is a struggle signal,
not a success signal. Dies: the strong retroactivity claim (corpus-mined
success labels from delta) — and the paper's corpus section now argues
recoverable-and-informative with task-dependent sign, which is the
enforcement story, not the learning-input story. Also this week: Rule 1
landed and immediately earned its keep — the guarded-policy 0/30 that had
three hypotheses spent on it (including a falsified counterfactual) was
settled by a two-minute pairing: the closed-loop teacher scores 25/30
through the same guard (raw 28/30, grip 39 N), so the guard is fine and
the policy failure is a filed ticket, not a result. Two real boundaries
fell out of the pairing for free: open-loop-timed controllers break behind
a rate-limited plant (clamp expert 30→0 — wrapper-alters-plant is general,
not policy-specific), and no action filter governs contact-entry
transients (60 N tier fails raw AND guarded). Dies also: my guard-saga
workflow — four detector modes excavated at GPU prices that chapter-3
controls (or the Robotiq note already in thread D) supplies at boot. Task
2 grid starts Monday on the sim result, with trained-through-guard as the
added column and the guard proven on a working reference first.
