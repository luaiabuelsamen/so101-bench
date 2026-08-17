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
enforcement story, not the learning-input story. Also this week: the jaw
guard needed two more phantom-seat fixes on the scaled policy (onset
latency, creep-tracking); the detector is now validated offline on
instrumented traces (0 phantoms / 0 misses, force-at-seat ≤ 49 N vs raw
90–156 N peaks) with the A/B on a held-out seed running. Task 2 grid
starts Monday on the sim result, as the instruction says it should.
