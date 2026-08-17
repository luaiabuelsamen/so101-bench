# The Servo Already Knows: Tracking Error as a Free Force Channel on Low-Cost Arms

*Draft for a CoRL/RSS workshop submission (low-cost manipulation). Two pages,
three figures, one pending hardware plot. The full lab record with every n and
CI is `findings.md` in this repo.*

## Claim

Position-servo arms with no force sensor still expose a force signal: the
servo's own tracking error, `delta = goal − present` in encoder counts, which
is already recorded in every LeRobot-format dataset as
`action[t−1] − state[t]`. We characterise this channel end-to-end on a
simulated SO-101 and show two deployable results: **(1)** a task-level force
*resolution* requirement that tracks the physical safety margin — coarsen the
channel past `margin / (N-per-count)` and crush-aware grasping collapses as a
cliff — and **(2)** a ~30-line, bus-observable jaw guard that eliminated
**96% of crush events (85/180 → 3/180)** across three behaviour-cloned
policies with no retraining. Secondarily, adding the channel (or the raw
action history it derives from) to a standard ACT policy is what makes grasp
*commitment* learnable at all: without any such input, policies plateau at
16–24% lift-commitment across three data recipes; with it, 47–76%, and a
counterfactual intervention on the trained network (zeroing the input at a
stall) collapses the planned lift 14×.

## Method sketch

MuJoCo SO-101 with validated contact (teleop replays grasp 8/10), integer
encoder quantisation on both observation and command, and a privileged
scripted expert used three ways: demonstrator, force-oracle upper bound, and
DAgger-style correction source. The channel was calibrated against
ground-truth contact force (linear at 2.00 N/count seated & quasi-static,
held-out RMSE 4.3 N over 4–78 N; non-informative while the object slides;
meaningless during fast transport — deviations to 36 N p95), its seating
detector evaluated against adversarial negatives (recall 1.00, grasp-ready
precision 0.78: it detects *obstruction*, and jams obstruct like seats), and
the physical safe-grip window measured without feedback (F_min = 20 N under a
fixed 3.7 s transport protocol). All decision rules, thresholds, and stopping
criteria were written before the corresponding results (repo:
`docs/overnight_plan.md`).

## Results

**Fig 1 — the cliff tracks the margin.** Sweeping observation quantum × crush
limit for the delta-regulated scripted controller: success is flat until the
quantum approaches the safety margin, then collapses; the collapse point moves
right as the margin widens (100/120/160 N). An input-ablation artifact would
fall equally everywhere; this tracks physical headroom.

**Fig 2 — safety as architecture.** The guard (stall-detected seating + a
4-count squeeze cap, bus signals only) on three ACT policies, paired episodes:
85/180 crush events raw → 3/180 guarded, median peak force 33–131 N → 13–26 N.
Caveats measured, not hidden: a force cap also removes successes a sloppy
policy earned by over-gripping, and the stall test must compare measured
motion against *servo-feasible* travel or fast-commanding policies trigger
phantom seats.

**Fig 3 — the channel is what makes commitment learnable.** BC ladder under
identical architecture: 5% → 17% (recovery kicks) → 21% (grip-depth
diversity) → 57–63% (600 demos, 224 px, 50k steps), reaching the deployable
scripted band (expert + realistic pose noise: 47–67%). Policies *without*
action-history input stayed at ≤24% commitment through every data recipe; the
teacher-forcing / counterfactual autopsy locates the failure at observability,
not optimisation. At scale, explicit `delta` and raw `a[t−1]` are
statistically indistinguishable (57% vs 60%, 2 seeds — form-vs-information
unresolved at this n; the *information* being necessary is the supported
claim).

**[Fig 4 — pending: real-arm calibration.]** Goal-position staircase into a
load cell on a physical STS3215 (`hardware/staircase_cal.py`, ~20 min):
N-per-count, linearity range, saturation. Until this lands, every newton in
this paper inherits the simulator's contact model — stated as the primary
limitation, not a footnote.

## Limitations

Simulation contact stiffness (elliptic cone, `impratio=10`, box pads) sets
the 2.00 N/count constant; the real servo adds PWM deadband and
load-dependent stiction. Two seeds per learned arm bound only coarse claims.
The guard's stall detector inherits a measured 0.78 precision against
jams/topping. One gripper, one object family, one task.

## Reproducibility

Everything in this repo: environment, expert, all experiment scripts,
pre-registrations, result JSONs with CIs, and the figures above
(`figures/paper/`). Cloud training reproduces via `scripts/modal_train.py`
(~$26 on H100s for the full grid).
