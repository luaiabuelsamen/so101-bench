# Enforce, Don't Learn: Actuator-Level Runtime Safety for Learned Policies on Low-Cost Arms

*Draft for a workshop submission (low-cost manipulation / safe learning).
Four pages, three figures + one hardware figure pending. Full lab record with
every n and CI: `findings.md`.*

## Premise, and the baseline question answered first

Low-cost servo arms (SO-100/101 and kin) ship without force sensing, and the
policies people train on them crush what they grasp. The STS3215 servo does
expose `Present_Load` and `Present_Current` over the bus -- so why build
anything on the tracking error `delta = goal - present`? Two reasons, one
argued and one measured. Argued: **the public LeRobot corpus records neither
load nor current** -- positions and goals only -- so `delta` is the only force
channel recoverable from the thousands of datasets that already exist (we
measured it across 555 of them: median 1.5 distinguishable levels during
contact holds at factory gains). Measured: our calibration protocol
(`hardware/staircase_cal.py`) logs `delta`, `Present_Load`, and
`Present_Current` against a load cell in one session; the comparison figure is
the first figure of this paper, and if the dedicated registers dominate on
live hardware, the claim narrows honestly to the recorded-corpus and
runtime-guard use cases below.

## Claim

**Safety properties of learned manipulation policies on low-cost arms should
be enforced at the actuator layer, not learned.** We support this with a
~30-line, bus-observable jaw guard -- stall-detected contact plus a bounded
squeeze cap -- that eliminated **96% of crush events (85/180 → 3/180)** across
three behaviour-cloned policies on identical paired episodes, with no
retraining and no privileged state; and with the converse result that
learning-side interventions do not deliver safety: adding the load channel to
the policy's observation halves crush events at native resolution, the effect
vanishes at 16-count resolution, and scaled, competent policies (57-63%
success) crush *more*, because demonstration-faithful cloning reproduces the
demonstrator's force distribution.

## Positioning

Sensorless force estimation is decades old -- generalized-momentum residual
observers for collision detection (De Luca; Haddadin), disturbance observers,
and current-based estimation on hobby-class actuators -- but that line
targets estimation accuracy on torque-controlled or current-readable
platforms. Runtime enforcement for learned policies is likewise established --
shielding (Alshiekh et al., AAAI 2018), control barrier functions (Ames et
al.), and the safe-learning-in-robotics survey of Brunke et al. -- but those
shields assume a dynamics model or a constraint expressed in state space. Our
contribution sits at the intersection the low-cost ecosystem actually
occupies: no torque sensing, no calibrated dynamics, kilohertz-free bus
telemetry only -- and shows a contact-safety shield needs nothing more than
the position register history the cheapest servo already provides.

## Evidence (figures)

1. **Foundation -- the channel is real but bounded** (calibration): linear at
   2.00 N/count seated and quasi-static (held-out RMSE 4.3 N over 4-78 N),
   non-informative during sliding, meaningless during fast transport;
   seating detectable at recall 1.00 / adversarial precision 0.78. Pending
   twin figure on hardware: `delta` vs `Present_Load` vs `Present_Current`
   vs load cell.
2. **Resolution requirement tracks the physical margin** (Fig 1): coarsening
   the channel collapses crush-aware grasping as a cliff whose location moves
   with the safety margin -- a design rule (`quantum < margin / N-per-count`)
   rather than a benchmark number.
3. **The guard** (Fig 2): 85/180 → 3/180 crushes across three checkpoints;
   median peak force 33-131 N → 13-26 N. Measured caveats included: a force
   cap deletes successes earned by over-gripping, and the stall test must
   compare measured motion against servo-feasible (not commanded) travel or
   fast-commanding policies trigger phantom seats.
4. **Learning does not substitute** (Fig 3): the BC ladder 5% → 57-63%
   success shows competence scaling with data and compute while crush rates
   *worsen*; observation-side fixes are resolution-fragile; policies without
   any action-history input never learn grasp commitment at all (16-24%
   plateau, causally attributed by input counterfactual, 14x).

## Screening protocol (secondary contribution)

Our rigid-block negative control showed a standard pick-place task cannot
detect force feedback at all (blind clamp ties oracle, 118/120 paired ties).
We propose oracle-first screening -- run blind/proxy/oracle scripted experts
before any training -- as a cheap admission test for any benchmark claiming to
study a sensing modality.

## Limitations

All force constants currently inherit the simulator's contact model; the
hardware calibration (in progress) either transfers them or replaces them --
a negative transfer result would itself be the first honest measurement of
this channel. Two seeds per learned arm. One gripper, one object family. The
guard's precision against jams is 0.78 and inherited by everything above it.

## Six-week plan to submission

W1: three-channel load-cell calibration on a real STS3215. W2: literature
positioning pass (20 papers, one page). W3-4: guard on the real arm, A/B
video against a deliberately over-gripping policy. W5-6: 4-page writeup,
three figures, workshop submission.
