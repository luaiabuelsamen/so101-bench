# Related-work difference table (draft)

Roadmap item `docs/roadmap.md:235` — "difference table in related work
(methods x {extra sensor, online, retroactive, works on position-only logs})".

**Status: draft, needs verification against the primary sources.** Every cell
below is derived from the characterisations already written in
`paper/main.tex:131-178`, not from re-reading the cited papers. That is enough
to lay out the axes and see where the paper sits, and not enough to publish.
Before this goes into `main.tex`, each row wants a check against the actual
paper — the rows marked ⚠ are the ones where the current prose does not fully
determine the cell.

## Column definitions

Stated explicitly because "extra sensor" is ambiguous on a servo bus, where
current and load are registers rather than added hardware.

| column | means |
|---|---|
| **beyond position** | needs a signal other than commanded + measured joint position — torque, current, load register, tactile, F/T |
| **online** | must run at collection or execution time; cannot be reconstructed later |
| **retroactive** | computable after the fact from an archived log |
| **position-only logs** | runs on a log carrying only synchronized commanded and measured positions — i.e. the public SO-100/101 corpus as it already exists |

`online` and `retroactive` are near-complementary by construction; both are kept
because the roadmap named both, and because a method can be neither when it
needs a calibration stage that no archived log can supply.

## Table

| method | beyond position | online | retroactive | position-only logs |
|---|:--:|:--:|:--:|:--:|
| **Sensorless force estimation** | | | | |
| Momentum-residual observers `deluca2005sensorless, haddadin2017collisions` | yes (torque) | yes | no | no |
| Virtual force sensors, low-cost `yen2019virtual` | yes ⚠ | yes | no | no |
| Torque from commanded vs. measured position `hwang2018virtual` | **no** | no | yes ⚠ | **yes** |
| Series-elastic actuation `williamson1995sea` | yes (deflection, by design) | yes | no | no |
| NeuralActuator `dou2026neuralactuator` | yes (per-actuator model) | yes ⚠ | no | no |
| FACTR 2 `oh2026factr2, liu2025factr` | yes (estimated ext. torque) | yes | no | no |
| Leader-follower discrepancy `wong2026beyond` | no (action labels) | no | yes ⚠ | yes ⚠ |
| **Grasp monitoring / runtime safety** | | | | |
| Robotiq object-detection register `robotiq2019manual` | yes (current limit) | yes | no | no |
| SCHUNK workpiece-loss `schunk_egk` | yes (integrated encoder) | yes | no | no |
| Tactile grasp monitoring `calandra2017feeling` | yes (tactile) | yes | no | no |
| Academic safety filters `alshiekh2018shielding, hsu2024safetyfilter, brunke2022safelearning` | yes (model / state est. / exteroception) | yes | no | no |
| **This paper** | | | | |
| Raw channel δ = a[t−1] − s[t] | **no** | no | **yes** | **yes** |
| Lag excess e (free-motion compensated) | **no** | no | **yes** | **yes** |
| Runtime guard (Sec. 4.4) | no | yes | — | n/a (enforcement, not observation) |

## What the table actually shows

The bottom-right cell is not empty, and that is the point of including it: the
`hwang2018virtual` row already sits there. `main.tex:139-141` says so directly
— "prior art for the raw channel". So the table must not be captioned as though
position-only force recovery is new here. What is new, on the paper's own
telling, is (a) the free-motion compensation that turns δ into e, described as
"a deliberate special case of FACTR 2's central idea, a linear free-motion
baseline fit on positions alone", and (b) the observation-design grid asking
which *form* of the channel a small-data imitation policy needs.

The honest one-line reading of the column: everything else in the table either
consumes telemetry beyond position, or needs a calibration stage, or both — so
none of it can be applied to the 500-odd archived SO-100/101 datasets, which is
the claim the paper actually needs.

## Second table — observation design, where the axes above do not discriminate

*Added 2026-09-03.* The axes above were built for force-estimation methods and are
degenerate on the observation-design literature: every row below is "beyond position:
no / retroactive: yes / position-only logs: yes", because they all edit what the policy
sees, not what the robot senses. Discriminating them needs a different axis — **what
latent the added observation resolves**, and **what it costs in observation width**.

| method | added observation | latent resolved | belongs to | width |
|---|---|---|---|---|
| ACT `zhao2023act` (and the stack's default) | none (`T_o`=1) | — | — | 0 |
| Long-context diffusion + past-token pred. `torne2025pasttoken` | `T_o` frames + aux. past-action loss | episodic/semantic (what happened earlier) | the task | 10s of frames |
| Long-context reactive policies `zeng2026revisiting` | `T_o` = 12–20 frames | expert's hidden plan (sticky FSM state, teleop idiosyncrasy) | **the demonstrator** | 12–20 frames |
| FACTR 2 `oh2026factr2` | estimated external torque | contact force | **the world** | 6 scalars + current telemetry + calibration |
| **This paper** — δ = a[t−1] − s[t] | previous command | contact force | **the world** | 6 scalars, `T_o`=1 |

The column that carries the argument is *belongs to*. A latent that lives in the
demonstrator is recoverable from the demonstrator's own past states, which is why
`zeng2026revisiting` can buy it with 12–20 frames of context and why their Markovian-
expert control removes the need entirely. An exogenous physical latent is not
recoverable from position history at any length; it needs a channel that observes the
world's effect on the actuator. FACTR 2 and this paper are the only rows that resolve a
world latent, and they differ on cost — theirs needs current telemetry plus a
calibration stage, ours needs two numbers every SO-100/101 log already stores.

**Caveat, and it belongs in the caption:** the *belongs to* / width framing is the
paper's claim, not a measured result. `zeng2026revisiting` shows context length alone
reaching 93.2% on FurnitureSimOneLeg (vs 90.6% for the best short-context policy), and
our own B ≈ C is consistent with `a[t−1]` acting as one frame of context rather than as
a force channel. Arm G (`s[t−k:t]`, no `a[t−1]`) is the experiment that earns the row —
see `docs/reading_notes/thread_A_history_in_IL.md` Part 5. Until it runs, this table
states a distinction we have argued and not demonstrated, and must not be captioned
otherwise.

## Open cells (⚠)

1. `yen2019virtual` — **CLOSED, 2026-09-04. Current-based, as the row assumed.**
   Abstract (Sensors 19(11):2603): "the contact force observer estimates the
   external torques on each joint according to the **motor electric current** and
   calculation errors of the system model", with "a torque saturation limiter…
   added to the servo drive for each axis" for real-time stiffness control. So
   beyond-position = yes (current), online = yes, retroactive = no,
   position-only = no. Row stands unchanged. Bib authors verified exactly
   against Crossref.
2. `hwang2018virtual` — **CLOSED, 2026-09-04, and it resolves in the paper's
   favour.** A calibration stage *is* required. From the abstract (Crossref
   DOI 10.3390/s18113856; MDPI full text returns 403 and PMC is behind a
   CAPTCHA, so this is the abstract, not the method section): "a dedicated
   system identification method is developed for the proposed virtual sensor
   to implement the sensor in actual experiments", validated against "the
   measurements obtained by an actual sensor". The inputs at inference time
   are positions, but the identified model cannot be obtained from an archived
   log — it needs the physical servo. So the row becomes:

   | method | beyond position | online | retroactive | position-only logs |
   |---|:--:|:--:|:--:|:--:|
   | Torque from cmd vs. measured position `hwang2018virtual` | no | no | only after per-servo system ID | **no, not from an archive alone** |

   What this does to the section below: "The bottom-right cell is not empty…
   the `hwang2018virtual` row already sits there" is **no longer correct as
   written**. Hwang et al. are prior art for the *raw channel* — estimating
   torque from commanded versus measured position on hobby servos — which the
   paper should keep conceding, and are *not* prior art for retroactive
   computability on an archived corpus, which is the claim the paper needs.
   `main.tex:138-140` currently concedes both by saying only "prior art for the
   raw channel" without the identification qualifier, and so understates its own
   distinction. The exact requirements of the identification stage (known loads?
   a reference torque sensor? special excitation?) are **not** established —
   only that one exists and is a precondition for implementation.
   Full text still worth reading before submission.

   Bibliography error found in the same pass and fixed: `refs.bib` had
   "Hwang, Seonghyeon and Minami, Yuta"; Crossref's publisher-deposited record
   gives **Yoonkyu Hwang, Yuki Minami, Masato Ishikawa**. Two wrong given names
   on the paper's closest prior art, where a familiar reviewer would notice.
3. `dou2026neuralactuator` — **CLOSED, 2026-09-04 (abstract, arXiv:2607.11734).**
   Training is offline but the data cannot come from an archived position-only
   log: "a twin-arm teleoperation system records robot states and **actuator
   telemetry** alongside **external-force labels**, yielding the Neural Actuation
   Dataset". The torque-surrogate head avoids ground-truth torque labels (trained
   through differentiable simulation from pose trajectories), but the
   external-force perception head is supervised by NAD's force labels. Inference
   is real-time via a Transformer. So `online` should read "offline training on a
   purpose-collected labelled dataset; real-time inference" and `position-only
   logs` stays **no**. Row's verdict unchanged, its reason sharpened.

   Worth flagging beyond the table: NeuralActuator validates on "a 6-DoF SO-101
   from LeRobot" — the same platform as this paper. It is the closest
   platform-matched competitor and Sec. 2 should say so.

   Bib error fixed in the same pass: the entry read "Dou, Zhiyang and
   Onyemelukwe, Chidera and Zhang, Haoran and others". arXiv gives twelve authors
   and two of the three named given names were wrong — **John U.** Onyemelukwe and
   **Hangxing** Zhang. Full list now in `refs.bib`.
4. `wong2026beyond` — **CLOSED, 2026-09-04, and the row was badly understated.
   This is the closest work to this paper that exists, and it is on real
   hardware.** Abstract (arXiv:2607.14578), authors verified exactly:

   They identify the same channel we do — "many deployments collect
   demonstrations through leader-follower teleoperation, where **tracking error
   between commanded leader motion and executed follower motion implicitly
   encodes contact**, resistance, and constraint violation" — and ask whether
   ACT's force-awareness depends on it. Their intervention *removes* the cue (an
   observation-centric ACT variant predicting future follower joint states rather
   than leader commands), and "across four real-world tasks spanning surface
   following, insertion, stiffness discrimination, and force-based stopping,
   **removing the implicit cue leads to severe failures in force-critical
   phases**." Their remedy is torque proxies from onboard motor current or joint
   effort, which "recover robust contact behavior and improve the base ACT
   policy".

   **This cuts both ways and the paper must say both halves.**

   *Against novelty:* the core observability claim — that the commanded-versus-
   measured discrepancy is what makes ACT force-aware, and that removing it breaks
   force-critical manipulation — is already published, on real hardware, across
   four tasks. Our E−A and the zeroing counterfactual are the simulated,
   additive-direction analogue of their real, subtractive-direction result. The
   table's old characterisation ("no (action labels) | no | yes ⚠ | yes ⚠") does
   not convey any of this, and a reviewer who knows this paper will notice.

   *For the paper:* it is independent real-hardware corroboration of the claim our
   grid can only make in simulation, and it should be cited as such rather than
   only as a related method. And the distinction survives, narrowed and specific:
   their remedy needs **current or joint-effort telemetry**, which App. D shows the
   recording stack caps on the contact joint and which no corpus dataset records;
   ours needs two position fields every archived log already stores. They subtract
   the cue to prove it matters; we add it to make it usable retroactively.

   Sec. 2 currently gives them one line. On this evidence they warrant a
   positioning paragraph, and Sec. 1's contribution claim should be re-read
   against them before submission.
5. Second table, *latent resolved* / *belongs to* for `torne2025pasttoken` and
   `zeng2026revisiting` — read off their own framing of why context helps, not
   from an experiment that isolated the latent. Both are entries 13 and 16 of
   thread A; the cells are as verified as that thread is.
6. Bibliography keys — `zhao2023act`, `torne2025pasttoken` and (added
   2026-09-03) `zeng2026revisiting` all resolve in `paper/refs.bib`, but none
   of the three is cited from `main.tex` yet, so this table cannot be dropped
   in without also adding the prose that cites them.

## Placement

Reads best as a compact `tabular` closing `sec:related`, or in the appendix with
a one-sentence pointer from the related-work text if the CoRL page budget binds
(`docs/roadmap.md:238` notes the §4 figure count as the other budget pressure).
Not yet inserted into `main.tex` — this file is the draft for review.
