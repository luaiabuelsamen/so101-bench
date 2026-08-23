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

## Open cells (⚠)

1. `yen2019virtual` — "extends the idea to low-cost platforms" does not say
   which signals. Check whether it is current-based.
2. `hwang2018virtual` — retroactive is inferred from the inputs being positions,
   not stated. Confirm no calibration stage is required; if one is, the
   position-only cell weakens to "yes, after calibration" and the paper's
   distinction from prior art gets sharper, not weaker.
3. `dou2026neuralactuator` — learned per-actuator models may be trainable
   offline from logs; if so the `online` cell is wrong.
4. `wong2026beyond` — shown "by intervention", so their analysis is not
   obviously an archived-log method. This row is the one most likely to be
   mis-stated above.

## Placement

Reads best as a compact `tabular` closing `sec:related`, or in the appendix with
a one-sentence pointer from the related-work text if the CoRL page budget binds
(`docs/roadmap.md:238` notes the §4 figure count as the other budget pressure).
Not yet inserted into `main.tex` — this file is the draft for review.
