# Task 2 grid — arm definitions (written before implementation, 2026-08-17)

Fixed for all arms: full demos_v3 (280 eps; the instruction's "200" miscounted
the set — 280 is the reproduced-baseline config), 12k steps, 96 px, batch 16,
seeds {0,1,2}, eval n=100 at crush −1 (tiers 120/60 on the per-arm best seed
only, to bound compute), historical harness. Resolution note from Gate 1:
sigma_seed ≈ 10 points at n=100 — differences under ~15 points are below
this grid's resolution and will not be claimed.

- **A base** — s[t] (6). Exists in ladder history; rerun under grid protocol.
- **B base_hist** — [s[t], a[t−1]] (12). The copycat-bait control.
- **C delta_input** — [s[t], a[t−1]−s[t]] (12). DONE: {11, 25, 31}/100.
- **D residual_head** — input s[t] (6); auxiliary head on the ACT encoder
  latent predicts delta[t] (6), loss += 0.1·MSE (normalized). No delta at
  input; the channel enters only as a representation-shaping target. Tests
  whether the *information* must be an input or merely a training signal.
- **E load_prior** — input [s[t], excess[t]] (12) where excess = (a[t−1] −
  s[t]) − K̂·rate, the per-joint tracking error minus the free-motion lag
  baseline (K̂ fit per joint by least squares on the no-contact frames of
  the same training set — free-motion self-labeling, the FACTR-2 steal;
  frozen before training). The channel arrives pre-separated into
  "load-like" vs "motion-like", testing whether the confound the
  characterization measured (motion lag) is what makes raw delta hard to
  use at small data.
- **F event_token** — input [s[t], seat[t]·𝟙₆] (12) where seat[t] ∈ {0,1}
  is the frozen stall/creep detector (guard v4's detection logic, no cap)
  run causally on (a[t−1], s[t]) jaw history. Commitment becomes an
  observable event, not an inferred force magnitude — the discrete version
  of what the scripted expert already keys on.

Trained-through-guard column: same arms on demos_v3g extended to 280
episodes (80 more to collect), evaluated raw AND guarded; the fix-2
question lives here at whatever resolution the grid affords.

Predictions (registered): D/E/F ≥ B at this budget if the channel's value
is information (E sharpest if the motion confound is the small-data
blocker); B ≈ C from the H100 result; guard row converts C/E/F's crush
tiers without deleting >15 points of success. If A≈B≈C≈D≈E≈F, the design
claim dies and the paper is observability + guard, as the instruction says.

## Compute policy (binding)

Seeds are NEVER reduced below 3 to fit compute; if the box is the
constraint, the grid takes longer, and if more seeds are needed for a
claim, the cell reruns. Jetson serial = ~3 h/cell, 36 cells ~= 4.5 days.
H100 alternative: ~12-15 min/cell train (eval stays local, free), ~9-10
GPU-hours ~= $40-50 for the full grid at 5 seeds -- exceeds the ~$4 Modal
credits remaining; a top-up is the user's call. Until then: Jetson serial,
3 seeds, arms in order A, B, D, E, F, then the guarded column.

---

## Addendum 2026-09-03 — the compute-cost figures above are stale, by ~6x

*Appended, not edited: the text above is frozen under tag `prereg-grid`
(d4077d5) and stays as written. This section corrects it for anyone planning
a run.*

`docs/task2_arms.md:46` estimates "~12-15 min/cell train" on H100 and
"$40-50 for the full grid at 5 seeds". Both were written before the launcher
spec was frozen and are superseded by `scripts/grid_spec.json`, which the
launcher reads exclusively: **`est_cell_minutes` 35, `est_cell_usd` 2.60**.
Measured actual, `findings.md:939`: dry + main = 9 H100 cells ≈ $27, i.e.
**$3.00/cell** at the grid operating point (280 demos, 96 px, 12k steps,
batch 16). Trust `grid_spec.json` and the measured number, not line 46.

**Why this matters beyond the dollar figure — the H100 ratio does not
transfer to the Orin.** A scale cell (600 demos, 224 px, 50k steps, batch 64)
is ~91x the arithmetic of a grid cell: steps 50k/12k = 4.17, batch 64/16 =
4.0, pixels (224/96)² = 5.44. The scale run cost $26 for 4 containers =
**$6.50/cell**, only ~2.2x a grid cell's $3.00. Ninety-one times the FLOPs for
2.2x the wall clock means the grid cell **does not load an H100** — batch 16 at
96 px is overhead-bound, and most of those 35 minutes is not compute.

The Orin is compute-bound at every one of these sizes, so the 91x applies there
and the 2.2x does not. Measured on the Orin this session: ACT at 0.3 s/step,
batch 4, 96 px → a scale cell extrapolates to ~15 days (linear, therefore an
upper bound; batch-4 per-step overhead makes real batch-64 scaling sublinear,
plausibly 7-10 days). **Six scale cells is 6-12 weeks serial on the Orin, not
the ~10-12 days a naive H100-ratio extrapolation gives.** Unchecked: whether
batch 64 at 224 px with a ResNet18 backbone fits in unified Orin memory at all;
a forced batch reduction would change the cell's meaning, not just its runtime.

**Consequence for the compute policy above.** "If the box is the constraint,
the grid takes longer" holds for grid-operating-point cells (~3 h each) and
does *not* hold for scale cells. The scale question is a funding question:
A, E and G at 600/224/50k, 2 seeds each, reusing the existing `base_hist`
cells as the control, is **6 cells ≈ $39**.

Provenance: derived jointly with session `projects-a5`, which caught the error;
the 6x mis-estimate came from propagating line 46 rather than `grid_spec.json`.

---

## Arms B2 and G — definitions frozen before implementation (2026-09-03)

*Same discipline as the original grid (rule 5): written before the cells run,
appended rather than edited into the frozen prereg above. Protocol inherited
unchanged — demos_v3 (280 eps), 12k steps, 96 px, batch 16, seeds {0,1,2},
eval n=100 at crush −1, historical harness, sigma_seed ≈ 10 so **only
differences ≥ 15 points are claimable at 3 seeds**.*

**Why these two arms exist.** `findings.md` (2026-09-03) established that arm E
consumes `a[t−2]` via the command rate while B and C do not, so the declared
primary E−B = +12.8 (t=2.61) confounds the free-motion subtraction with an extra
frame of action history, and decomposes exactly into E−C = +4.8 (t=0.86, the
extra frame) and C−B = +8.0 (t=1.44, representation at matched information).
Neither half is separately resolved. B2 and G are the two controls the grid lacks.

- **B2 `base_hist2`** — input `[s[t], a[t−1], a[t−2]]` (18). The same raw
  information as E, presented raw. `E − B2` is the value of the fitted
  free-motion subtraction with information held fixed — the sentence the title
  rests on.
- **G `ghist`** — input `[s[t], s[t−8]]` (12). Position context only; no
  `a[t−1]` in any form, so the tracking error is not computable from it. Tests
  whether state history substitutes for the channel.

### Registered predictions and decision rules

**B2.** Pre-commit to the decision rule, because at 3 seeds this cell will
probably not resolve and we should not discover that after seeing the number:

- Claim "the subtraction is the contribution" **only if E − B2 ≥ 15**.
- Claim "the extra frame is the contribution" **only if B2 − B ≥ 15**.
- If neither clears 15, the cell is **non-resolving**, and the paper discloses
  the confound (decomposition table, `findings.md` 2026-09-03) without
  attributing it. That is the expected outcome on the existing numbers — E−B is
  only +12.8 in total — and it is still worth the Orin night, because a B2 that
  lands near E is directional evidence the headline is the extra frame, which
  changes what gets written at 600.

**Width caveat, registered before the result.** B2 is 18-dim; B, C and E are all
12-dim. B2 is information-matched to E but **not** width-matched, and it is the
only 18-dim arm in the grid. At 280 demonstrations extra width can overfit or
can help. So the readings are asymmetric and must be reported as such:
**E ≥ B2 is strong** evidence for the subtraction; **E < B2 is ambiguous**
between "raw access beats the fitted form" and "width helped". Do not report the
second branch as a clean negative.

**G.** Both live accounts predict G loses at 280, so **this cell does not
discriminate** — it is a slope anchor, not a test, and its whole value is making
G at 600 interpretable (`zeng2026revisiting` Fig. 8: long-context arms lose at
200 demos and win at 1000, so a budget-dependent quantity measured at one budget
cannot be read). Registered: G < C at 280, and G − A ≥ 0. If G ≥ C at 280 both
accounts are in trouble and that is the interesting failure.

**Scope limit, registered.** As defined, G is **one** past frame at lag 8. That
is width-matched to E, which is why it is specified this way, but it is *not*
`zeng2026revisiting`'s control — their effect requires T_o = 12–20 frames and is
absent at their T_o = 2. G therefore **bounds** the position-history explanation;
it does not settle it. A faithful long-context arm (`ghist_long`, T_o = 8,
48-dim, not width-matched to anything) is deferred to the 600-demo budget, where
Fig. 8 says a long-context arm should start winning. The paper must not describe
G as settling the Zeng alternative.

### Cost

Both are grid-operating-point cells: ~3 h each on the Orin (free, and the Orin is
compute-viable at this size — see the cost addendum above; it is *not* viable for
600/224/50k cells), or ~$2.60 each on H100 per `grid_spec.json`. Run B2 first: it
attacks the declared primary, G attacks a general alternative that both accounts
already agree on at this budget.
