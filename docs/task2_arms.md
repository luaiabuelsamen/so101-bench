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
