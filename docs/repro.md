# Reproducibility manifest (Lag Excess, draft v0.4)

Everything a reviewer needs to reproduce or audit each claim, by artifact.
For double-blind submission this repo is anonymized to an archive; the two
pre-registration freezes are verifiable from commit timestamps preserved in
the archive's git history.

## Pre-registration freezes (verifiable)

| tag | commit | timestamp (UTC−7) | freezes |
|---|---|---|---|
| `prereg-corpus` | 7b7a818 | 2026-08-16 18:38 | corpus protocol: close/seat detectors, δ-at-seat, success proxy, prediction (docs/phase3.md) |
| `prereg-grid` | d4077d5 | 2026-08-17 07:54 | six arm definitions, budget, seeds, resolution rule, predictions (docs/task2_arms.md) |

Both tags predate the corresponding runs recorded in `docs/findings.md`.

## Simulator and task

- Plant + contact model + task: `src/so101_bench/scene.py` (MJCF under
  `src/so101_bench/assets/`), success window and crush latch in
  `scene.update_crush`; domain randomization ranges in
  `DomainRandomization`.
- Scripted teacher (force-mode; the demo distribution): `src/so101_bench/expert.py`.
- Demonstration collection: `scripts/collect.py` (stock),
  `scripts/collect_guarded.py` (through-guard twin; kicks p=0.02 σ=1.5,
  grip target ~ U[12,45] counts).

## The six observation transforms

All in `scripts/train_act.py::FastChunkDataset.__getitem__` (train side)
and `train_act.py::evaluate` (matching eval side): `base`, `hist` (B),
delta (C), `resid` (D, aux head in `build_policy.ResidualACT`), `excess`
(E; K̂ fit in `FastChunkDataset.__init__`, saved as `k_hat` in each
checkpoint's `norm_stats.npz`), `token` (F; detector = `so101_bench/guard.py`
with the cap and rate limit disabled).

## Grid results, per seed

- Main column: `results/grid_{A..F}_*_s{0..4}.json` (n=100 episodes each;
  eval seed 1000, protocol length 260 steps).
- Guarded column: `results/grid_{A..F}g_*_s{0..2}.json` (in progress).
- Statistics (Welch t, bootstrap CIs): recomputable from the JSONs;
  sweep/figure scripts below.

## Corpus study

- Script (frozen protocol implementation): `scripts/corpus_delta_outcome.py`.
- Inclusion/exclusion ledger: `results/corpus_delta_outcome.json` records
  every scored dataset; exclusion gates (≥20 episodes with a close event,
  median one close per episode, both proxy classes ≥5, quantum/range
  gates) are in the script constants; candidate pool = the 555-repo list
  (`--quanta-jsonl`); shortfall to n=16 documented in findings.
- Alignment robustness: `scripts/offset_sweep.py` →
  `results/offset_sweep.json` (k=0..5; median d-span 0.12; k* ≈ 3–4).

## Force model, guard, calibration

- Force-from-δ characterization suite: `scripts/characterize_delta.py`,
  `scripts/phase1b_validity.py`, `scripts/phase1c_boundaries.py`,
  `scripts/resolution_sweep.py` (the cliff).
- Guard (v4) and its paired-reference test: `src/so101_bench/guard.py`,
  `scripts/eval_expert_guarded.py`, `results/expert_guarded_pairing.json`,
  `results/guard_h100*.json`.
- Hardware staircase protocol (Sec. 4.5, pending): `hardware/staircase_cal.py`.

## Figures

`scripts/fig_teaser.py`, `scripts/fig_mechanism.py`, `scripts/fig_grid.py`,
figure block in `scripts/corpus_delta_outcome.py`; rendered copies under
`figures/paper/`.

## Environment

Python 3.10; lerobot editable tree (self-versioned 0.5.2) with ACT;
MuJoCo ≥3.0 EGL; exact training hyperparameters in
`scripts/grid_spec.json` (frozen; the Modal launcher reads only this
file). Training hardware: Jetson AGX Orin (local cells) and H100 (Modal;
`scripts/modal_grid.py`), bit-nonreproducible across GPUs — hence
seed-level statistics rather than run-level.

## Checkpoint availability (2026-08-23)

Checkpoints are not git-tracked and were never pushed to the Hub, so the copies
under `checkpoints/` are the only ones. A disk-pressure cleanup on 2026-08-22
kept seven and deleted the rest, including `checkpoints/modal_outputs/`.

Present: `grid_A_base_s1`, `grid_B_base_hist_s1`, `act_v3full_delta_s1`,
`modal_E_s0`, `grid_O_oracle_s{0,1,2}`.

One consequence to know about before trusting a `make figures`-style rerun:

- **`grid_A_base_s0` is gone**, and `scripts/fig_teaser.py:115` is the only
  consumer. Fig. 1 therefore cannot be re-rendered as-is; the committed
  `figures/paper/fig_teaser.png` is intact and the paper builds. Recovering it
  means retraining that one cell (`scripts/run_grid.sh`, the base arm, the
  cheapest in the grid) and accepting a non-identical checkpoint -- training is
  bit-nonreproducible across GPUs, so the re-rendered episode will differ even
  at the same seed. The seed-level numbers are unaffected:
  `results/grid_A_base_s0.json` was never touched.

Every other figure script resolves: `fig_mechanism.py` needs only
`modal_E_s0/excess/norm_stats.npz`, and `fig_grid.py` / `fig_ksweep.py` /
`corpus_delta_outcome.py` read `results/*.json` rather than weights.

`scripts/fig_teaser.py` carries a "simulation" stamp as of this commit, matching
`fig_mechanism.py` and `fig_grid.py`. It will appear the next time the figure is
regenerated, not in the committed PNG. `fig_ksweep.py` is deliberately
unstamped: it plots the public teleoperation corpus, which is real-robot data.
