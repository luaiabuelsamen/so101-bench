#!/bin/bash
# Task 2 GUARDED-COLUMN runner: same cells as run_grid.sh but trained on
# demos_v3g280 (collected through guard v4). Serial, idempotent, relaunchable.
# Result names carry a "g" suffix on the letter: grid_Ag_base_s0.json etc.
set -u
cd /home/jetson3/projects/so101-bench
PY=/home/jetson3/projects/clean_env/venv/bin/python
export MUJOCO_GL=egl

run_cell() {
  local letter=$1 arm=$2 seed=$3
  local json=results/grid_${letter}g_${arm}_s${seed}.json
  [ -f "$json" ] && { echo "SKIP ${letter}g/$arm s$seed (done)"; return; }
  echo "CELL ${letter}g/$arm s$seed START $(date -u +%H:%M)"
  $PY scripts/train_act.py --arm "$arm" --root data/demos_v3g280 \
    --out "checkpoints/grid_${letter}g_${arm}_s${seed}" --steps 12000 \
    --image-size 96 --seed "$seed" --eval-episodes 100 --eval-crush -1 \
    --json "$json" >/dev/null 2>&1
  if [ -f "$json" ]; then
    echo "CELL ${letter}g/$arm s$seed DONE: $(grep -o '"success": [0-9]*' "$json" | head -1)"
  else
    echo "CELL ${letter}g/$arm s$seed FAILED (no json)"
  fi
}

for seed in 0 1 2; do run_cell C delta "$seed"; done
for seed in 0 1 2; do run_cell A base "$seed"; done
for seed in 0 1 2; do run_cell B base_hist "$seed"; done
for seed in 0 1 2; do run_cell D resid "$seed"; done
for seed in 0 1 2; do run_cell E excess "$seed"; done
for seed in 0 1 2; do run_cell F token "$seed"; done
echo "GUARDED COLUMN COMPLETE $(date -u)"
