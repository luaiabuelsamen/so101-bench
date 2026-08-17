#!/bin/bash
# Task 2 grid runner: serial, idempotent (skips cells whose result json
# exists), relaunchable after interruption. Arm C (delta) is complete via
# the Gate-1 seeds. Waits for the standalone arm-A seed-0 run if present.
set -u
cd /home/jetson3/projects/so101-bench
PY=/home/jetson3/projects/clean_env/venv/bin/python
export MUJOCO_GL=egl

until [ -f results/grid_A_base_s0.json ] || ! pgrep -f "grid_A_base_s0" >/dev/null; do
  sleep 120
done

run_cell() {
  local letter=$1 arm=$2 seed=$3
  local json=results/grid_${letter}_${arm}_s${seed}.json
  [ -f "$json" ] && { echo "SKIP $letter/$arm s$seed (done)"; return; }
  echo "CELL $letter/$arm s$seed START $(date -u +%H:%M)"
  $PY scripts/train_act.py --arm "$arm" --root data/demos_v3 \
    --out "checkpoints/grid_${letter}_${arm}_s${seed}" --steps 12000 \
    --image-size 96 --seed "$seed" --eval-episodes 100 --eval-crush -1 \
    --json "$json" >/dev/null 2>&1
  if [ -f "$json" ]; then
    echo "CELL $letter/$arm s$seed DONE: $(grep -o '"success": [0-9]*' "$json" | head -1)"
  else
    echo "CELL $letter/$arm s$seed FAILED (no json)"
  fi
}

for seed in 0 1 2; do run_cell A base "$seed"; done
for seed in 0 1 2; do run_cell B base_hist "$seed"; done
for seed in 0 1 2; do run_cell D resid "$seed"; done
for seed in 0 1 2; do run_cell E excess "$seed"; done
for seed in 0 1 2; do run_cell F token "$seed"; done
echo "GRID QUEUE COMPLETE $(date -u)"
