#!/bin/bash
# One entry point for the physical SO-101 pair.
#
#   scripts/arms.sh ports              # list serial ports + which USB device
#   scripts/arms.sh calibrate-leader   # one-time leader calibration (interactive)
#   scripts/arms.sh calibrate-follower # redo follower calibration (interactive)
#   scripts/arms.sh teleop             # leader-follower teleoperation loop
#   scripts/arms.sh teleop-cam         # teleop + front camera preview
#   scripts/arms.sh record NAME N      # record N teleop episodes -> data/real/NAME
#   scripts/arms.sh staircase          # bench calibration (delta/load/current vs load cell)
#
# Convention (from the original working setup): FOLLOWER=/dev/ttyACM0,
# LEADER=/dev/ttyACM1. If the wrong arm moves in teleop, the USB enumeration
# swapped -- override with:  FOLLOWER=/dev/ttyACM1 LEADER=/dev/ttyACM0 scripts/arms.sh teleop
set -euo pipefail

FOLLOWER="${FOLLOWER:-/dev/ttyACM0}"
LEADER="${LEADER:-/dev/ttyACM1}"
FID="${FID:-my_awesome_follower_arm}"
LID="${LID:-my_awesome_leader_arm}"
CAM="${CAM:-/dev/video0}"
LEROBOT=/home/jetson3/projects/clean_env/lerobot
PY=/home/jetson3/projects/clean_env/venv/bin/python
BENCH=/home/jetson3/projects/so101-bench

cd "$LEROBOT"

case "${1:-help}" in
  ports)
    found=$(ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
    [ -n "$found" ] && echo "$found" || echo "no serial devices"
    ;;
  calibrate-leader)
    exec $PY src/lerobot/scripts/lerobot_calibrate.py \
      --teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id="$LID"
    ;;
  calibrate-follower)
    exec $PY src/lerobot/scripts/lerobot_calibrate.py \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id="$FID"
    ;;
  teleop)
    exec $PY src/lerobot/scripts/lerobot_teleoperate.py \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id="$FID" \
      --teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id="$LID"
    ;;
  teleop-cam)
    exec $PY src/lerobot/scripts/lerobot_teleoperate.py \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id="$FID" \
      --robot.cameras="{\"front\": {\"type\": \"opencv\", \"index_or_path\": \"$CAM\", \"width\": 640, \"height\": 480, \"fps\": 30}}" \
      --teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id="$LID" \
      --display_data=true
    ;;
  record)
    NAME="${2:?usage: arms.sh record NAME N_EPISODES}"
    N="${3:?usage: arms.sh record NAME N_EPISODES}"
    exec $PY -m lerobot.scripts.lerobot_record \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id="$FID" \
      --robot.cameras="{\"front\": {\"type\": \"opencv\", \"index_or_path\": \"$CAM\", \"width\": 640, \"height\": 480, \"fps\": 30}}" \
      --teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id="$LID" \
      --dataset.fps=30 \
      --dataset.repo_id="luaia/$NAME" \
      --dataset.root="$BENCH/data/real/$NAME" \
      --dataset.num_episodes="$N" \
      --dataset.push_to_hub=false
    ;;
  staircase)
    shift
    cd "$BENCH"
    exec $PY hardware/staircase_cal.py --port "$FOLLOWER" "$@"
    ;;
  *)
    sed -n '2,14p' "$0"
    ;;
esac
