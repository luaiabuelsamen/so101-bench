"""Offline validation of the lerobot-record CLI args that scripts/arms.sh passes.

Parses the real RecordConfig via draccus with the real argv, but never opens a
serial port or a camera. Catches unknown-arg / missing-required-field errors
without needing the robot plugged in.
"""

import sys
import traceback

FOLLOWER = "/dev/ttyACM0"
LEADER = "/dev/ttyACM1"
FID = "my_awesome_follower_arm"
LID = "my_awesome_leader_arm"
CAM = "/dev/video0"
BENCH = "/home/jetson3/projects/so101-bench"
NAME = "pickplace_real_v0"
N = "5"

ARGV = [
    "--robot.type=so101_follower",
    f"--robot.port={FOLLOWER}",
    f"--robot.id={FID}",
    f'--robot.cameras={{"front": {{"type": "opencv", "index_or_path": "{CAM}", '
    f'"width": 640, "height": 480, "fps": 30}}}}',
    "--teleop.type=so101_leader",
    f"--teleop.port={LEADER}",
    f"--teleop.id={LID}",
    "--dataset.fps=30",
    f"--dataset.repo_id=luaia/{NAME}",
    f"--dataset.root={BENCH}/data/real/{NAME}",
    "--dataset.single_task=pick up the object and place it in the target zone",
    f"--dataset.num_episodes={N}",
    "--dataset.episode_time_s=25",
    "--dataset.reset_time_s=8",
    "--dataset.push_to_hub=false",
]

print("[1/3] importing lerobot_record (validates the import chain)...", flush=True)
try:
    import importlib.util as u

    spec = u.spec_from_file_location(
        "lerobot_record_mod",
        "/home/jetson3/projects/clean_env/lerobot/src/lerobot/scripts/lerobot_record.py",
    )
    mod = u.module_from_spec(spec)
    sys.modules["lerobot_record_mod"] = mod
    spec.loader.exec_module(mod)
except Exception:
    print("IMPORT FAILED:", flush=True)
    traceback.print_exc()
    sys.exit(1)
print("      import OK", flush=True)

print("[2/3] parsing argv into RecordConfig via draccus...", flush=True)
try:
    import draccus

    cfg = draccus.parse(config_class=mod.RecordConfig, args=ARGV)
except SystemExit as e:
    print(f"PARSE FAILED (argparse/draccus exited {e.code})", flush=True)
    sys.exit(1)
except Exception:
    print("PARSE FAILED:", flush=True)
    traceback.print_exc()
    sys.exit(1)
print("      parse OK", flush=True)

print("[3/3] resolved config:", flush=True)
d = cfg.dataset
print(f"      repo_id        = {d.repo_id}")
print(f"      root           = {d.root}")
print(f"      single_task    = {d.single_task!r}")
print(f"      num_episodes   = {d.num_episodes}")
print(f"      episode_time_s = {d.episode_time_s}")
print(f"      reset_time_s   = {d.reset_time_s}")
print(f"      fps            = {d.fps}")
print(f"      push_to_hub    = {d.push_to_hub}")
print(f"      video          = {d.video}")
print(f"      robot.port     = {cfg.robot.port}")
print(f"      robot.cameras  = {cfg.robot.cameras}")
print(f"      teleop.port    = {cfg.teleop.port}")
print("\nARGS-VALID: the record command will not die on argument errors.")
