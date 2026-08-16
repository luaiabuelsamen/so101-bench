"""Multi-seed ACT training on Modal GPUs.

One-time dataset upload (550 MB), from the repo root:

    modal volume create so101-bench
    modal volume put so101-bench data/demos_v4 demos_v4

Then the full grid (3 arms x 5 seeds, parallel containers):

    modal run scripts/modal_train.py

or a subset:

    modal run scripts/modal_train.py --arms delta --seeds 0,1,2

Each container trains one (arm, seed) with the exact same `train_act.py` used
locally -- same data path, same normalisation, same closed-loop MuJoCo eval
(EGL headless works in Modal's GPU containers) -- and writes checkpoints,
loss CSVs and eval JSONs back to the volume:

    modal volume get so101-bench outputs ./modal_outputs

Cost note: ~8k steps is minutes on an H100; the closed-loop eval (CPU MuJoCo)
dominates at ~15 min per run. 15 runs ~ 4-5 container-hours total. Switch
`GPU` to "A10G" to halve the price at ~2x the training time; eval time is
unchanged.

NOTHING in this file runs automatically -- `modal run` is always an explicit,
billed action.
"""

from __future__ import annotations

import modal

GPU = "H100"
STEPS = 50000
IMAGE = 224

app = modal.App("so101-bench-act")
vol = modal.Volume.from_name("so101-bench", create_if_missing=True)

image = (
    modal.Image.debian_slim(python_version="3.10")
    .apt_install("libgl1", "libglib2.0-0", "libegl1", "libgles2")
    .pip_install("torch", "mujoco>=3.0", "numpy", "pandas", "pyarrow", "wandb")
    .env({"MUJOCO_GL": "egl", "PYTHONPATH": "/repo/src"})
    .add_local_dir("vendor/lerobot", "/lerobot_src", copy=True)
    .run_commands("pip install '/lerobot_src[dataset]'")
    .add_local_dir("src", "/repo/src", copy=True)
    .add_local_dir("scripts", "/repo/scripts", copy=True)
    .add_local_file("pyproject.toml", "/repo/pyproject.toml", copy=True)
)


@app.function(image=image, gpu=GPU, volumes={"/vol": vol}, timeout=6 * 3600,
              secrets=[modal.Secret.from_name("wandb-secret")])
def train_one(arm: str, seed: int, steps: int = STEPS, eval_eps: int = 100) -> str:
    import subprocess
    import sys

    cmd = [
        sys.executable, "/repo/scripts/train_act.py",
        "--arm", arm,
        "--seed", str(seed),
        "--steps", str(steps),
        "--batch", "64",                       # H100 headroom; lr unchanged
        "--image-size", str(IMAGE),
        "--wandb",
        "--root", "/vol/demos_v4",
        "--out", "/vol/outputs/checkpoints",
        "--eval-episodes", str(eval_eps),
        "--eval-crush", "-1", "120",
        "--json", f"/vol/outputs/act_{arm}_s{seed}.json",
    ]
    proc = subprocess.run(cmd, cwd="/repo", capture_output=True, text=True)
    vol.commit()
    tail = proc.stdout[-2000:] + proc.stderr[-1000:]
    if proc.returncode != 0:
        raise RuntimeError(f"{arm} s{seed} failed:\n{tail}")
    return f"{arm} s{seed} done\n{tail}"


@app.local_entrypoint()
def main(arms: str = "delta,base,base_hist", seeds: str = "0,1",
         steps: int = STEPS, eval_eps: int = 100):
    jobs = [(a, int(s), steps, eval_eps) for a in arms.split(",") for s in seeds.split(",")]
    print(f"launching {len(jobs)} runs on {GPU}: {jobs}")
    for result in train_one.starmap(jobs, return_exceptions=True):
        print("=" * 60)
        print(result if not isinstance(result, Exception) else f"FAILED: {result}")
