"""Task 2 grid cells on Modal H100 -- spec-driven, no value overrides.

    modal run scripts/modal_grid.py --phase dry      # one real cell, check the bill
    modal run scripts/modal_grid.py --phase main     # D/E/F remaining seeds
    modal run scripts/modal_grid.py --phase seeds45  # +2 seeds on all six arms

Rails (docs: the phase-3 Modal rails):
- every training value comes from scripts/grid_spec.json, committed; the
  only CLI argument is the phase NAME;
- each cell checks the volume for its result json and exits immediately if
  present (idempotent, resumable, crash never reruns paid work);
- per-function timeout from the spec (~2x expected cell time);
- `modal run` is always typed by a human; nothing here self-launches.

Results land on the volume: outputs/grid/grid_{ARM}_s{SEED}.json plus
checkpoints. Download:  modal volume get so101-bench outputs/grid ./results/modal_grid
"""

from __future__ import annotations

import json
import pathlib

import modal

SPEC = json.loads(
    (pathlib.Path(__file__).parent / "grid_spec.json").read_text()
)
T = SPEC["train"]

app = modal.App("so101-grid")
vol = modal.Volume.from_name("so101-bench", create_if_missing=True)

image = (
    modal.Image.debian_slim(python_version="3.10")
    .apt_install("libgl1", "libglib2.0-0", "libegl1", "libgles2")
    .pip_install("torch", "mujoco>=3.0", "numpy", "pandas", "pyarrow")
    .env({"MUJOCO_GL": "egl", "PYTHONPATH": "/repo/src"})
    .add_local_dir("vendor/lerobot", "/lerobot_src", copy=True)
    .run_commands("pip install '/lerobot_src[dataset]'")
    .add_local_dir("src", "/repo/src", copy=True)
    .add_local_dir("scripts", "/repo/scripts", copy=True)
)


@app.function(image=image, gpu=T["gpu"], volumes={"/vol": vol},
              timeout=T["timeout_s"])
def train_cell(arm: str, seed: int) -> str:
    import os
    import subprocess
    import sys

    letter = {"base": "A", "base_hist": "B", "delta": "C",
              "resid": "D", "excess": "E", "token": "F"}[arm]
    tag = f"grid_{letter}_{arm}_s{seed}"
    out_json = f"/vol/outputs/grid/{tag}.json"
    if os.path.exists(out_json):
        return f"SKIP {tag} (done on volume)"
    os.makedirs("/vol/outputs/grid", exist_ok=True)

    cmd = [
        sys.executable, "/repo/scripts/train_act.py",
        "--arm", arm,
        "--seed", str(seed),
        "--steps", str(T["steps"]),
        "--batch", str(T["batch"]),
        "--image-size", str(T["image_size"]),
        "--root", T["root"],
        "--out", f"/vol/outputs/grid/ckpt_{tag}",
        "--eval-episodes", str(T["eval_episodes"]),
        "--eval-crush", *[str(c) for c in T["eval_crush"]],
        "--json", out_json,
    ]
    proc = subprocess.run(cmd, cwd="/repo", capture_output=True, text=True)
    vol.commit()
    if proc.returncode != 0 or not os.path.exists(out_json):
        raise RuntimeError(
            f"{tag} failed:\n{proc.stdout[-1500:]}\n{proc.stderr[-1000:]}"
        )
    result = json.loads(open(out_json).read())
    return f"DONE {tag}: {result}"


@app.local_entrypoint()
def main(phase: str):
    cells = SPEC["phases"][phase]          # KeyError on unknown phase = good
    est = len(cells) * T["est_cell_usd"]
    print(f"phase '{phase}': {len(cells)} cells, est ~${est:.0f} "
          f"(~{T['est_cell_minutes']} min/cell)")
    for res in train_cell.starmap([(a, s) for a, s in cells]):
        print(res, flush=True)
