"""Counterfactual: does the guarded policy lift if it FEELS the demo grip?

Guard v3 physically holds the jaw at seat-4 (~40 N). From the latch step
onward, the policy's delta input on the jaw dim is overwritten with the
demo-typical committed value (-32 counts, median over 51,765 grip frames of
demos_v4). Everything else identical to the guarded eval (applied-command
feedback, seed 3000, 30 episodes, crush-off tier).

Hypothesis (user's step 1): the 0/30 collapse is the policy waiting for a
force signature the guard forbids. Confirmed if picks/places return.
"""
import sys, json
import numpy as np
import torch

sys.path.insert(0, "/home/jetson3/projects/so101-bench/scripts")
from eval_guard import JawGuard, load_policy, wilson  # noqa: E402
from so101_bench import DemoEnv, ExpertConfig  # noqa: E402
from so101_bench.scene import JAW_SHUT, RAD_PER_TICK  # noqa: E402

DEV = "cuda"
D_STAR = -32.0            # demo-median committed jaw delta, counts
EPISODES = 30

policy, st = load_policy("delta", "/home/jetson3/projects/so101-bench/modal_outputs/checkpoints")
policy.eval()

env = DemoEnv(seed=3000, cameras=("front", "gripper_fpv"), image_size=224,
              stride=2, crush_newtons=None, expert=ExpertConfig())
scene = env.scene

results = []
for ep in range(EPISODES):
    scene.reset()
    policy.reset()
    guard = JawGuard()
    z0 = scene.block_pos()[2]
    a_prev = None
    peak_rise, peak_force, latch_step = 0.0, 0.0, None
    for step in range(260):
        s = torch.as_tensor(scene.state_ticks(), dtype=torch.float32)
        s_n = (s - torch.tensor(st["s_mean"])) / torch.tensor(st["s_std"])
        d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
        if guard.seat_jaw is not None:
            d[5] = D_STAR                      # the counterfactual feeling
        d_n = (d - torch.tensor(st["d_mean"])) / torch.tensor(st["d_std"])
        obs = {"observation.state": torch.cat([s_n, d_n]).unsqueeze(0).to(DEV),
               "observation.images.front": torch.as_tensor(scene.image("front"))
               .permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEV),
               "observation.images.gripper_fpv": torch.as_tensor(
                   scene.image("gripper_fpv"))
               .permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEV)}
        with torch.no_grad():
            a_n = policy.select_action(obs).squeeze(0).cpu()
        action = a_n * torch.tensor(st["a_std"]) + torch.tensor(st["a_mean"])
        cmd = action.numpy().astype(float)
        cmd[5] = guard(float(scene.state_ticks()[5]), cmd[5])
        cmd[5] = max(cmd[5], JAW_SHUT / RAD_PER_TICK)
        if latch_step is None and guard.seat_jaw is not None:
            latch_step = step
        a_prev = torch.tensor(cmd, dtype=torch.float32)
        scene.hold(cmd * RAD_PER_TICK, frames=2)
        peak_rise = max(peak_rise, scene.block_pos()[2] - z0)
        peak_force = max(peak_force, scene.grip_force())
    picked = peak_rise > 0.03
    placed = picked and scene.block_in_box() and not scene.crushed
    results.append(dict(picked=bool(picked), placed=bool(placed),
                        peak_force=float(peak_force),
                        latch=latch_step, rise=float(peak_rise)))
    print(f"ep{ep}: latch@{latch_step} rise={peak_rise*1000:.0f}mm "
          f"peakF={peak_force:.0f}N picked={picked} placed={placed}", flush=True)

pk = sum(r["picked"] for r in results)
pl = sum(r["placed"] for r in results)
lo, hi = wilson(pl, EPISODES)
pf = float(np.median([r["peak_force"] for r in results]))
print(f"\nCOUNTERFACTUAL: picked {pk}/{EPISODES}, placed {pl}/{EPISODES} "
      f"[{lo:.2f},{hi:.2f}], peakF median {pf:.0f}N")
print("baseline guarded (no counterfactual): placed 0/30, picked ~2-5/30")
json.dump(results, open("/home/jetson3/projects/so101-bench/results/counterfactual_commit.json", "w"),
          indent=2, default=float)
env.close()
