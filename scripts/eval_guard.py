"""Guard-wrapper A/B: the same trained policy, raw vs wrapped, paired seeds.

    python scripts/eval_guard.py --arms base delta delta_q16

THE CLAIM UNDER TEST. The demonstrations carry the teacher's force profile
(peak grip median 77 N), so a cloned policy should crush at tight thresholds
no matter what its inputs are. The guard is ~30 lines of jaw-safety around ANY
policy's actions, using only bus-observable signals and the constants
validated in Phases 1b/1c:

  * stall-based seating detection on the observed jaw channel (the frozen
    detector: recall 1.00, grasp-ready precision 0.78 adversarially);
  * once seated, the jaw command is floored at (seat - GUARD_COUNTS): the
    policy may open or hold, but cannot command deeper than a bounded squeeze.

No retraining, no object knowledge, no privileged state. If it converts
crushes to successes on the identical checkpoint, that is the deployable
result: wrap your existing SO-101 policy, stop crushing things.

Same paired protocol as everything else: identical eval seeds raw vs guarded,
Wilson CIs, per-tier crush/drop/success plus true-force statistics.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).parent))

from so101_bench import DemoEnv, ExpertConfig  # noqa: E402
from so101_bench.guard import JawGuard  # noqa: E402
from so101_bench.scene import JAW_SHUT, RAD_PER_TICK  # noqa: E402

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
GUARD_COUNTS = 4.0          # deepest allowed squeeze past detected seat
STALL_FRACTION = 0.3
STALL_WINDOW = 4            # frames of jaw history at the policy's 15 Hz rate
#: The servo tracks at most ~6.5 counts per 30 Hz control frame (measured:
#: 0.0100 rad/frame free-running). A stall test that compares measured motion
#: against COMMANDED travel false-positives whenever a policy commands faster
#: than the servo can slew -- the scaled ACT policy does exactly that, and the
#: unclamped guard latched phantom seats at open aperture (0/50, 0 N grip).
#: Feasible travel is the correct denominator.
MAX_SLEW_PER_FRAME = 13.0   # counts per policy step (2 control frames)
#: Seating means the command NET-deepened across the whole window while the
#: jaw failed to follow. Both phantom-seat modes seen on the scaled policy --
#: (a) big single-step extrapolation, (b) sub-count closing jitter around a
#: persistent equilibrium offset, where the deadband keeps the jaw static --
#: cancel out in net terms: jitter nets to ~0 and fails NET_CLOSE_MIN; a real
#: squeeze nets tens of counts the jaw doesn't track.
NET_CLOSE_MIN = 8.0         # counts of net command deepening over the window
GAP_MIN = 5.0               # meas-cmd gap that must be open at test time
#: The H100 policy also CREEPS its close (~4 counts/step, cloned from the
#: teacher's guarded move), and a creeping jaw tracks the command INTO the
#: object -- no kinematic stall until deep in the squeeze. Contact shows up
#: instead as gap in EXCESS of the rate-proportional following lag
#: (gap ~= K_LAG * rate when free). K_LAG measured on 609 free-closing
#: windows of this policy's traces (median 2.32, p90 2.97 steps); re-estimate
#: from free motion when porting to hardware. Detector validated offline on
#: 12 instrumented episodes: persist=3 gives 0 phantoms / 0 misses,
#: force-at-seat median 37 N max 49 N vs raw peaks 90-156 N
#: (scratchpad/guard_replay_test.py).
K_LAG = 2.97                # steps of following lag per count/step of rate
EXCESS_MIN = 6.0            # counts of gap beyond lag baseline = contact
PERSIST = 3                 # consecutive qualifying steps before latching
CRUSH_TIERS = [-1.0, 120.0, 60.0]
EPISODES = 30


class JawGuard:
    """Bus-observable jaw safety layer. Wraps action counts, returns counts."""

    def __init__(self):
        self.hist: deque[float] = deque(maxlen=STALL_WINDOW + 1)
        self.cmd_hist: deque[float] = deque(maxlen=STALL_WINDOW + 1)
        self.seat_jaw: float | None = None
        self.streak = 0

    def __call__(self, jaw_meas_counts: float, jaw_cmd_counts: float) -> float:
        self.hist.append(jaw_meas_counts)
        self.cmd_hist.append(jaw_cmd_counts)
        if self.seat_jaw is None and len(self.hist) == STALL_WINDOW + 1:
            # net terms over the window, in closing-positive counts
            net_close = self.cmd_hist[0] - self.cmd_hist[-1]
            followed = self.hist[0] - self.hist[-1]
            feasible = min(net_close, MAX_SLEW_PER_FRAME * STALL_WINDOW)
            rate = max(net_close / STALL_WINDOW, 0.0)
            gap = jaw_meas_counts - jaw_cmd_counts     # meas above deeper cmd
            stall = (
                net_close >= NET_CLOSE_MIN
                and gap >= GAP_MIN
                and followed < STALL_FRACTION * feasible
            )
            creep = rate > 0.5 and gap - K_LAG * rate >= EXCESS_MIN
            self.streak = self.streak + 1 if (stall or creep) else 0
            if self.streak >= PERSIST:
                self.seat_jaw = jaw_meas_counts
        if self.seat_jaw is not None:
            return max(jaw_cmd_counts, self.seat_jaw - GUARD_COUNTS)
        return jaw_cmd_counts


def load_policy(arm: str, root: str):
    """Returns (policy, norm_stats). The stats are not optional: the policy is
    trained on standardised inputs/targets (lerobot 0.3.4 ignores
    dataset_stats), and feeding raw counts sends the arm to the zero pose --
    the first run of this script scored 0/30 with 0.0 N peak force because of
    exactly that."""
    import numpy as np
    from lerobot.policies.act.modeling_act import ACTPolicy

    path = Path(root) / arm
    return ACTPolicy.from_pretrained(path).to(DEVICE), np.load(path / "norm_stats.npz")


@torch.no_grad()
def rollout(env, policy, st, arm: str, guarded: bool):
    scene = env.scene
    scene.reset()
    policy.reset()
    guard = JawGuard()
    z0 = scene.block_pos()[2]
    a_prev = None
    peak_rise, peak_force = 0.0, 0.0
    for _ in range(260):
        s = torch.as_tensor(scene.state_ticks(), dtype=torch.float32)
        s_n = (s - torch.tensor(st["s_mean"])) / torch.tensor(st["s_std"])
        if arm != "base":
            d = (a_prev - s) if a_prev is not None else torch.zeros_like(s)
            d_n = (d - torch.tensor(st["d_mean"])) / torch.tensor(st["d_std"])
            s_in = torch.cat([s_n, d_n])
        else:
            s_in = s_n
        obs = {
            "observation.state": s_in.unsqueeze(0).to(DEVICE),
            "observation.images.front": torch.as_tensor(scene.image("front"))
            .permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
            "observation.images.gripper_fpv": torch.as_tensor(
                scene.image("gripper_fpv")
            ).permute(2, 0, 1).float().div(255).unsqueeze(0).to(DEVICE),
        }
        a_n = policy.select_action(obs).squeeze(0).cpu()
        action = a_n * torch.tensor(st["a_std"]) + torch.tensor(st["a_mean"])
        cmd = action.numpy().astype(float)
        if guarded:
            cmd[5] = guard(float(scene.state_ticks()[5]), cmd[5])
        cmd[5] = max(cmd[5], JAW_SHUT / RAD_PER_TICK)
        # Feed back the APPLIED command, not the policy's intention: on a
        # real bus a[t-1] is what was sent to the servo, and the guard is
        # part of the plant. Feeding the raw intention made delta explode
        # ~10x past training range whenever the guard held the jaw (policy
        # commands deep, jaw held at seat-4), and the delta-gated policy
        # collapsed to 0/30 success while the force cap itself worked
        # (results/guard_h100_v3_rawfeedback.json).
        a_prev = torch.tensor(cmd, dtype=torch.float32)
        scene.hold(cmd * RAD_PER_TICK, frames=2)
        peak_rise = max(peak_rise, scene.block_pos()[2] - z0)
        peak_force = max(peak_force, scene.grip_force())
        if scene.crushed:
            break
    picked = peak_rise > 0.03
    placed = picked and scene.block_in_box() and not scene.crushed
    return dict(placed=placed, crushed=scene.crushed,
                dropped=picked and not placed and not scene.crushed,
                peak_force=peak_force)


def wilson(k, n):
    z, p = 1.96, k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arms", nargs="*", default=["base", "delta", "delta_q16"])
    ap.add_argument("--ckpt", default="checkpoints/act")
    ap.add_argument("--json", default="results/guard_ab.json")
    ap.add_argument("--image-size", type=int, default=96)
    ap.add_argument("--seed", type=int, default=2000)
    args = ap.parse_args()

    rows = []
    print(f"{'arm':>10} {'crush':>6} {'mode':>8} {'success':>9} {'crushed':>8} "
          f"{'dropped':>8} {'peakF med':>10}")
    for arm in args.arms:
        try:
            policy, st = load_policy(arm, args.ckpt)
        except Exception as e:
            print(f"{arm}: checkpoint not found ({e})")
            continue
        policy.eval()
        for crush in CRUSH_TIERS:
            for guarded in (False, True):
                env = DemoEnv(
                    seed=args.seed, cameras=("front", "gripper_fpv"),
                    image_size=args.image_size, stride=2,
                    obs_quantum=16.0 if arm == "delta_q16" else 1.0,
                    crush_newtons=None if crush <= 0 else crush,
                    expert=ExpertConfig(),
                )
                outs = [rollout(env, policy, st, arm, guarded)
                        for _ in range(EPISODES)]
                env.close()
                k = sum(o["placed"] for o in outs)
                lo, hi = wilson(k, EPISODES)
                pf = float(np.median([o["peak_force"] for o in outs]))
                row = dict(arm=arm, crush=crush, guarded=guarded, success=int(k),
                           episodes=EPISODES, ci=[float(lo), float(hi)],
                           crushed=int(sum(o["crushed"] for o in outs)),
                           dropped=int(sum(o["dropped"] for o in outs)),
                           peak_force_median=pf)
                rows.append(row)
                print(f"{arm:>10} {crush:6.0f} {'guard' if guarded else 'raw':>8} "
                      f"{k:5d}/{EPISODES} {row['crushed']:8d} {row['dropped']:8d} "
                      f"{pf:10.1f}", flush=True)

    Path(args.json).parent.mkdir(exist_ok=True)
    with open(args.json, "w") as fh:
        json.dump(rows, fh, indent=2, default=float)   # numpy scalars
    print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
