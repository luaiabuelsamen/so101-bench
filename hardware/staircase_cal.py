"""Real-arm force calibration: delta vs Present_Load vs Present_Current.

THE PREMISE QUESTION THIS ANSWERS. The STS3215 already reports load and
current over the bus, so the first reviewer question is: why estimate force
from tracking error at all? Two candidate answers, one argued and one
measured. Argued: the public LeRobot corpus records only positions and goals
-- neither load nor current -- so `delta` is the only force channel
recoverable from the thousands of datasets that already exist (this repo's
corpus study measured exactly that channel across 555 of them). Measured:
THIS script logs all three channels against a load cell simultaneously, so
the same 20-minute session yields the delta-vs-Load-vs-Current comparison --
linearity, noise floor, and resolution of each. If Present_Load beats delta
on live hardware, that is the finding, and delta's claim narrows to the
recorded-corpus and guard use cases.

THE POINT. Every envelope number in this project (2.00 N/count, the ±9.2 N
budget, F_min = 20 N, the cliff locations) currently inherits MuJoCo's contact
model (`impratio=10`, hand-set pad `solimp`). This 20-minute measurement
replaces that provenance with hardware: a goal-position staircase pressed into
a scale gives the true N-per-count curve, its linearity range, and where the
servo saturates. One plot converts the simulation study into a hardware claim.

BENCH SETUP (your 20 minutes):
  1. Kitchen scale (or load cell) flat on the table, a rigid spacer on the pan
     (anything stiff; a wooden block works).
  2. Position the arm BY HAND (torque off) so the closed gripper's fixed jaw
     tip rests just above the spacer — a few mm gap. Note the scale's zero.
  3. Run:  python hardware/staircase_cal.py --port /dev/ttyACM0 --joint wrist_flex
     The script steps Goal_Position DOWN in 1-count increments, holding ~1 s
     per step. At each hold, READ THE SCALE and type the grams shown, Enter.
     Type 'x' + Enter to stop (do this when the scale stops rising = servo
     saturated, or at ~2 kg to be kind to the arm).
  4. It saves hardware/staircase_<joint>.json and prints the fitted N/count,
     the linear range, and the saturation point.

Repeat --direction up for the unloading branch (hysteresis), and optionally on
a second joint. Everything else is automatic.

SAFETY: steps are single encoder counts with a 1 s hold — the gentlest
possible loading. Ctrl-C at any point returns the joint to its start position.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode  # noqa: E402

# lerobot's bus layer (same tree the sim work pins)
from lerobot.motors.feetech import FeetechMotorsBus  # noqa: E402

G = 9.81


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--joint", default="wrist_flex",
                    help="which servo presses (gripper/wrist_flex/elbow_flex)")
    ap.add_argument("--motor-id", type=int, default=None,
                    help="override bus id if the name mapping differs")
    ap.add_argument("--direction", choices=("down", "up"), default="down")
    ap.add_argument("--max-steps", type=int, default=60)
    ap.add_argument("--hold-s", type=float, default=1.0)
    args = ap.parse_args()

    # Standard SO-101 follower id map; override with --motor-id if yours differs.
    ids = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
           "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
    mid = args.motor_id or ids[args.joint]
    bus = FeetechMotorsBus(
        port=args.port,
        motors={args.joint: Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)},
    )
    bus.connect()
    try:
        start = int(bus.read("Present_Position", args.joint, normalize=False))
        print(f"connected; {args.joint} at {start} counts. "
              f"Zero your scale now, then press Enter to begin.")
        input()
        rows = []
        goal = start
        step = -1 if args.direction == "down" else +1
        for k in range(args.max_steps):
            goal += step
            bus.write("Goal_Position", args.joint, goal, normalize=False)
            time.sleep(args.hold_s)
            present = int(bus.read("Present_Position", args.joint, normalize=False))

            def reg(name):
                try:
                    return int(bus.read(name, args.joint, normalize=False))
                except Exception:
                    return None

            load, current = reg("Present_Load"), reg("Present_Current")
            reading = input(
                f"step {k+1}: goal {goal} present {present} "
                f"delta {goal - present:+d} load {load} curr {current} "
                f"| scale grams> ").strip()
            if reading.lower() in ("x", "q", ""):
                break
            rows.append(dict(step=k + 1, goal=goal, present=present,
                             delta=goal - present, present_load=load,
                             present_current=current,
                             newtons=float(reading) / 1000.0 * G))
        out = Path(__file__).parent / f"staircase_{args.joint}_{args.direction}.json"
        out.write_text(json.dumps(rows, indent=2))
        print(f"\nwrote {out} ({len(rows)} points)")
        if len(rows) >= 5:
            import numpy as np
            d = np.array([r["delta"] for r in rows], float)
            f = np.array([r["newtons"] for r in rows], float)
            # fit on the pre-saturation range: drop trailing points where force
            # stopped rising
            rising = np.ones(len(f), bool)
            for i in range(len(f) - 1, 0, -1):
                if f[i] <= f[i - 1] + 0.05:
                    rising[i] = False
                else:
                    break
            a, b = np.polyfit(d[rising], f[rising], 1)
            rmse = float(np.sqrt(np.mean((a * d[rising] + b - f[rising]) ** 2)))
            print(f"delta:  {a:.3f} N/count  bias {b:+.2f} N  "
                  f"RMSE {rmse:.2f} N over {int(rising.sum())} points")
            for chan in ("present_load", "present_current"):
                v = np.array([r[chan] for r in rows], float)
                if np.isfinite(v).all() and v[rising].std() > 0:
                    aa, bb = np.polyfit(v[rising], f[rising], 1)
                    rr = float(np.sqrt(np.mean((aa * v[rising] + bb - f[rising]) ** 2)))
                    print(f"{chan}: {aa:.4f} N/unit  RMSE {rr:.2f} N  "
                          f"(vs delta RMSE {rmse:.2f})")
            print(f"sim value was 2.00 N/count -- ratio {a / 2.00:.2f}x")
            if (~rising).any():
                print(f"saturation onset around delta = {d[rising][-1]:.0f} counts, "
                      f"{f[rising][-1]:.1f} N")
    finally:
        try:
            bus.write("Goal_Position", args.joint, start, normalize=False)
        except Exception:
            pass
        bus.disconnect()
        print("returned to start; disconnected.")


if __name__ == "__main__":
    main()
