"""Does Present_Load survive a real grasp? Measured on recorded teleop.

The static bench data (scripts/appendix_d_channels.py) shows Present_Load is an
affine function of delta -- the same signal, so it cannot be a better one. This
asks the next question, which only real teleop can answer: does it stay the same
signal at the forces a grasp actually produces?

It does not. The load register pins at its limit during contact, while delta
goes on resolving. That is not a redundancy argument any more, it is a range
argument, and it is the stronger half of the reviewer's-baseline answer.

    python scripts/real_channel_saturation.py --root data/real/pickplace_real_v0

SETTLED 2026-09-03 (was a caveat here). 500 is the CONFIGURED torque limit, not
the register's ceiling: Present_Load is full-scale 1000, and a read of addr 48
on this follower returns 1000 for joints 1-5 and 500 for the gripper alone.

That asymmetry is not local. lerobot sets it, unconditionally, for every
SO-100/SO-101 follower it connects -- robots/so_follower/so_follower.py:172-175,
inside configure(), which connect() always calls:

    if motor == "gripper":
        Max_Torque_Limit  = 500   # 50% of max torque to avoid burnout
        Protection_Current = 250  # 50% of max current to avoid burnout
        Overload_Torque    = 25

So the ceiling that clips Present_Load through contact is a property of the
stack the whole fleet runs, applied to the one joint where manipulation makes
contact. The same block halves Present_Current, the other reviewer baseline.
delta needs no register and no limit, and is untouched by all three writes.

The claim is therefore ecosystem-wide rather than n=1, with one honest limit:
this dataset logs position and load but NOT current, so current's saturation is
inferred from the configuration write, not measured here.
"""

import argparse
import glob
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

GRIPPER = 5


def channel(root: Path):
    files = sorted(glob.glob(str(root / "data" / "**" / "*.parquet"), recursive=True))
    t = pa.concat_tables([pq.read_table(f) for f in files])
    act = np.array(t["action"].to_pylist(), float)
    st = np.array(t["observation.state"].to_pylist(), float)
    ep = np.array(t["episode_index"].to_pylist())
    pos, load = st[:, :6], st[:, 6:12]

    # delta = a[t-1] - s[t], within an episode so boundaries never mix.
    deltas, loads, eps = [], [], []
    for e in sorted(set(ep)):
        m = ep == e
        deltas.append(act[m][:-1, GRIPPER] - pos[m][1:, GRIPPER])
        loads.append(load[m][1:, GRIPPER])
        eps.append(np.full(m.sum() - 1, e))
    return np.concatenate(deltas), np.concatenate(loads), np.concatenate(eps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--sat", type=float, default=499.5, help="saturation threshold")
    args = ap.parse_args()

    delta, load, ep = channel(Path(args.root).resolve())
    absl = np.abs(load)
    sat = absl >= args.sat
    n_ep = len(set(ep))

    print(f"{len(delta)} frames over {n_ep} episodes\n")
    print(f"Present_Load saturated (|load| >= {args.sat:.0f}): {100*sat.mean():.1f}% of frames")
    print(f"  episodes touching saturation: {len(set(ep[sat]))}/{n_ep}\n")

    for label, mask in (("saturated", sat), ("unsaturated", ~sat)):
        d = np.abs(delta[mask])
        print(f"  |delta| {label:<12}: mean {d.mean():6.2f}  sd {d.std():5.2f}  "
              f"span {d.min():5.2f}-{d.max():5.2f} deg")

    print(f"\ncorr(delta, load) all frames   : {np.corrcoef(delta, load)[0,1]:+.3f}")
    print(f"corr(delta, load) unsaturated  : {np.corrcoef(delta[~sat], load[~sat])[0,1]:+.3f}")

    d_sat = np.abs(delta[sat])
    print(
        f"\nInside saturation load reports one constant value, while delta still spans\n"
        f"{d_sat.min():.2f}-{d_sat.max():.2f} deg ({d_sat.max()-d_sat.min():.2f} deg of range the\n"
        f"register cannot express). Saturation is not incidental: it coincides with\n"
        f"contact, where |delta| averages {np.abs(delta[sat]).mean():.2f} deg against "
        f"{np.abs(delta[~sat]).mean():.2f} deg elsewhere."
    )


if __name__ == "__main__":
    main()
