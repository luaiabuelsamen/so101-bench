"""Can the contact transition be decoded from the bus-observable channel?

    python scripts/event_classifier.py --data data/contact_transitions.parquet

The learned counterpart of the hand-built stall detector, asked the question
that detector failed (0.78 adversarial grasp-ready precision): from local
signal history alone, classify each frame as

    0 pre_seat   -- free closing or sliding push (no valid pinch yet)
    1 valid_seat -- two-pad contact with an upright, unspun object
    2 obstructed -- two-pad contact on a jammed/spun/tilted object, or the
                    known-invalid fast-transport regime

(The 4-class variant splits pre_seat into free vs sliding and is reported as
a secondary breakdown.)

INPUT ARMS, identical architecture and training budget, differing only in the
features -- exactly the audit's design:

    delta_q1    jaw tracking error at native resolution, window of 16 frames
    delta_q4    same episodes, observation quantised OFFLINE to 4 counts
    delta_q16   ... to 16 counts (paired at frame level with q1 by design)
    state_act   raw (s, a) 12-dim history -- delta is a deterministic function
                of this input, so any win for the explicit-delta arms is a
                REPRESENTATION result, not new sensory information
    shuffled    delta_q1 with the window's temporal order permuted -- controls
                for the classifier keying on marginal magnitude rather than
                temporal structure

Splits are by episode with width-bin 2 held out entirely (episode geometry,
never frames). Oracle labels are targets only; no oracle quantity is an input.

Metrics: frame macro-F1; seat-event detection latency (first frame the model
sustains p(valid_seat) > 0.5 for 3 frames, versus ground-truth seat onset);
and the one that matters most -- jam precision: of episodes the model declares
seated, how many were actually jams. The hand detector's answer was 22%.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))

WINDOW = 16
EPOCHS = 12
HELD_OUT_WIDTH_BIN = 2
SEED = 0


def quantise(x: np.ndarray, q: float) -> np.ndarray:
    return np.round(x / q) * q if q > 1.0 else np.round(x)


def build_frames(df: pd.DataFrame, quantum: float) -> pd.DataFrame:
    d = df.copy()
    for j in range(6):
        d[f"s{j}q"] = quantise(d[f"s{j}"].to_numpy(), quantum)
        d[f"a{j}q"] = np.round(d[f"a{j}"].to_numpy())
    d["delta"] = d["a5q"] - d["s5q"]
    return d


def labels_of(df: pd.DataFrame) -> np.ndarray:
    two = df["contact_fixed"].to_numpy() & df["contact_moving"].to_numpy()
    clean = df["upright"].to_numpy() & ~df["spun"].to_numpy()
    fast = (df["phase"] == "fast_transport").to_numpy()
    y = np.zeros(len(df), dtype=np.int64)          # pre_seat
    y[two & clean & ~fast] = 1                     # valid_seat
    y[(two & ~clean) | fast] = 2                   # obstructed / invalid regime
    return y


def windows(df: pd.DataFrame, cols: list[str]):
    """Per-episode sliding windows of `cols`, causal (ends at t)."""
    xs, ys, eps, ts = [], [], [], []
    y_all = labels_of(df)
    for ep, g in df.groupby("episode"):
        arr = g[cols].to_numpy(dtype=np.float32)
        y = y_all[g.index.to_numpy()]
        for t in range(WINDOW, len(g)):
            xs.append(arr[t - WINDOW : t])
            ys.append(y[t])
            eps.append(ep)
            ts.append(t)
    return (np.stack(xs), np.array(ys), np.array(eps), np.array(ts))


def train_eval(x_tr, y_tr, x_te, y_te, seed=SEED):
    import torch
    import torch.nn as nn

    torch.manual_seed(seed)
    dev = "cuda" if torch.cuda.is_available() else "cpu"

    class Net(nn.Module):
        def __init__(self, d_in):
            super().__init__()
            self.gru = nn.GRU(d_in, 32, batch_first=True)
            self.head = nn.Linear(32, 3)

        def forward(self, x):
            h, _ = self.gru(x)
            return self.head(h[:, -1])

    # normalise per feature from train stats
    mu = x_tr.reshape(-1, x_tr.shape[-1]).mean(0)
    sd = x_tr.reshape(-1, x_tr.shape[-1]).std(0) + 1e-6
    x_tr = (x_tr - mu) / sd
    x_te = (x_te - mu) / sd

    counts = np.bincount(y_tr, minlength=3).astype(np.float32)
    weights = counts.sum() / (3 * np.maximum(counts, 1))
    net = Net(x_tr.shape[-1]).to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=1e-3)
    lossf = nn.CrossEntropyLoss(weight=torch.tensor(weights, device=dev))
    xt = torch.tensor(x_tr, device=dev)
    yt = torch.tensor(y_tr, device=dev)
    n = len(xt)
    for _ in range(EPOCHS):
        perm = torch.randperm(n, device=dev)
        for i in range(0, n, 512):
            idx = perm[i : i + 512]
            opt.zero_grad()
            loss = lossf(net(xt[idx]), yt[idx])
            loss.backward()
            opt.step()
    with torch.no_grad():
        logits = net(torch.tensor(x_te, device=dev))
        prob = torch.softmax(logits, dim=1).cpu().numpy()
    return prob


def macro_f1(y, pred) -> float:
    f1s = []
    for c in range(3):
        tp = ((pred == c) & (y == c)).sum()
        fp = ((pred == c) & (y != c)).sum()
        fn = ((pred != c) & (y == c)).sum()
        p = tp / (tp + fp) if tp + fp else 0.0
        r = tp / (tp + fn) if tp + fn else 0.0
        f1s.append(2 * p * r / (p + r) if p + r else 0.0)
    return float(np.mean(f1s))


def event_metrics(df, prob, eps, ts, y):
    """Episode-level: seat declarations vs truth; latency on true seats."""
    lat, declared_jam, declared_ok = [], 0, 0
    truth = labels_of(df)
    for ep in np.unique(eps):
        m = eps == ep
        p_seat = prob[m][:, 1]
        t_ep = ts[m]
        # sustained declaration: p > 0.5 for 3 consecutive windows
        fire = None
        run = 0
        for i, p in enumerate(p_seat):
            run = run + 1 if p > 0.5 else 0
            if run >= 3:
                fire = t_ep[i]
                break
        g = df[df["episode"] == ep]
        y_ep = truth[g.index.to_numpy()]
        true_seat = next((t for t, yy in enumerate(y_ep) if yy == 1), None)
        # any declaration on an episode that never truly seats is false --
        # this covers jams, topping, empty and escaped closures uniformly
        never_seats = not (y_ep == 1).any()
        if fire is not None:
            if never_seats:
                declared_jam += 1
            else:
                declared_ok += 1
        if fire is not None and true_seat is not None:
            lat.append((fire - true_seat) / 30.0)
    prec = declared_ok / (declared_ok + declared_jam) if declared_ok + declared_jam else None
    return dict(
        seat_declarations=declared_ok + declared_jam,
        declared_on_jam=declared_jam,
        grasp_ready_precision=prec,
        latency_median_s=float(np.median(lat)) if lat else None,
        latency_p95_s=float(np.percentile(lat, 95)) if lat else None,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=str, default="data/contact_transitions.parquet")
    args = parser.parse_args()
    df0 = pd.read_parquet(args.data)
    print(f"{len(df0)} frames, {df0['episode'].nunique()} episodes; "
          f"held-out width bin {HELD_OUT_WIDTH_BIN}")

    arms = [
        ("delta_q1", 1.0, ["delta"], False),
        ("delta_q4", 4.0, ["delta"], False),
        ("delta_q16", 16.0, ["delta"], False),
        ("state_act", 1.0, [f"s{j}q" for j in range(6)] + [f"a{j}q" for j in range(6)],
         False),
        ("shuffled", 1.0, ["delta"], True),
    ]
    rng = np.random.default_rng(SEED)
    print(f"\n{'arm':>10} {'macroF1':>8} {'jam-safe prec.':>15} {'lat med':>8} "
          f"{'lat p95':>8}")
    results = {}
    for name, q, cols, shuffle in arms:
        d = build_frames(df0, q)
        x, y, eps, ts = windows(d, cols)
        if shuffle:
            for i in range(len(x)):
                x[i] = x[i][rng.permutation(WINDOW)]
        te = np.isin(eps, d[d["width_bin"] == HELD_OUT_WIDTH_BIN]["episode"].unique())
        prob = train_eval(x[~te], y[~te], x[te], y[te])
        pred = prob.argmax(1)
        f1 = macro_f1(y[te], pred)
        ev = event_metrics(d, prob, eps[te], ts[te], y[te])
        results[name] = dict(macro_f1=f1, **ev)
        prec = ev["grasp_ready_precision"]
        print(f"{name:>10} {f1:8.3f} "
              f"{('%.2f' % prec) if prec is not None else '-':>15} "
              f"{('%.2fs' % ev['latency_median_s']) if ev['latency_median_s'] is not None else '-':>8} "
              f"{('%.2fs' % ev['latency_p95_s']) if ev['latency_p95_s'] is not None else '-':>8}",
              flush=True)

    print("\nreference: hand-built stall detector grasp-ready precision 0.78, "
          "latency 0.07-0.10 s (Phase 1c)")
    import json

    Path("results").mkdir(exist_ok=True)
    with open("results/event_classifier.json", "w") as fh:
        json.dump(results, fh, indent=2)
    print("wrote results/event_classifier.json")


if __name__ == "__main__":
    main()
