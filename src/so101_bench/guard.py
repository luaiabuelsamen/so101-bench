"""The jaw guard: bus-observable runtime force cap for any policy or expert.

Detector history (each mode cost a GPU eval before Rule 1 existed; the
engineered answer -- seat by stall, hold by position offset -- is standard
industrial gripper practice, cf. Robotiq object-detection at boot):

- slew mode: comparing measured motion against COMMANDED travel
  false-positives when commands outrun the servo slew limit;
- jitter mode: single-step extrapolation lets sub-count closing jitter
  against the deadband read as a stall;
- onset/backlash mode: the servo takes 2-4 steps to start following a new
  close (longer after direction reversals);
- creep-tracking miss: a slow close (the cloned guarded-move, ~4 counts per
  step) tracks the command INTO the object with no kinematic stall until
  deep in the squeeze -- contact appears instead as gap in excess of the
  rate-proportional following lag.

The v3 detector: (net-travel stall OR lag-excess contact), sustained
``persist`` consecutive steps. Validated offline on 12 instrumented episodes
of the scaled 224px policy: 0 phantoms, 0 misses, force-at-seat median 37 N
max 49 N against raw peaks of 90-156 N.
"""

from __future__ import annotations

from collections import deque


class JawGuard:
    """Wraps jaw action counts, returns capped counts. One instance per
    episode -- the seat latch is single-shot.

    Default constants are per POLICY step (15 Hz, two 30 Hz control frames),
    measured/tuned on the scaled policy's traces. ``K_LAG`` is the servo's
    following lag in steps (gap ~= K_LAG * closing rate when free); re-measure
    it from free motion when the cadence or the servo changes --
    ``at_control_rate`` re-expresses the same physics at 30 Hz.
    """

    def __init__(
        self,
        window: int = 4,
        stall_fraction: float = 0.3,
        max_slew: float = 13.0,
        net_close_min: float = 8.0,
        gap_min: float = 5.0,
        k_lag: float = 2.97,
        excess_min: float = 6.0,
        persist: int = 3,
        guard_counts: float = 12.0,
        max_close_rate: float = 8.0,
    ):
        # guard_counts is the FORCE budget past detected seat, in counts
        # (~2 N/count on this plant): 12 counts ~= +24 N, landing grips at
        # ~25-35 N -- above the ~20 N transport minimum, below the 60 N
        # crush tier. The first value (4 counts ~= +8 N) capped a
        # slow-closing expert at ~10 N total and it dropped everything:
        # Rule 1 pairing, results/expert_guarded_pairing.json.
        #
        # max_close_rate limits CLOSING speed only (opening stays instant --
        # release is the safety direction). A ballistic slam produces its
        # impact spike faster than any detector window; rate-limiting turns
        # the approach quasi-static so the stall/creep terms can act before
        # force builds. Same reason industrial grippers expose a closing
        # speed setting.
        self.window = window
        self.stall_fraction = stall_fraction
        self.max_slew = max_slew
        self.net_close_min = net_close_min
        self.gap_min = gap_min
        self.k_lag = k_lag
        self.excess_min = excess_min
        self.persist = persist
        self.guard_counts = guard_counts
        self.max_close_rate = max_close_rate
        self.hist: deque[float] = deque(maxlen=window + 1)
        self.cmd_hist: deque[float] = deque(maxlen=window + 1)
        self.seat_jaw: float | None = None
        self.streak = 0
        self.last_out: float | None = None

    @classmethod
    def at_control_rate(cls, **overrides) -> "JawGuard":
        """Same physical constants re-expressed per 30 Hz control frame."""
        kw = dict(window=8, max_slew=6.5, k_lag=5.94, persist=6,
                  max_close_rate=4.0)
        kw.update(overrides)
        return cls(**kw)

    def __call__(self, jaw_meas_counts: float, jaw_cmd_counts: float) -> float:
        # closing-speed limit first: commands may open freely but may only
        # deepen max_close_rate counts per call relative to the last APPLIED
        # command
        if self.last_out is not None:
            jaw_cmd_counts = max(jaw_cmd_counts,
                                 self.last_out - self.max_close_rate)
        self.hist.append(jaw_meas_counts)
        self.cmd_hist.append(jaw_cmd_counts)
        if self.seat_jaw is None and len(self.hist) == self.window + 1:
            # net terms over the window, in closing-positive counts
            net_close = self.cmd_hist[0] - self.cmd_hist[-1]
            followed = self.hist[0] - self.hist[-1]
            feasible = min(net_close, self.max_slew * self.window)
            rate = max(net_close / self.window, 0.0)
            gap = jaw_meas_counts - jaw_cmd_counts   # meas above deeper cmd
            stall = (
                net_close >= self.net_close_min
                and gap >= self.gap_min
                and followed < self.stall_fraction * feasible
            )
            creep = rate > 0.5 and gap - self.k_lag * rate >= self.excess_min
            self.streak = self.streak + 1 if (stall or creep) else 0
            if self.streak >= self.persist:
                self.seat_jaw = jaw_meas_counts
        out = jaw_cmd_counts
        if self.seat_jaw is not None:
            out = max(jaw_cmd_counts, self.seat_jaw - self.guard_counts)
        self.last_out = out
        return out
