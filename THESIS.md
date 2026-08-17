# The sentence

**On low-cost robots, the binding constraint on learned manipulation is
observability — not data scale, not model scale — and the missing observations
are already latent in the actuators and logs the fleet ships with.**

Every project in this repo must argue for or against this sentence. Anything
that does neither gets killed, however fun.

## The portfolio, tested against the sentence

| project | argues | status |
|---|---|---|
| Corpus regrasp study (delta at grasp time vs outcome across public datasets) | for: the latent is recoverable retroactively, ecosystem-wide | **next build** (script drafted; paused for reading week) |
| BC ladder + autopsy (5%→60% from one observability fix; 14× counterfactual) | for: the causal core | done; ship it |
| base_hist result | complicates: raw action history suffices at scale — the *form* is open, and the copycat-shortcut literature bears on it directly | done; framing depends on reading |
| Jaw guard (96% crush elimination, bus-only) | for: the same latent, enforced instead of learned | done; deployable byproduct |
| Sensing characterisation + envelope | foundation: what the latent channel is and where it's valid | done |
| Oracle-first task screening | methodological corollary: benchmarks can't detect modalities nobody checks | done; secondary |
| Hardware calibration (delta vs Present_Load vs Present_Current vs load cell) | decides whether the cheap-channel claim survives contact with a real servo | **blocking**; 20-min bench session, script ready |
| LeRobot PR: `a[t-1]` / delta in observation configs, with corpus result attached | ship-the-fix milestone; the thesis made useful to everyone on the stack | after corpus + reading |
| More sim recipes / DP / VLA arms / RL | argues nothing new about the sentence | **killed** until a reading-derived question requires them |

## Standing rules (adopted 2026-08-16)

1. One sentence, years. This file.
2. Read before build — 30–50 papers on the thread before the next line of
   code (`docs/reading.md`).
3. Hardware or it isn't a claim.
4. Reviewer's baseline first (for us: `Present_Load` / `Present_Current`).
5. Figure before code — axes and expected curve drawn first.
6. Ship every 6–8 weeks: 4 pages, one figure, one claim; workshop or arXiv.
7. Reuse the platform (scene, experts, autopsy pipeline); stop rewriting.
8. Work is finished only when another human has received it.

This week: reading, and this sentence.
