# Pre-registration: real-arm paired trial, base vs excess

**Frozen 2026-09-04, before any `excess` rollout has been run.** Written so the
decision rule cannot be chosen after seeing the outcome.

## What is being tested

Whether the lag-excess channel improves real-arm task success over a
position-only baseline, at 50 demonstrations, with a 115-frame open-loop
jumpstart. Both policies are trained on the identical recording; they differ
only in the observation constructed from it.

## What is already known, and is NOT this test

- `base` on real hardware: **2/18 = 11.1%**, Wilson 95% [0.03, 0.33].
  This is the pipeline at this budget. `base` never observes the channel, so it
  says nothing about the method.
- Simulation, 280 demos, 5 seeds: E 26.6% vs A 6.2%. Different plant, different
  task, two cameras, no jumpstart. It motivates this test; it cannot substitute
  for it.

## Protocol

Alternating `base` / `excess` from the same physical reset, `--jumpstart 115`,
`--max-steps 400`. Operator records the outcome. Trials where something failed
that was not the policy (dropped serial, camera stall) are discarded before the
outcome is known, never after.

## Decision rule

Primary statistic: the paired difference in success count, McNemar's exact test
on discordant pairs.

- **Claim an improvement** only if the exact two-sided p < 0.05.
- At n = 10 pairs that needs essentially all discordant pairs to favour
  `excess`: 5 of 5, or 6 of 7. A 4-vs-1 split is p = 0.375 and is **not** a
  result.
- Report the paired difference with its interval regardless of outcome.

## Declared in advance

1. A null is reportable and expected to be uninformative at this n. It does not
   license "the channel does not help on hardware"; it licenses "this test could
   not resolve it".
2. If `excess` underperforms, the `k_hat` fit is a live alternative explanation
   before the channel is. Its free-motion proxy is contaminated on this data:
   the jaw sits near-closed most of the time, so 66% of frames are labelled
   free-motion including gripping ones, and 7.7% of those are load-saturated.
   That is a measurement defect, not a finding about the channel.
3. The jumpstart frame is part of the result and must be reported with it. At
   115 the policy is handed the arm at the pre-grasp pose; it performs the
   grasp, lift, transport and place, and does not perform the approach.
4. n required for 80% power, unpaired, against base = 11.1%: 19 per arm if
   `excess` is 50%, 32 if 40%, 68 if 30%, 179 if 22%. Pairing reduces these by
   roughly a third. A 10-pair session is powered only for a very large effect.
