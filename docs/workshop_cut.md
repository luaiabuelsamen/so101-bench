# The workshop cut: what a 4-page version contains

`THESIS.md` rule 6 says ship 4 pages, one figure, one claim, workshop or arXiv.
The body is currently 6,119 words, roughly eight pages in CoRL format. Four pages
is about 3,200. So the cut is ~2,900 words, 48% of the body.

This file exists so the choice is between two concrete documents rather than
between a paper and an idea. **Luai decides; neither session should cut
unilaterally.**

## Where the words are now

| section | words | |
|---|---:|---|
| Introduction | 937 | |
| Related Work | 759 | |
| Method (4 subsections) | 947 | |
| Ch. behaviour on the plant | 241 | |
| **Observation design at 280** | **1025** | the result |
| Ecosystem study | 465 | the honest negative |
| Runtime guard | 339 | infrastructure, per THESIS |
| **Limitations** | **1256** | larger than the result |
| Conclusion | 115 | |

The Limitations number is the striking one. It grew across 2026-09-03/04 as we
added the operating-point disclosure, the `a[t-2]` confound, arm G, the crush
-disabled disclosure, and the collection-variance note. Every one is honest and
belongs somewhere. Together they make the caveats longer than the finding, which
reads as a paper arguing against itself.

## What survives at 4 pages

Keeping THESIS's own scope: *"a small-data observability result on one servo
family plus an honest negative"*.

| keep | budget | why |
|---|---:|---|
| Introduction | 550 | premise, retroactivity, contributions. The Xie quote earns its place; the six-family taxonomy does not. |
| Method | 350 | $\delta$, $e$, $\hat{K}$, the task's force band. Envelope detail to appendix. |
| Grid result | 750 | E−A, D≈A, the table, one figure. |
| Corpus negative | 350 | pre-registered, failed, reported. Non-negotiable. |
| Limitations | 500 | see below |
| Related work | 400 | Wong, FACTR 2, Hwang, Zeng, Xie. Five citations doing work, not fifteen. |
| Conclusion | 100 | |
| **total** | **3000** | leaves margin |

## What moves out

- **Runtime guard** → appendix or dropped. `THESIS.md` already calls it
  infrastructure, not research, and it costs 339 words plus a figure.
- **Channel behaviour on the plant** → appendix. It supports the method; it is
  not the claim.
- **Envelope, quantisation rule, resolution cliff** → appendix. The rule is
  falsified as a threshold anyway and now needs three sentences of hedging.
- **Appendix D** → one paragraph in the body, full version to supplement. The
  hardware comparison is a strong reviewer answer but not the claim.

## The hard part: Limitations at 500 words

Cutting 1256 → 500 means choosing which caveats are load-bearing. Proposed
ranking, most to least:

1. **Simulation.** Everything except the corpus study is simulated. Unavoidable.
2. **Arm G.** A position-history control matches the raw channel arm, so the
   grid shows temporal information helps and not that this channel is uniquely
   capable. This is the one a reviewer would otherwise find.
3. **Crush disabled.** The headline never exercised the force band.
4. **Operating point.** 280 demos; at 600 the one tested pair was a tie, and E
   was never run there.
5. **E−B confounded by $a[t{-}2]$**, superseded by E−B2.

1–3 are non-negotiable. 4 and 5 compress to a sentence each. Everything else --
collection variance, the compensator ladder, the offset sweep, the corpus proxy
caveats -- moves to the appendix, cited from a single sentence.

## The case for NOT cutting

Stated fairly, because the measurement above is not an argument by itself.

The paper is a coherent full-length submission. The audits of 2026-09-03/04 left
every section verified against its generating data, every citation checked
against literally-grepped source, and 15 numeric claims machine-checked. That is
a stronger artifact than most first submissions, and a 48% cut discards
verification work rather than fat.

The counter is THESIS rule 6 and the fact that ship-and-iterate beats
polish-and-hold, plus rule 8: *"work is finished only when another human has
received it."*

## What is NOT a reason to cut

That the paper looks weaker after tonight. It does, and the reason is that the
weaknesses moved from latent to stated. A shorter paper with the same caveats
removed would be a worse paper, not a better one.
