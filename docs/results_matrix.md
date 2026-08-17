# Overnight results matrix (final)

| arm | data | reach | seat | lift | **place /100** | lift\|seat |
|---|---|---|---|---|---|---|
| base | v1 | 91 | 61 | 10 | **5** | 16% |
| base | v2 | 97 | 63 | 15 | **9** | 24% |
| base | v3 | 98 | 81 | 18 | **2** | 22% |
| delta | v1 | 97 | 38 | 18 | **5** | 47% |
| delta | v2 | 99 | 72 | 40 | **17** | 56% |
| delta | v3 | 100 | 49 | 37 | **21** | 76% |

Recipes: v1 clean demos; v2 +kicks (recovery); v3 +grip-depth diversity.

The two trends that ARE the result:
- **base commitment is flat across all data: 16 -> 24 -> 22%** (representation-limited;
  autopsy: its stall-plan amplitude is 6 counts, and grasp state is unobservable
  without the channel)
- **delta commitment climbs with every data improvement: 47 -> 56 -> 76%**, place
  5 -> 17 -> 21 -- the channel converts data engineering into task progress
- causal evidence (small scale): zeroing delta at a stall collapses the planned lift 14x (482 -> 35
  counts); restoring a demo-typical value recovers it (566)

Trades logged: depth diversity cost delta some seat completion (72 -> 49) while
buying commitment (56 -> 76%) and retention (place|lift 42 -> 57%); kicked data
erodes the crush-gentleness advantage (Phase-1b motion-confound frames).

Next lever (queued): demos_v4 (600 eps @ 224 px, v3 recipe -- collected tonight)
+ H100 50k steps. Autopsy's predicted landing zone: place 35-45%.
