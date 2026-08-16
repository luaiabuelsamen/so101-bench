# Overnight results matrix (auto-assembled; delta-v3 pending)

| arm | data | reach | seat | lift | place /100 | lift\|seat |
|---|---|---|---|---|---|---|
| base | v1 | 91 | 61 | 10 | **5** | 16% |
| base | v2 | 97 | 63 | 15 | **9** | 24% |
| base | v3 | 98 | 81 | 18 | **2** | 22% |
| delta | v1 | 97 | 38 | 18 | **5** | 47% |
| delta | v2 | 99 | 72 | 40 | **17** | 56% |
| delta | v3 | – | – | – | – | *pending* |

Data recipes: v1 = clean scripted demos; v2 = + physics kicks (recovery);
v3 = + grip-depth diversity U[12,45] counts.

Headline structure (fill delta-v3 when it lands):
- base plateaus at lift|seat 16-24% across ALL data recipes -> representation-limited
- delta moves with data: place 5 -> 17 -> ? -> the channel makes data engineering pay
- scale run queued: demos_v4 (600 eps @ 224px) + Modal H100 50k steps
