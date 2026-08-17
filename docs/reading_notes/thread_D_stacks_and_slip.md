# Thread D — Observation Specs of the Dominant Stacks, and Grasp/Slip Detection Without Tactile

Read against the thesis: *"On low-cost robots, the binding constraint on learned manipulation is
observability — not data scale, not model scale — and the missing observations are already latent
in the actuators and logs the fleet ships with."*

Claim under test (Half 1): "the field's default observation spec omits action history / load
information, mostly without stated justification."
Question under test (Half 2): is grasp-state-from-actuator-feedback established practice in
industrial hardware while absent from the learning stacks?

All URLs below were fetched during this reading session (2026-08-16). Quotes are verbatim from the
fetched page unless marked "closely paraphrased."

---

## HALF 1 — What the dominant open stacks actually observe

### Summary table

| Stack | Proprioceptive input | Obs. history | Past actions as input | Stated rationale for the spec | Source (fetched) |
|---|---|---|---|---|---|
| ACT / ALOHA (Zhao et al. 2023) | Current follower joint positions (14-D); no velocity/torque/current | 1 step | No | History-conditioning avoided citing causal confusion; nothing said about omitting load/effort | [arXiv HTML v1](https://arxiv.org/html/2304.13705v1) |
| Diffusion Policy (Chi et al. 2023) | EE pose(s) + gripper width(s) (real); low-dim state (sim); no torque/current | To = 2 | No | Empirical: history beyond 2 steps *hurts* vision-based DP (App. B.1) | [arXiv HTML v5](https://arxiv.org/html/2303.04137v5) |
| OpenVLA (Kim et al. 2024) | **None** — single image + language only | 1 image | No | Simplification to fit the VLM interface; single-image named as a limitation | [arXiv HTML v3](https://arxiv.org/html/2406.09246v3) |
| pi0 (Physical Intelligence 2024) | q_t = "a vector of joint angles" | 1 step | No | None stated | [arXiv HTML v3](https://arxiv.org/html/2410.24164v3) |
| SmolVLA (HF 2025) | "robot sensorimotor state" → one linear token (fields not enumerated; = LeRobot `observation.state`, i.e. motor positions on SO-10x) | 1 step | No | None stated for field choice; ablation only on *where* state enters (VLM vs expert) | [arXiv HTML](https://arxiv.org/html/2506.01844) |
| GR00T N1 (NVIDIA 2025) | "robot state" = joint / EE positions+rotations, gripper state (embodiment-specific MLP encoder) | 1 step (no history described) | No (noised action chunk is the denoising target, not history) | None stated | [arXiv HTML](https://arxiv.org/html/2503.14734) |
| LeRobot framework | `observation.state` = motor `Present_Position` only; `action` = leader `Goal_Position` | ACT 1 (hard-coded), DP 2, pi0 1, SmolVLA 1 | No | None; `n_obs_steps != 1` raises "Multiple observation steps not handled yet" (ACT config) | docs + source, below |

Verdict on the claim: **holds, with one precise amendment** — see summary. No stack observes
velocity, torque, motor current, or commanded-vs-actual error; none feeds back past actions; only
ACT gives any reason (and the reason targets history-conditioning, not the load channel).

---

### 1. ACT / ALOHA — Zhao, Kumar, Levine, Finn, Zhao (RSS 2023)

URL fetched: https://arxiv.org/html/2304.13705v1 (also abstract page https://arxiv.org/abs/2304.13705)

Observation spec: "The observations are composed of the current joint positions of follower robots
and the image feed from 4 cameras" (ALOHA/experiment setup); the action space is "the absolute
joint positions for two robots, a 14-dimensional vector." One observation timestep; no velocities,
no torques, no currents, no past actions. The paper's own action-space justification is the single
most thesis-relevant sentence in the corpus (§IV, training pipeline): **"It is important to use the
leader joint positions instead of the follower's, because the amount of force applied is implicitly
defined by the difference between them, through the low-level PID controller."** So the authors
explicitly recognize that leader−follower position difference *is* the force channel — on the
action side — and then never close the loop by letting the policy observe that difference (the
observation is the follower position alone, so the policy sees only one operand of the
subtraction). The only rationale touching history appears in §IV-A: "Action chunking can mitigate
this issue when the confounder is within a chunk, without introducing the causal confusion issue
for history-conditioned policies [de Haan et al. 2019]" — a reason to avoid *observation history*,
not a reason to discard load information.

**For the thesis (strongly).** ACT's own text asserts that force lives in a[t]−s[t]; the
observation spec then discards it. This is the omission stated in the authors' own words.

### 2. Diffusion Policy — Chi et al. (RSS 2023 / IJRR)

URL fetched: https://arxiv.org/html/2303.04137v5 (also abstract page https://arxiv.org/abs/2303.04137)

Spec: "at time step t the policy takes the latest T_o steps of observation data O_t as input and
predicts T_p steps of actions" (formulation section, §2.3 of v5); actions are outputs only, never
inputs. Proprioception in the real robot experiments is end-effector-level: "The proprioceptive
observation space is extended to include the poses of both end-effectors and the gripper widths of
both grippers" (§7.1); no joint torques or currents anywhere. Hyperparameter Table 7 sets To=2,
Ta=8, Tp=16 for all real tasks, and Appendix B.1 gives the rationale — an empirical one: "We found
state-based Diffusion Policy to be insensitive to observation horizon... However, vision-based
Diffusion Policy, in particular the variant with CNN backbone, see performance decrease with
increasing observation horizon. In practice, we found an observation horizon of 2 is good for most
of the tasks" (Fig. 14: "prefers low but >1 observation horizon"). So DP is the one stack with a
stated, measured reason for a short observation window — and the measurement says *more history
hurts*, which is exactly why the thesis should be framed as "one derived load feature," not "longer
history."

**For the thesis, with a warning.** The omitted channel (gripper width is there, but effort/current
is not) is unjustified; the short horizon is justified empirically, so the paper must not conflate
"add history" with "add the delta."

### 3. OpenVLA — Kim et al. (2024)

URL fetched: https://arxiv.org/html/2406.09246v3

The most extreme spec in the set: "Given an image observation and a language instruction, the model
predicts 7-dimensional robot control actions." No proprioception of any kind, one image, no
history, no past actions. The authors do flag part of this as a limitation: "the current OpenVLA
model has several limitations. First, it currently only supports single-image observations," and
they point at "interleaved image and text" VLM pretraining as the path to richer inputs — but the
discussion is about *images and sensor heterogeneity*, not about load or action-history channels.
The rationale, insofar as one exists, is architectural convenience: the policy must look like a VLM
so it can inherit VLM pretraining.

**For the thesis.** Proprioception is dropped entirely, without task-level justification; the
stated future-work direction is more sensors/frames, not recovering what the actuators already
report.

### 4. pi0 — Black et al., Physical Intelligence (2024)

URL fetched: https://arxiv.org/html/2410.24164v3 (also abstract page https://arxiv.org/abs/2410.24164)

The observation is defined formally: "o_t = [I_t^1, ..., I_t^n, ℓ_t, q_t], where I_t^i is the i-th
image (with 2 or 3 images per robot), ℓ_t is a sequence of language tokens, and q_t is a vector of
joint angles." Architecture section adds: "We add an input q_t for the robot's proprioceptive
state, which is mapped to the transformer embedding dimension using a linear projection." Single
timestep; the action chunk A_t (H=50) is the flow-matching *output*; the noisy action tokens are
denoising targets, not an action-history input. No rationale of any kind is offered for why the
proprioceptive state is joint angles only — no velocities, efforts, or previous commands — in a
paper otherwise meticulous about architecture choices.

**For the thesis.** A frontier lab's flagship VLA reduces proprioception to q_t by definition,
without comment.

### 5. SmolVLA — Shukor et al., Hugging Face (2025)

URL fetched: https://arxiv.org/html/2506.01844

Inputs per Figure 1: "(i) language instruction, (ii) RGB image(s), and (iii) robot sensorimotor
state"; "Sensorimotor states are projected into a single token via a linear layer." The paper never
enumerates what the state vector contains; since it trains on community LeRobot datasets with
SO-100/SO-101 arms, the state is LeRobot's `observation.state` — motor positions (see entry 7). The
only observation-spec ablation is *where* the state enters ("Table 11 indicates including state
information in the VLM leads to significantly better performance"), i.e., they measure that
position-state matters but never ask which state fields are missing. Bonus detail for the thesis:
the async inference stack drops "near-duplicate" observations "compared in joint-space" — so two
moments that differ only in load (same positions, different currents) are, by construction,
*the same observation* to the whole system.

**For the thesis.** The de-facto SO-101 stack treats joint-position space as the complete state
space, at the policy input, in its ablations, and even in its runtime dedup logic.

### 6. GR00T N1 — NVIDIA (2025)

URL fetched: https://arxiv.org/html/2503.14734

"A diffusion transformer (DiT) processes the robot's proprioceptive state and action, which are
then cross-attended with image and text tokens from the Eagle-2 VLM backbone" (§2.1); training data
is unified so that "the input consists of the robot state, visual observations, and language
instruction, and the output is the corresponding motor action." The state contents, where
specified per embodiment, are positional: e.g. for RoboCasa, "The state representation comprises
the position and rotation of both the end-effector and the robot base, as well as the gripper's
state"; the humanoid spaces are "joint position and rotation" of arms/hands/waist/neck. Action
chunks use H=16; no observation history is described anywhere in the architecture, and no rationale
is given for the state field choice. Notably, GR00T N1 standardizes on the LeRobot schema:
"visual observations (observation.images.*) and robot state information (observation.state), while
actions represent the control commands sent to the robot" — the schema's omissions propagate
upward into a foundation model.

**For the thesis.** Embodiment-aware encoders were built to handle *varying dimensions* of
position vectors, not to admit new modalities like effort; and the LeRobot schema is explicitly the
substrate.

### 7. LeRobot framework — docs and source (Hugging Face)

URLs fetched:
- Dataset format: https://huggingface.co/docs/lerobot/lerobot-dataset-v3
- IL tutorial: https://huggingface.co/docs/lerobot/il_robots
- Robot driver: https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/so_follower/so_follower.py
- ACT config: https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/policies/act/configuration_act.py
- Servo register table: https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/motors/feetech/tables.py

The dataset schema is `observation.state`, `action`, `observation.images.*`, `timestamp` (v3 docs,
sample dict). During teleop recording the loop is literally `action = teleop_device.get_action();
robot.send_action(action)` with `observation = robot.get_observation()` (il_robots tutorial) — the
action recorded is the leader's goal position, the state is the follower's present position.
In the driver, `get_observation()` executes `self.bus.sync_read("Present_Position", ...)` and
nothing else from the motor bus; `send_action()` writes `Goal_Position`. Meanwhile the same
repository's Feetech control table for the SO-101's STS3215 servos defines, as read-only registers:
`Present_Position (56)`, `Present_Velocity (58)`, `Present_Load (60)`, `Present_Voltage (62)`,
`Present_Temperature (63)`, `Present_Current (69)` — i.e., the framework *knows* the actuator
publishes load and current and simply never reads them into the observation. Policy configs:
ACT `n_obs_steps: int = 1` and raises "Multiple observation steps not handled yet" if changed;
diffusion `n_obs_steps = 2`; pi0 and SmolVLA `n_obs_steps = 1`; no config has an action-history
input.

**For the thesis (this is its factual foundation).** The corpus-wide omission is a five-line code
path: one `sync_read` call selects position-only observability for every dataset the community
records — while `action` (the leader goal) is stored in the same parquet row, which is what makes
delta = a[t−1] − s[t] retroactively recoverable across thousands of existing datasets.

### 8. The rationale that does exist in the field: causal confusion / copycat

URLs fetched: https://arxiv.org/abs/1905.11979 (de Haan, Jayaraman, Levine, NeurIPS 2019);
https://arxiv.org/abs/2010.14876 (Wen et al., NeurIPS 2020)

Where the field justifies omitting history at all, this is the justification. De Haan et al.:
"access to more information can yield worse performance" because behavioral cloning is "non-causal"
— nuisance correlates (their canonical examples include a brake-light indicator and past actions)
predict the expert action in training and betray the policy under distribution shift. Wen et al.
name the specific failure for histories: "in partially observed settings when expert actions are
strongly correlated over time, the imitator learns to cheat by predicting the expert's previous
action"; their fix removes "excess information about the previous expert action nuisance
correlate, while retaining the information necessary to predict the next action." ACT cites de
Haan directly as its reason to prefer chunking over history-conditioning (entry 1). This is
honest against-thesis pressure: delta = a[t−1] − s[t] contains a[t−1], so a referee will raise
copycat. The counter the evidence supports: the copycat hazard is the *shared positional
component* of past actions, and the delta subtracts precisely that component out, leaving the
load-dependent residual — closer to Wen et al.'s prescription (keep the causal information, remove
the nuisance) than to naive history-conditioning; the 5%→57–63% result is the empirical check that
the residual acts as signal, not shortcut.

**Against the thesis (must be engaged, and can be turned).**

---

## HALF 2 — Grasp outcome and slip from actuator feedback alone (no tactile skin)

### 9. Robotiq 2F-85 / 2F-140 — object detection register (industrial practice)

URLs fetched:
- Manual (Control section): https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190503.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/4.%20Control.htm
- Vendor explainer: https://blog.robotiq.com/knowledge/how-object-detection-works-on-robotiq-grippers

The most widely deployed collaborative gripper family ships grasp-outcome detection as a firmware
feature, computed from exactly the commanded-vs-actual position discrepancy plus a motor-current
threshold. The manual's gOBJ register: 0x01 "Fingers have stopped due to a contact while opening
before requested position. Object detected opening"; 0x02 same while closing; and — the sentence to
quote — **0x03: "Fingers are at requested position. No object detected or object has been loss /
dropped."** Mechanism: "The force will fix the maximum current sent to the motor while in motion.
If the current limit is exceeded, the fingers stop and trigger an object detection notification."
The vendor blog states the design plainly (closely paraphrased): Robotiq grippers use no tactile
sensors; they infer object presence from position and force feedback — whether the requested finger
position was reached under the configured force. In other words, industrial grasp verification *is*
sign(goal − present position) with a current limit — the same delta the learning stacks discard.

**For the thesis (the irony, on vendor letterhead).**

### 10. SCHUNK EGK — workpiece loss detection (second industrial source)

URL fetched: https://schunk.com/us/en/gripping-systems/parallel-gripper/egk/c/PGR_6557

SCHUNK's small-components electric gripper advertises, as a headline feature, "maximum process
reliability by avoiding workpiece loss due to integrated gripping force maintenance with loss
detection," built on an "integrated absolute encoder" (position feedback) plus a holding
brake/elastic pretension for force maintenance. That is: a second major industrial vendor treats
detecting a lost part from motor/encoder feedback as a solved, sellable capability, no tactile
skin involved. Details of the detection algorithm are not on the product page (the operating
manual PDF was not fetched), but the feature's existence and its encoder basis are explicit.

**For the thesis.** Two of the largest gripper vendors ship grasp/loss state from actuator
feedback as standard; neither signal has an analog in any Half-1 observation spec.

### 11. Current as Touch — proprioceptive contact from motor current (2026)

URL fetched: https://arxiv.org/abs/2607.03529

Recent academic confirmation that the actuator-latent channel is rich: the authors use "motor
current and joint states," arguing "motor current is closely related to actuator torque" and is
"an intrinsic signal for perceiving contact force, object resistance, and grasp stability without
additional sensing hardware." They report "stable compliant grasping, safer and more efficient
teleoperation, and improved downstream policy learning without external tactile or force sensors"
on multiple dexterous hands, by predicting a compliance reference position for a standard PD
controller. Note the mirror-image of our formulation: they *command* a position whose PD error
generates the desired force; we *read* the position error to infer the force. Same physics, both
directions. [Abstract fetched; full text not read.]

**For the thesis.** Independent, current-generation evidence that grasp stability is decodable
from motor current + joint state alone, and that it improves policy learning.

### 12. FACTR 2 — learned force sensing from commodity-arm signals improves policies (2026)

URL fetched: https://arxiv.org/html/2606.12406v1

Closest prior art found in this thread; must be cited and differentiated. Premise: "Many low-cost
robot arms lack built-in force sensing"; "Motor current is readily provided on most robot arms,
and is approximately related to motor torque through τm = K·Im." Their estimator input is
x_i = [q_{i−h:i}, q̇_{i−h:i}, Δq^d_{i−h:i}] — joint positions, velocities, and **commanded-position
tracking errors over a history window**, i.e., a windowed version of our delta channel — and their
force-aware policy "outperforms prior force-aware policies by over 17% in task progress" on Franka,
AgileX Piper ($2.5k) and YAM arms (e.g., NIST Belt 0.494→0.767). Differentiation for our paper:
FACTR 2 builds a *new* sensing pipeline (calibration rollouts, learned estimator, new data
collected with these signals); our claim is upstream of that — the rawest form of the channel
(a[t−1]−s[t]) is already present, unlabeled, in thousands of *existing* position-only LeRobot
datasets and needs no new hardware, estimator, or recollection to exploit.

**For the thesis — and the prior-art bar the paper must clear explicitly.**

### 13. Proprioception-based grasping with a series-elastic gripper (Hang et al., 2018)

URL fetched: https://arxiv.org/abs/1803.09674

The pre-deep-learning statement of the principle: "For robotic manipulation, proprioception is
translated as the combination of joint position and torque sensing," and the paper demonstrates
that "proprioception alone can be the basis for versatile performance, including multiple types of
grasps for objects with multiple shapes and sizes, and transitions between grasps" — fingertip and
enveloping grasps on unknown objects with no vision and no tactile skin, via SEA-based MIMO
control. Useful as the citable definition of proprioceptive grasping and as evidence the control
community considered position+torque sufficient for grasp-state reasoning eight years before the
current learning stacks standardized on position-only. [Abstract fetched; full text not read.]

**For the thesis.**

### 14. Grasp state estimation from tendon-based proprioception (2025)

URL fetched: https://arxiv.org/abs/2509.12969

An underactuated hand whose SEA tendon tensions, "without reliance on vision or tactile feedback,"
suffice to estimate "contact timing, joint angles, relative object stiffness, and external
disturbances" — i.e., a full grasp-state vector recovered from actuator-side signals only, with no
sensors in the fingers. Demonstrates that the information content of actuator feedback extends
beyond binary part-presence to object property estimation. [Abstract fetched; full text not read.]

**For the thesis.**

### 15. The Feeling of Success — Calandra et al. (CoRL 2017), the tactile counterpoint

URL fetched: https://arxiv.org/abs/1710.05512

The canonical grasp-outcome-prediction study: 9,000+ grasp trials with a two-finger gripper and
GelSight sensors, concluding that "incorporating tactile readings substantially improve grasping
performance" over vision alone. This is the strongest nearby claim *for dedicated touch hardware*,
and it frames our lane precisely: the question this thread answers is not whether tactile helps
(it does) but whether the zero-hardware actuator channel — which Calandra's comparison did not
include as a condition — already carries the grasp-outcome bit that the learning stacks currently
get from neither source. [Abstract fetched; full text not read.]

**Orthogonal to the thesis** (supports "grasp outcome is predictable from local sensing"; silent on
actuator-only signals).

---

## Summary — does the omission claim hold, and what is the strongest sentence?

**The claim holds across all seven stacks, with one amendment.** Every surveyed policy consumes, at
most, current positional proprioception: follower joint positions (ACT, pi0, SmolVLA, GR00T,
LeRobot), or EE pose + gripper width (Diffusion Policy), or nothing at all (OpenVLA). None
observes velocity, torque, motor current, or commanded-minus-actual error; none takes past actions
as input; observation history is 1 step everywhere except Diffusion Policy's 2. The amendment:
"without stated justification" is true for the *load channel* everywhere, but not quite true for
*history* — ACT explicitly invokes causal confusion (de Haan 2019) to justify chunking over
history-conditioning, Diffusion Policy measured that history beyond 2 frames hurts its
vision-based variant, and OpenVLA lists single-image input as a known limitation. So the precise,
defensible form of the claim is: the field has articulated reasons to fear *action/observation
history* (copycat, causal confusion) and no reasons at all for discarding *load information* — and
the two got conflated, throwing out the delta with the history. The paper should state this
conflation, then engage copycat head-on, since delta = a[t−1]−s[t] contains a[t−1] (the defense:
the subtraction removes the shared positional component that constitutes the copycat shortcut,
retaining the load-dependent residual — and empirically 5%→57–63%).

**The irony sharpens as hoped.** Industrial parallel grippers have shipped grasp-state-from-
actuator-feedback for a decade: Robotiq's gOBJ register decides "object detected" vs "object has
been loss / dropped" purely from whether the fingers reached the requested position under a motor-
current limit, and SCHUNK sells encoder-based "loss detection" as a process-reliability feature.
The learning stacks discard exactly this signal, even though (a) ACT's authors wrote that force
"is implicitly defined by the difference" between commanded and actual positions, and (b) the
LeRobot driver that feeds the community corpus reads `Present_Position` from a servo whose own
register table (in the same repo) also publishes `Present_Load` and `Present_Current`. Closest
prior art: FACTR 2 (2606.12406) already shows commodity-arm force signals improve policies — but
via a new learned estimator and new data; nobody has claimed the retroactive, corpus-scale
recoverability of the channel from position-only logs.

**Strongest single sentence the evidence supports:** *Every dominant open manipulation stack —
ACT, Diffusion Policy, OpenVLA, pi0, SmolVLA, GR00T N1, and the LeRobot framework beneath them —
trains on logs that record the same commanded-vs-actual position pair from which Robotiq's firmware
has long computed "object detected / object dropped," yet none of them lets the policy see the
difference.*
