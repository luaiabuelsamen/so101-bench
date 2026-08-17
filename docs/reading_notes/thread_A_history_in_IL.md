# Thread A — Observation histories and causal misattribution in imitation learning

*Reading pass 2026-08-16. 15 papers read (abstract + intro + relevant sections via
arXiv/ar5iv HTML), 3 sighted. Every URL below was actually fetched.*

**Thesis being read against:** on low-cost robots, the binding constraint on learned
manipulation is observability — not data scale, not model scale — and the missing
observations are already latent in the actuators and logs the fleet ships with.

**Our measured results these papers are judged against:** standard ACT (state `s[t]` +
images, never the previous action) plateaus at 16–24% grasp-commitment across three
data recipes; appending either the servo tracking error `delta = a[t-1] − s[t]` or the
**raw previous action** `a[t-1]` lifts task success ~5% → 57–63% (600 demos, 224 px,
50k steps). Zeroing the delta input at a stall collapses the planned lift chunk 14×
(482 → 35 counts). Teacher-forced (TF) prediction accuracy is ~97% in every phase for
**all** arms, with and without history.

**The question this thread answers:** the canon says history inputs cause shortcut
failures (causal confusion / copycat / inertia / latching); our history input helped
10×. Under what conditions does each regime hold, and has anyone already reconciled
them — or already made our point?

Each entry carries an audit line: `[Autocorrelation | Latent observable from current
obs? | Shortcut measured or assumed?]`.

---

## Part 1 — The shortcut canon: where history hurts

### 1. de Haan, Jayaraman, Levine — "Causal Confusion in Imitation Learning", NeurIPS 2019
<https://arxiv.org/abs/1905.11979> · <https://ar5iv.labs.arxiv.org/html/1905.11979>

The canonical "more information can yield worse performance" result: they inject the
expert's **previous action** into the observation (concatenated into the state for
MountainCar/Hopper, overlaid as a symbol on frames for Pong/Enduro/UpNDown) and BC
collapses (Hopper ~3000 → ~0 reward) while held-out validation loss *improves*; in
their GTA-V driving experiment, history conditioning gives better validation
perplexity (0.834 vs 0.989) but ~2× the collisions. The tasks are chosen so the
original observation is (near) sufficient — the injected channel's only content is
`a[t-1]`, pure nuisance. The shortcut is measured, not assumed: the diagnostic is the
divergence between held-out loss and deployment reward, plus inspection of learned
causal graphs; the fix is interventional (graph-parameterized policies disambiguated
by expert queries or rollout returns). Critically for us, the phenomenon's signature
is *TF-better / rollout-worse*; our runs show *TF-unchanged (~97% all arms) /
rollout-10×-better* — the exact mirror — so by this paper's own diagnostic our delta
channel is not a confound. They flag but never resolve the genuinely partially
observed case ("history would seem a natural part of the state space for real-world
driving"), and their method assumes disentangled observations (it degrades on rotated
states).
`[Autocorrelation: high (standard control) | Latent observable: yes, by construction | Shortcut: measured (loss-vs-reward divergence, graph inspection)]`

**Orthogonal-to-For the thesis:** condemns `a[t-1]` only where it carries no task
latent, and supplies the diagnostic under which our channel passes.

### 2. Wen, Lin, Darrell, Jayaraman, Gao — "Fighting Copycat Agents in Behavioral Cloning from Observation Histories", NeurIPS 2020
<https://arxiv.org/abs/2010.14876> · <https://ar5iv.labs.arxiv.org/html/2010.14876>

Names the copycat shortcut and its two conditions: "(i) expert actions over time are
strongly correlated, and (ii) past expert actions are easily recovered from the
observation history" — both hold *maximally* in our `base_hist` arm (`a[t-1]` is
handed over verbatim), yet we improved 10×, so the conditions are necessary for the
shortcut but not sufficient for net harm. Their own setting is explicitly partially
observed (MuJoCo with velocities/forces stripped from the state), and in their own
comparison BC-OH (history) *beats* BC-SO (single obs) in 5 of 6 environments
(HalfCheetah −38 → 820); the honest reading of the canon's flagship is that history
**under-delivers because of a shortcut tax**, not that it hurts on net. The shortcut
is measured with a probe we should steal: an MLP predicting `a[t]` from past actions
alone — the BC-OH policy's actions are ~5× more predictable-from-history than the
expert's (Walker2D MSE 0.46e-2 vs 2.47e-2), and the predictability ratio
anti-correlates with reward. Their fix is an adversarial information bottleneck that
removes `a[t-1]`-specific information while keeping what is shared with `a[t]` — the
exact inverse of our intervention, which *injects* `a[t-1]`; both are consistent once
you ask what the channel carries beyond the copyable part (velocity there, contact
force here). The missing measurement on our side is exactly their probe run on our
checkpoints.
`[Autocorrelation: high | Latent observable: no (velocity hidden) — history genuinely needed | Shortcut: measured (predictability probe)]`

**Orthogonal-to-For the thesis:** defines the conditions under which our result
"should" have failed and hands us the instrument that will show why it didn't.

### 3. Wen, Lin, Qian, Gao, Jayaraman — "Keyframe-Focused Visual Imitation Learning", ICML 2021
<https://arxiv.org/abs/2106.06452> · <https://ar5iv.labs.arxiv.org/html/2106.06452>

Locates the copycat harm at rare **action changepoints**: "when expert actions are
highly temporally correlated, the demonstration dataset has a very tiny fraction of
important 'changepoint' samples," so the BC loss is dominated by smooth segments
where copying wins; their fix upweights keyframes found by a copycat-MLP's action
prediction error (top 10% × 5), taking history-BC on CARLA from below single-frame
(33.0 vs 42.7% success) to above it (43.4%). The decisive ablation for us is
CARLA-without-speed: with the ego-speed latent hidden, single-frame BC craters to
9.2%, history BC reaches 25.7%, keyframe-weighted history 36.8% — in a genuinely
latent-deficient task, **history flips to strongly positive even before their fix**.
Their abstract states the field's tension verbatim: "In partially observable settings,
imitation policies must rely on observation histories, but many seemingly paradoxical
results show better performance for policies that only access the most recent
observation." Our grasp-commitment moment is precisely a changepoint, and delta is
the only channel that disambiguates it (jaws-closed-on-block vs jaws-closed-on-air
are visually identical at 224 px); their framework predicts the benefit should
concentrate at that keyframe, which is what our 14× lift-chunk counterfactual shows.
`[Autocorrelation: high | Latent observable: varied experimentally (speed shown/hidden) | Shortcut: measured (APE probe; per-changepoint error)]`

**For the thesis:** the sign of the history effect tracks the observability deficit,
measured — the closest the copycat line comes to our claim, still without naming
actuator latents.

### 4. Chuang, Yang, Wen, Gao — "Resolving Copycat Problems in Visual Imitation Learning via Residual Action Prediction", ECCV 2022
<https://arxiv.org/abs/2207.09705> · <https://ar5iv.labs.arxiv.org/html/2207.09705>

Diagnosis: history networks learn to recover `a[t-1]` from frames and predict `a[t]`
from it; their fix is architectural — the memory branch may only predict the residual
`a[t] − a[t-1]`, with a stop-gradient into the current-frame branch, removing the
incentive to encode the previous action at all. They state when history is genuinely
needed ("the most recent visual frame usually misses essential information, such as
the objects' motion and appearance that are occluded in the current frame"), and
their history baselines already beat single-frame on their partially observed setups
(CARLA NoCrash-dense 34.1 vs 13.1 successes; theirs 52.0). The structural rhyme with
our result is exact and inverted: they subtract the copyable component on the
**target** side; our delta = `a[t-1] − s[t]` subtracts it on the **input** side — and
what survives the subtraction on a position-controlled servo is the tracking error,
i.e. the force latent, which is why *injecting* rather than removing
`a[t-1]`-information is the right move on our hardware.
`[Autocorrelation: high | Latent observable: no (occlusion/motion by design) | Shortcut: measured (baseline comparisons; information-flow analysis)]`

**Orthogonal-to-For the thesis:** same algebra, opposite direction — confirms every
fix in this line works by isolating the non-copyable information in history.

### 5. Codevilla, Santana, López, Gaidon — "Exploring the Limitations of Behavior Cloning for Autonomous Driving", ICCV 2019
<https://arxiv.org/abs/1904.08980> · <https://ar5iv.labs.arxiv.org/html/1904.08980>
(venue confirmed via ICCV open-access listing / UAB research portal)

The **inertia problem**: with ego-speed in the observation, "the probability it stays
static is indeed overwhelming in the training data. This creates a spurious
correlation between low speed and no acceleration, inducing excessive stopping and
difficult restarting." This is the nearest structural analog to our delta — a scalar
self-state channel that is an *effect of past actions* — and there it **hurt**,
measured as the fraction of episodes frozen ≥ 8 s, a rate that *grew with more
training data*. Speed was included because it is necessary ("without the speed
context, the model cannot learn if and when it should accelerate or brake"):
necessity and harm coexist in the same channel, and their mitigation is an auxiliary
speed-prediction head plus dropout, not removal. The sign difference from our case is
instructive: stopped-at-zero-speed is a massively over-represented, self-consistent
near-absorbing regime in driving data, while stalled-jaws-under-load in our demos is
always followed within the chunk by the expert's lift — at the keyframe, the
channel-conditional expert action distribution is decisive there and degenerate here.
Also the cleanest published evidence that more data amplifies misattribution — the
anti-data-scale arm of the thesis.
`[Autocorrelation: high; stopping near-absorbing | Latent observable: speed necessary AND harmful | Shortcut: measured (stuck-episode rate vs dataset size)]`

**Against-in-form, For-in-substance:** proves self-state channels can be shortcut
fuel; any resolution must condition on what the channel disambiguates at decision
points — which is the thesis's observability axis.

### 6. Bansal, Krizhevsky, Ogale — "ChauffeurNet: Learning to Drive by Imitating the Best and Synthesizing the Worst", arXiv:1812.03079, 2018 (Waymo)
<https://arxiv.org/abs/1812.03079> · <https://ar5iv.labs.arxiv.org/html/1812.03079>

At 30M expert examples, feeding the ego's past-motion history makes the network
"learn to 'cheat' by just extrapolating from the past rather than finding the
underlying causes of the behavior" (stopping because it was decelerating, not because
of the stop sign); the fix is 50% dropout on the past-pose channel. The shortcut is
measured interventionally — scenario ablations rendering stop signs and other
vehicles in and out to verify the model "responds to the appropriate causal factors" —
the same species of autopsy as our delta-zeroing counterfactual. The ego past-pose
carries no latent the mid-level scene representation doesn't already expose, so the
channel is all shortcut and no sensor: the cleanest example of the pure-nuisance pole
of the axis. Confirms that scale does not fix misattribution, and canonizes the folk
practice — truncate or drop out self-history — that ACT and diffusion policies
silently inherit.
`[Autocorrelation: high | Latent observable: yes (rendered scene sufficient) | Shortcut: measured (scenario ablations)]`

**Orthogonal-to-For the thesis:** fixes the far pole of the channel-content axis and
supplies the interventional-autopsy methodology.

---

## Part 2 — Theory that separates the regimes

### 7. Spencer, Choudhury, Venkatraman, Ziebart, Bagnell — "Feedback in Imitation Learning: The Three Regimes of Covariate Shift", arXiv:2102.02872, 2021
<https://arxiv.org/abs/2102.02872> · <https://ar5iv.labs.arxiv.org/html/2102.02872>

Opens from the practitioner observation that "conditioning policies on previous
actions leads to a dramatic divergence between 'held out' error and performance of
the learner in situ," and re-derives causal confusion as ordinary covariate shift
amplified by feedback (past action → input features) — no metaphysical confounding
needed when all variables are observed. Three regimes by realizability and density
ratio: BC is provably fine when the expert is realizable in the class (ε≈0) or when
the learner-expert density ratio stays bounded; interaction is only *needed* in the
hard regime, and their benchmark audit finds standard suites "realizable and simple,"
where "naive behavioral cloning provides excellent results." The translation to our
numbers is sharp: TF ≈ 97% for every arm means near-realizability on-support
everywhere, but the base policy's residual error is concentrated at grasp keyframes
where the expert's action is **not a function of the observed state at all** —
misspecification induced by the observation space, not the hypothesis class; adding
delta restores realizability exactly there, moving the problem into the regime where
BC is guaranteed to work. Their remedy (ALICE: simulator-corrected losses without
expert queries) is the algorithmic road we didn't need to take.
`[Autocorrelation: n.a. (theory) | Latent observable: parameterized via realizability | Shortcut: reframed as covariate shift; benchmark audit measured]`

**For the thesis:** supplies the formal language — our fix changes the observation to
restore realizability, rather than changing the algorithm to survive its absence.

### 8. Swamy, Choudhury, Bagnell, Wu — "Causal Imitation Learning under Temporally Correlated Noise", ICML 2022, PMLR 162:20877–20890
<https://arxiv.org/abs/2202.01312> · <https://ar5iv.labs.arxiv.org/html/2202.01312> · <https://proceedings.mlr.press/v162/swamy22a.html>

A third mechanism, distinct from copycat and from missing latents: temporally
correlated noise in the **expert's** actions (persistent wind; teleop lag/tremor)
makes past actions confound current states, so even a current-state-only BC latches
onto spurious state-action correlations; with i.i.d. noise BC is unbiased. Their fix
is instrumental-variable regression with past states as instruments — DoubIL (needs a
simulator) and ResiduIL (fully offline) — validated on LunarLander/HalfCheetah/Ant
with injected correlated noise. For us this is a risk-register item rather than a
counter: the sim expert's noise is controlled, but the fleet-corpus study the thesis
proposes runs on human teleop where operator lag is textbook TCN riding on exactly
the `a[t-1]`-vs-`s[t]` gap, so a delta-conditioned corpus policy could inherit the
confound — ResiduIL is the published mitigation to cite. Note their confound needs no
history input at all, which cleanly separates "history in the observation" from
"confounding," two things the copycat debate tends to fuse.
`[Autocorrelation: induced via noise, in the expert | Latent observable: yes; confound is in the data process | Shortcut: proven (SCM) + measured on injected noise]`

**Orthogonal to the thesis:** a genuine caveat for the corpus-scale claim, with the
mitigation already on the shelf.

### 9. Swamy, Choudhury, Bagnell, Wu — "Sequence Model Imitation Learning with Unobserved Contexts", NeurIPS 2022 (Advances in NeurIPS 35:17665–17676)
<https://arxiv.org/abs/2208.02225> · <https://ar5iv.labs.arxiv.org/html/2208.02225>

The paper that owns the reconciliation question. Setting: the expert holds a hidden
context; the learner can only become expert-equivalent by inferring it from history
("asymptotic realizability"); they prove on-policy training is necessary-and-
sufficient under identifiability, while off-policy BC on histories **latches** —
"off-policy methods treat their suboptimal past actions as though they came from the
expert" — with sharp phase transitions in a causal bandit, and a hidden-target-velocity
HalfCheetah where "adding history to BC actually reduces the performance of the
learned policies, in contrast to DAgger." Our result sits squarely inside their
setting (unobserved context = load state; off-policy BC; history input) and came out
opposite, so the boundary conditions matter: (a) our context is identified in **one
step from an exogenous physical residual** — `s[t]` measured against `a[t-1]` — not by
integrating one's own action sequence, so the posterior-collapse mechanism has no
purchase; (b) ACT chunking means the policy conditions on its own action only at
chunk boundaries (~1/100 steps), throttling the latching feedback loop; (c) the latch
earns no TF advantage in our data (97% with or without history), so there is no
gradient toward it. They explicitly provide *no* sufficient conditions for
off-policy-with-history to be safe — the hole our result fills empirically — and
their theory makes a testable prediction for us: on-policy correction (DAgger-style)
on the delta policy should widen the margin further.
`[Autocorrelation: high | Latent observable: no (hidden context by construction) | Shortcut: proven (Bayesian latching argument) + measured (bandit phase diagram, MuJoCo)]`

**For the thesis, as the strongest counter-theory to answer:** our run is the missing
"off-policy safe zone" data point — single-step exogenous identification of the
latent.

### 10. Ortega et al. — "Shaking the foundations: delusions in sequence models for interaction and control", DeepMind tech report, arXiv:2110.10819, 2021
<https://arxiv.org/abs/2110.10819> · <https://ar5iv.labs.arxiv.org/html/2110.10819>

Formalizes why conditioning on one's own actions *as evidence* corrupts inference:
an action sampled without knowledge of the latent carries no information about it,
yet Bayesian conditioning updates the posterior anyway — "whichever action we choose
will convince ourselves of the box configuration" — the auto-suggestive delusion; the
fix is treating self-generated actions as interventions, `P(Θ|do(a))` not `P(Θ|a)`.
They state when the problem vanishes: latent observed before acting, no latent
confounder, or observations rich enough to identify the task on their own. The
application to our channel is decisive: `a[t-1]` **by itself** is delusion fuel (pure
self-evidence), but jointly with the *measured* `s[t]` it becomes the reference point
of a physical measurement — the environment's rejection of the command — so the
informative content of delta is exogenous, and inference from it about load is
intervention-consistent. That the raw-`a[t-1]` arm matches the delta arm (57–63% vs
base ~5%) says the network learns the subtraction on its own: the do-calculus
critique licenses exactly the input we added, *provided the state it is differenced
against is measured rather than remembered*.
`[Autocorrelation: n.a. (theory) | Latent observable: parameterized | Shortcut: proven (SCM); bandit example measured]`

**For the thesis:** the causal-formal reason our history input is sound where naive
action-conditioning is not.

---

## Part 3 — History as a latent-state sensor: where it helps

### 11. Mandlekar, Xu, Wong, Nasiriany, Wang, Kulkarni, Fei-Fei, Savarese, Zhu, Martín-Martín — "What Matters in Learning from Offline Human Demonstrations for Robot Manipulation" (robomimic), CoRL 2021 (Oral)
<https://arxiv.org/abs/2108.03298> · <https://ar5iv.labs.arxiv.org/html/2108.03298>

The field's own benchmark-scale evidence that history **helps** manipulation BC from
human demos: BC-RNN over 10-step windows beats feedforward BC massively on the hard
tasks (Transport PH 17.3% → 71.3%; "the performance gap is larger for longer-horizon
tasks"), attributed to human non-Markovianity ("humans may not act purely based on a
single current observation"). Simultaneously, *adding* proprioception hurts:
"including end effector velocity information, and joint information hurts agents
trained on low-dim observations substantially (49%–88% relative performance drop)" —
channel content, not channel count, decides the sign. There is no copycat probe, no
causal-confusion discussion, and no past-action input anywhere; the help/hurt pattern
is reported as brute empirics, and none of the winning observation sets contain
anything force-bearing or command-vs-achieved shaped — nobody checked. For the
thesis this is the observation-design question sitting open, unasked, in the standard
reference the whole stack tunes against.
`[Autocorrelation: high (teleop) | Latent observable: partially (object state given; human intent not) | Shortcut: not examined — effects measured, mechanism unaddressed]`

**For the thesis:** history already helps at benchmark scale in manipulation,
unexplained; the velocity-hurts finding is the observability axis surfacing unnamed.

### 12. Kumar, Fu, Pathak, Malik — "RMA: Rapid Motor Adaptation for Legged Robots", RSS 2021
<https://arxiv.org/abs/2107.04034> · <https://ar5iv.labs.arxiv.org/html/2107.04034>

Our delta mechanism stated as a design principle, in locomotion: the adaptation
module regresses an 8-d extrinsics latent (payload mass and placement, motor
strength, friction, terrain) from the last 50 steps of **states and actions**,
because "when we command a certain movement of the robot joints, the actual movement
differs from that in a way that depends on the extrinsics." Trained by supervised
distillation from a privileged-RL teacher on on-policy rollouts — so it never risks
copycat (targets are latents, not autocorrelated actions, and the data is on-policy)
— and deployed zero-shot on a real A1. It is the independent proof that
command-vs-achieved actuator signals *observe* the load latent, fast enough to
matter, on cheap hardware. The port nobody had done is ours: the same input pair, but
inside off-policy BC from demonstrations, where the copycat canon predicted it would
backfire — and on a position-controlled arm a single step of the raw pair turns out
to suffice, no estimator, no distillation phase.
`[Autocorrelation: high | Latent observable: no from instantaneous state; yes from command/response history | Shortcut: n.a. — on-policy supervised, immune by construction]`

**For the thesis:** the mechanism's existence proof; our contribution is that it
survives — indeed rescues — off-policy BC.

### 13. Torne Villasevil, Tang, Liu, Finn — "Learning Long-Context Diffusion Policies via Past-Token Prediction", CoRL 2025, PMLR 305:1744–1755
<https://arxiv.org/abs/2505.09561> · <https://arxiv.org/html/2505.09561v1> · <https://proceedings.mlr.press/v305/villasevil25a.html>

The published **flip** of copycat: modern diffusion policies show action-
predictability ratios "significantly below 1, indicating a surprising underuse of
past action information"; naive long-context is *worse* than short (Transport 60% →
0%), and "recent works have empirically found that image-conditioned specialist and
generalist policies degrade with history, leading many works to exclude history
altogether." Their fix is an auxiliary loss predicting **past** action tokens
alongside future ones (plus cached-embedding staged training), roughly 3×-ing
long-context performance; history-critical tasks (counting scoops, remembering which
side to place) go from <30% to ~80%. This is the closest published reconciliation —
both over- and under-reliance on history exist, and the cure is regularizing toward
faithful temporal modeling — but every one of their history-critical latents is
**episodic/semantic memory** (what happened earlier in the task), recovered from
observation history. Bluntly: they made the "two regimes" point first, at CoRL, with
the same probe family (predictability ratio) we should run; they did **not** make the
observability point — nothing about actuators, force, or the commanded-vs-achieved
gap — and their memory regime is a third thing, not ours: slow episodic latents vs
our fast physical latent, auxiliary-loss machinery vs a 6-scalar observation edit.
`[Autocorrelation: high | Latent observable: no (episodic context) | Shortcut & anti-shortcut: measured (predictability ratio in both directions)]`

**For the thesis:** legitimizes history-as-necessary in the flagship-policy era and
leaves the actuator-latent case unclaimed.

---

## Part 4 — The manipulation stack's observation design

### 14. Zhao, Kumar, Levine, Finn — "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware" (ACT/ALOHA), RSS 2023
<https://arxiv.org/abs/2304.13705> · <https://ar5iv.labs.arxiv.org/html/2304.13705>
(RSS 2023 per roboticsproceedings.org rss19/p016, sighted in search)

The design our baseline copies: observation = current follower joint positions + 4
camera images, **no history of anything**; chunking is motivated explicitly against
"temporally correlated confounders, such as pauses in demonstrations that are hard to
model with Markovian single-step policies" — ACT's answer to this literature is to
*predict* action sequences rather than condition on them. Then the buried near-miss,
verbatim: "It is important to use the leader joint positions instead of the
follower's [as actions], because the amount of force applied is implicitly defined by
the difference between them, through the low-level PID controller." The paper thus
*knows* the force latent lives in the command-vs-achieved gap — and uses the fact
only on the **action** side (what to output), shipping a policy that cannot
**observe** the force it is applying, even though `a[t-1]` and `s[t]` are both in
every training log and their difference is never an input. Our delta arm is this
sentence moved across the equals sign, worth 5% → 57–63% on our task; that the
omission survives unchanged into ALOHA derivatives and LeRobot defaults is the
fleet-scale version of the thesis. Chunking also quietly throttles any copycat loop —
within a chunk the policy never sees its own outputs — which is part of why adding
`a[t-1]` at chunk boundaries is safe.
`[Autocorrelation: high | Latent observable: no — force explicitly named as latent in the a−s gap, then left unobserved | Shortcut: designed around (chunking), not measured]`

**For the thesis — the smoking gun:** the flagship low-cost-manipulation paper states
the missing observation's exact location and still ships the policy blind to it.

### 15. Oh, Liu, Tao, Han, Shaw, Funabashi, Salakhutdinov, Pathak — "FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning", arXiv:2606.12406, June 2026
<https://arxiv.org/abs/2606.12406> · <https://arxiv.org/html/2606.12406>

The closest existing work to the thesis. NEXT estimates external joint torque on
sensorless commodity arms (AgileX Piper, $2.5k; validated against Franka torque
sensors) as measured motor torque (from current) minus an LSTM-predicted free-motion
torque, where the LSTM input already includes the **commanded-vs-achieved tracking
error** `Δq_d` over a 50-step history; feeding `τ̂_ext` into BC observations plus
contact-phase re-sampling (FIRST) beats prior force-aware policies by >17% task
progress across five contact-rich tasks. They also find "policy failures concentrate
not in free-space motion, but in the brief pre-contact intervals requiring precise
alignment" — keyframe geometry again — and that raw motor current as a policy input
helps on only 1 of 2 ablation tasks while the learned estimate helps on both. What it
concedes to the thesis: force observability is the binding gap on cheap arms, and it
is recoverable from actuator signals the robot already produces. What it does *not*
claim, and we do: (i) on position-controlled hobby servos **no estimator is needed at
all** — raw `a[t-1]` in the observation suffices and the network learns the
subtraction; (ii) the channel is recoverable **retroactively from logs every LeRobot
dataset already ships** (NEXT needs current telemetry plus per-arm free-motion
calibration, both absent from existing corpora); (iii) any connection to the
copycat/causal-confusion literature — "history" appears here only as estimator input,
never as the contested observation. Between this paper and ACT's leader-position
sentence, our point exists in the literature as two disconnected halves; nobody has
joined them, and nobody has the causal instrumentation (our 14× zeroing) on the join.
`[Autocorrelation: high | Latent observable: no without the estimate — stated via failure concentration at pre-contact | Shortcut: not discussed; force-channel benefit measured]`

**For the thesis:** independent convergence on the mechanism, one venue-cycle ago;
the no-new-sensing, no-model, and retroactive-corpus claims remain ours — as does the
reconciliation with the copycat canon.

---

## Also sighted (fetched, not full entries)

- **Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion",
  RSS 2023 / extended.** <https://arxiv.org/abs/2303.04137> ·
  <https://ar5iv.labs.arxiv.org/html/2303.04137>. Read: uses a 2-step observation
  window as folk practice across tasks, notes policies "can easily overfit to this
  pausing behavior" (idle demo actions), tolerates ~4 steps of latency via
  receding-horizon position control; no shortcut analysis, no force-bearing channel.
  The truncate-context folk practice, canonized.
- **Mark, Liang, Attarian, Fu, Dwibedi, Shah, Kumar — "BPP: Long-Context Robot
  Imitation Learning by Focusing on Key History Frames", arXiv:2602.15010, Feb 2026.**
  [NOT READ — abstract only] <https://arxiv.org/abs/2602.15010>. Attributes latching
  to exponentially-shrinking *coverage* of history space and finds existing
  regularizers inconsistent; compresses history to VLM-selected keyframes. A fourth
  reconciliation mechanism (coverage) — consistent with why our 6-scalar, one-step
  history stays safe: its support is dense in 600 demos.
- **Shao, Kleine Buening, Kwiatkowska — "Causal Imitation Learning under
  Expert-Observable and Expert-Unobservable Confounding", ICLR 2026.**
  [NOT READ — abstract only] <https://arxiv.org/abs/2502.07656>. Unifies hidden-
  confounder IL settings and uses **trajectory histories as instruments** —
  histories as the *solution* to confounding, formalized; the theory door for
  treating delta as an instrumented measurement.

---

## Thread summary — the gap, and the experiment it implies

The literature turns out to hold **three named regimes and one unnamed one**. (1)
*Shortcut regime* (de Haan; Wen ×2; Chuang; Codevilla; ChauffeurNet): history or
self-state channels hurt when expert actions are smooth, the copyable part of the
channel is recoverable, and the channel carries little or nothing about a
task-necessary latent; the harm concentrates at rare changepoints, and its measured
signature is *validation loss down, rollout down, action-predictability above the
expert's*. Notably, even here the flagship result is not "history hurts": BC with
history beat single-frame in 5 of 6 of Wen et al.'s own environments — history
under-delivers by a shortcut tax. (2) *Latching/delusion regime* (Swamy-contexts;
Ortega): when identifying a hidden context requires integrating one's **own** past
actions, off-policy BC provably corrupts its posterior; the fix is on-policy data or
do-calculus. (3) *Memory regime* (PTP; BPP; robomimic-RNN): slow episodic latents
make history necessary, and modern diffusion policies actually **under-use** it
(predictability ratio < 1), fixed by auxiliary past-token losses or keyframe
compression. (4) The unnamed regime — ours: the channel is a **one-step physical
measurement of a fast exogenous latent**. `a[t-1]` differenced against the measured
`s[t]` is the servo loop's rejection of the command — external force — a latent that
is decision-critical at grasp keyframes, invisible to vision and position, and
deterministic from two numbers every fleet log already stores. Nobody parameterizes
the history question by *what the appended channel physically observes*; every
"history necessary" construction in the canon is an occlusion, a hidden velocity, or
an episodic fact, never an actuator latent. The two halves of our point exist,
disconnected: ACT states the latent's address on the action side ("force ... defined
by the difference between them") and ships a policy blind to it; FACTR 2 (June 2026)
puts an *estimated* force into BC observations on commodity arms — but needs current
telemetry plus calibration no existing corpus has, and never touches the history/
copycat question. No one has shown the raw previous command — the exact input the
canon warns against — acting as the force sensor, no one has the interventional
autopsy (our 14× zeroing), and no one has run the copycat probes on a case where
history wins 10×. Our TF-97%-everywhere / rollout-10× signature is the mirror image
of causal confusion's signature and is currently undocumented in the literature.

**The single experiment this implies — channel-content substitution (2×2 +
probes), all on existing infrastructure:** train five arms on the same recipe —
{base, +`a[t-1]`, +delta, +privileged ground-truth contact force from sim,
+`a[t-1]`+privileged force} — and evaluate each on the contact task *and* on a
no-contact reach variant; report rollout success, TF loss, Wen's action-
predictability ratio, and the zeroing counterfactual for every cell. The
observability thesis predicts an interaction: privileged force ≈ delta ≈ raw
`a[t-1]` on the contact task; `a[t-1]`'s marginal gain collapses to ~0 once
privileged force is present (it was a sensor, now redundant); all channels ≈ base on
the reach variant; predictability ratios stay at expert level throughout. The
copycat/latching account predicts the opposite: an `a[t-1]` gain that persists
additively under privileged force, appears even on the reach variant, and comes with
predictability > expert and a TF-loss drop. One sweep decides which regime we are in,
turns the reconciliation into a measured claim no current paper makes, and hands the
LeRobot PR and the corpus study their causal story.
