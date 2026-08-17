# Thread B — Sensorless force/torque estimation (esp. low-cost actuators)

Reading notes, 2026-08-16. Read against the thesis: *"On low-cost robots, the binding
constraint on learned manipulation is observability — not data scale, not model scale —
and the missing observations are already latent in the actuators and logs the fleet ships with."*

Our result under test: on simulated SO-101 (Feetech STS3215), delta = goal − present is a
calibrated force channel (2.00 N/count seated & quasi-static; held-out RMSE 4.3 N over
4–78 N; non-informative during sliding; invalid during fast transport). Claimed
differentiators: (i) retroactivity from the public LeRobot corpus (positions+goals only),
(ii) a 30-line model-free runtime guard, (iii) a resolution-vs-safety-margin design rule.
Obvious live baseline: the STS3215's Present_Load / Present_Current registers.

Every entry below lists the URL actually fetched. "PDF, pp. X–Y read" means I read those
pages of the paper itself; "HTML/abs fetched" means the arXiv HTML or abstract page was
fetched and summarized. One entry is marked [NOT READ — abstract only].

---

## 1. The momentum-residual / observer line (the canon)

### 1.1 De Luca & Mattone, "Sensorless Robot Collision Detection and Hybrid Force/Motion Control," ICRA 2005
URL fetched: http://www.diag.uniroma1.it/~deluca/pHRI_elective/ICRA05_ADL_Mattone_SensorlessCollisionDetection.pdf (PDF, pp. 1–2 read)

Origin paper of the momentum residual for collisions: a collision is treated as an actuator
fault, and the residual r = K[∫(α − τ − r)dt + p] is a first-order-filtered estimate of the
contact joint torque, computable from commanded torque τ, positions/velocities (q, q̇), and
the full dynamic model — explicitly no acceleration and no force sensor. They note that the
"intuitive" alternative — comparing applied torque with the nominal model-based command and
looking for transients — suffers from hard-to-tune thresholds. Validation is simulation on a
two-link planar arm; friction is essentially ignored (rigid model), and sliding is handled
by *switching controllers after* detection, not by the residual itself. Requires commanded
torque as input, which a position-servo hobby arm does not expose; position tracking error
is never the signal.

**Verdict: Orthogonal to the thesis** (torque-commanded robots, model-based); establishes
the residual-guard architecture our 30-line guard is a degenerate, model-free case of.

### 1.2 Haddadin, De Luca & Albu-Schäffer, "Robot Collisions: A Survey on Detection, Isolation, and Identification," IEEE T-RO 33(6), 2017
URL fetched: http://www.diag.uniroma1.it/~labrob/pub/papers/TRO_Collision_Dec2017.pdf (PDF, pp. 1295–1298 and 1301–1302 read)

The definitive taxonomy: energy observer, direct estimation (needs q̈, impractical),
inverse-dynamics monitoring for stiff position-controlled robots (compare feedforward torque
τ̂_m,ff against applied motor torque τ_m), velocity observer, and the momentum observer —
all requiring motor torque/current plus a dynamic model. Two things matter for us. First,
for flexible joints they formalize deflection-as-torque: τ_J = K_J(θ − q), the elastic
torque read from the motor–link displacement — exactly the physics our delta channel
borrows, except our "spring" is the servo's position loop (τ ≈ Kp·delta) rather than a
physical elastic element with torque sensing. Second, their threshold doctrine (Sec. IV) is
ε_μ = μ_max + ε_safe: an *empirical disturbance envelope* (friction, current noise, model
error) plus a robustness margin — a noise-floor rule for limiting false positives, not a
rule connecting sensor resolution to the *task's* force safety margin. Position tracking
error of a position-controlled joint never appears in the monitoring-signal taxonomy; no
retroactive-from-logs discussion anywhere.

**Verdict: For the thesis in mechanism (deflection→torque is textbook), Against novelty of
"guard = thresholded residual"; leaves differentiators (i) and (iii) untouched.**

### 1.3 Garofalo, Mansfeld, Jankowski & Ott, "Sliding Mode Momentum Observers for Estimation of External Torques and Joint Acceleration," DLR (ICRA 2019)
URL fetched: https://elib.dlr.de/129060/1/root.pdf (PDF, pp. 1–4 read)

Upgrades the classic first-order momentum observer to super-twisting sliding-mode variants
with finite-time convergence to τ_ext. States the classic observer's two standing
assumptions baldly: (1) the dynamic model is perfectly known, (2) friction is negligible or
known — precisely the assumptions a $30 gearmotor violates. Quantifies the
bandwidth-vs-noise tradeoff (K_O of 20–80 Hz on DLR/KUKA-class hardware, noise amplification
∝ K_O·M) and reiterates that thresholds must be set empirically to suppress false positives.
Torque-level accuracy plots are in Nm on torque-sensed lightweight robots; nothing about
low-cost actuators, position error, or logs.

**Verdict: Orthogonal** — high-end refinement of the observer line; useful as the "what the
canon assumes that hobby servos break" citation.

### 1.4 Zhang, Zhao, Zhang & Liu, "Disturbance Recognition and Collision Detection of Manipulator Based on Momentum Observer," Sensors 20(15), 2020
URL fetched: https://pmc.ncbi.nlm.nih.gov/articles/PMC7435754/ (full text fetched)

The observer canon transplanted to a self-built low-cost 3-joint collaborative arm with
motor-current sensing (no torque sensors; 19-bit output encoders). They bolt a
GA-identified LuGre friction model onto a generalized-momentum observer; friction is
explicitly "the main component" of the disturbance. Even after compensation, residual
friction RMSE is 0.697–1.02 Nm per joint and the collision threshold lands at 7.80 Nm
(max normal-operation disturbance + margin) — i.e., on cheap gearing, the *friction floor
sets the detection floor*, which at their link lengths is tens of newtons at the tool. This
is the strongest observer-line datapoint that our ~4 N-class error on a $110 arm is not
embarrassing: their heavily-modeled current-based pipeline on better encoders yields a
usable threshold in the same force decade. Inputs are (q, q̇, current) only; tracking error
unused; nothing retroactive.

**Verdict: For the thesis** (friction floor, not sensing absence, is the real constraint —
and current+model doesn't beat it by much on cheap hardware); Against nothing of ours.

---

## 2. Virtual force/torque sensors on cheap actuators

### 2.1 Yen, Tang, Lin & Lin, "Development of a Virtual Force Sensor for a Low-Cost Collaborative Robot and Applications to Safety Control," Sensors 19(11):2603, 2019
URL fetched: https://pmc.ncbi.nlm.nih.gov/articles/PMC6612363/ (full text fetched; www.mdpi.com returned 403)

A US$5,000 7-axis arm (BLDC + harmonic drives, one current sensor and 24-pulse Hall sensing
per joint) gets a current-based contact-force observer with per-axis experimentally
calibrated friction. Reported accuracy: ~10% mean error in static conditions, ~15% during
multi-axis dynamic force control, over a stated useful range of 20–80 N — i.e., roughly
2–8 N absolute, on hardware a tier above hobby servos. Two statements are gold for us:
the observer "cannot detect an external torque that was less than the static friction
torque" (an explicit stiction floor), and the safety application is a fixed 35 N threshold
taken from ISO 15066 (lowest body-region limit), with stiffness/torque saturation as the
reaction — a regulatory threshold, not a resolution-vs-margin design rule. Quasi-static
assumptions are baked in ("dynamic equation calculation can be simplified or ignored").
No position-error channel (their position error appears only inside impedance-control force
*generation*), no retroactivity.

**Verdict: For the thesis** (stiction-floor framing; current-based accuracy on low-cost
gearing is ~5–10% of range, matching ours); the nearest prior for our design rule but stops
at "use the ISO number."

### 2.2 Hwang, Minami & Ishikawa, "Virtual Torque Sensor for Low-Cost RC Servo Motors Based on Dynamic System Identification Utilizing Parametric Constraints," Sensors 18(11):3856, 2018
URL fetched: https://pmc.ncbi.nlm.nih.gov/articles/PMC6263914/ (full text fetched)

**The closest prior on Question 1.** A sealed hobby RC servo (GWS S03T, ~0.78 Nm) has an
unknown internal controller; users cannot access motor voltage or current. Their estimator
uses *only the reference angle θr and measured angle θ* — the servo's tracking pair — via
v(t) = Cr(s)θr − Cy(s)θ with the unknown controller transfer functions eliminated by two
system-ID experiments with different known inertias; torque then follows from identified
closed-loop models (their Eq. 12). So torque-from-(command, position) on a hobby servo was
published in 2018, and once identified, it runs on *recorded* command+position logs — the
retroactivity property exists here in embryo, though they never make the point (no corpus,
no fleet, single bench servo). Critically: validation is qualitative agreement against a
load cell under 0.1–10 Hz multisine excitation with fixed inertia loads — **no RMSE, no
force range, no contact tasks, no friction/backlash analysis, no validity envelope**.

**Verdict: Against differentiator "delta is the force signal" as a *concept* — this must be
cited or a reviewer will find it. For the thesis overall (the signal is latent in the
cheapest servos). Our surviving edge over it: one-parameter physical calibration (2.00
N/count) instead of per-unit two-inertia system ID, quantified accuracy over a 20× force
range, an explicit validity envelope (seated/quasi-static vs sliding vs transport), and the
corpus-scale retroactivity claim.**

---

## 3. Deflection-as-force: the SEA connection

### 3.1 Williamson, "Series Elastic Actuators," MIT MS thesis (with G. Pratt), Feb 1995
URL fetched: https://groups.csail.mit.edu/lbr/hrg/1995/mattw_ms_thesis.pdf (PDF, title + abstract pages read)

The founding SEA document (companion to Pratt & Williamson, IROS 1995, DOI
10.1109/IROS.1995.525827 — [NOT READ — abstract only]): put a spring of known stiffness
between geared motor and load, measure its deflection, and force sensing/control comes from
F = k·x, trading closed-loop bandwidth for clean, stable, shock-tolerant force control —
"Stiffness isn't everything." The design premise is that current-through-gearbox is a bad
force sensor (friction, backlash, reflected inertia), so *displacement across a compliance*
is the trustworthy signal. Our channel is exactly this with the servo's PD position loop as
the spring: delta is the deflection, Kp the stiffness, and our "invalid during fast
transport" is the SEA bandwidth tradeoff resurfacing. The SEA literature never frames a
stock position servo's tracking error as the elastic element, because SEAs add a *physical*
spring precisely to control the stiffness value.

**Verdict: For the thesis** — 30 years of SEA practice validate deflection-as-force on
geared drives; we should present delta as a "virtual SEA" and inherit its theory, which also
predicts our failure modes.

### 3.2 Tregear, Aktas & Rodriguez y Baena, "Investigating the Effect of a Series Elastic Actuation Retrofit to Black-Box Actuators," arXiv:2605.24127, 2026
URL fetched: https://arxiv.org/abs/2605.24127 (abs fetched)

Retrofits a torsional series-elastic module (FE-analyzed stiffness 2155.4 Nm/rad, ~£25) onto
a black-box actuator whose backlash and static friction otherwise preclude clean force
control, tripling force-control bandwidth (10.3→30.3 Hz) and beating a commercial sensor's
performance by 7.63%. The premise is identical to ours — black-box position actuators hide
their internals, so measure a deflection — but their answer is *add hardware*, while ours is
*read the deflection the controller already maintains*. They do not discuss the internal PD
loop as a virtual spring. Useful as the "hardware-added" contrast that makes the
retroactivity differentiator legible: a physical retrofit can never be applied to logs
already collected.

**Verdict: For the thesis** (same diagnosis, hardware remedy); no threat to any
differentiator.

### 3.3 Zhu, Hao et al. (Yale), "Forces for free: Vision-based contact force estimation with a compliant hand," Science Robotics, June 2025 — [NOT READ — abstract only]
URLs fetched: https://zenodo.org/records/15453923 (dataset record fetched; confirms authors/date); paper at https://www.science.org/doi/10.1126/scirobotics.adq5046 (403) and PubMed (cookie-walled) — content known only from search-result abstract text.

A wrist camera watches a deliberately compliant 3D-printed hand and maps finger deformation
to contact force, 0.2–0.4 N error, with the hand optimized to minimize friction/hysteresis
and estimator memory added to fight the partial observability friction causes. The title
*is* our thesis in vision form: force observability extracted "for free" from sensors the
robot already ships with, with elastic deflection as the underlying physics. Differences:
purpose-built compliant hardware (not stock), a trained vision estimator (not a calibration
constant), and nothing retroactive about existing datasets.

**Verdict: For the thesis** (independent "forces are latent in what you already have"
framing at a top venue — good company, and worth citing as convergent evidence); does not
touch hobby-servo delta.

---

## 4. Low-cost teleop and learned estimators (the competition)

### 4.1 Yamane, Li, Konosu, Inami, Oaki, Tsuji & Sakaino, "Design and Experimental Validation of Sensorless 4-Channel Bilateral Teleoperation for Low-Cost Manipulators," arXiv:2507.06174, 2025
URLs fetched: https://arxiv.org/abs/2507.06174 and https://arxiv.org/html/2507.06174v1

Sensorless force-reflecting teleop on CRANE-X7 ($2.5k, Dynamixel XM430/XM540 — a tier above
Feetech): a combined velocity/external-force observer built from encoder position, reference
torque, and identified nonlinear dynamics, with no current measurement. They are blunt about
the low-cost pathology list — slow control cycles, insufficient encoder resolution,
significant backlash — and attack it with careful dynamics identification rather than better
hardware. No standalone force-accuracy number is reported (0.516 Nm free-motion torque error
is control tracking, not estimation accuracy); tracking error per se is not the signal, and
nothing is retroactive (the observer needs their identified model and torque-mode control).

**Verdict: For the thesis** (force feedback extracted from proprioception on cheap arms
improves teleop); Against nothing — but shows the observer route requires torque-commandable
servos, which STS3215 position mode is not.

### 4.2 Yang, Acar, Xu, Deguet, Kazanzides & Wu, "An Effectiveness Study Across Baseline and Learning-based Force Estimation Methods on the da Vinci Research Kit Si System," arXiv:2405.07453, 2024 (Hamlyn Symposium)
URLs fetched: https://arxiv.org/abs/2405.07453 and https://arxiv.org/html/2405.07453v1

The benchmark yardstick: on dVRK-Si, an LSTM over free-space joint angles/velocities
predicts nominal torques whose residual (vs measured joint torque) gives force; RMSE 2.16 N
over a ±41.5 N range (5.27%), vs 0.96 N / 3.07% on dVRK Classic. Degradation on the Si is
attributed to poor PID control and missing gravity compensation — i.e., *controller quality
is a force-sensing parameter*, which is our world in miniature. Baselines are torque-sensor
manipulations, not current-from-scratch; position tracking error is not an input; nothing
retroactive.

**Verdict: Orthogonal-to-For.** Key use: our 4.3 N RMSE over 4–78 N (~5.8% of range) sits
between dVRK-Si (5.27%) and Yen's low-cost cobot (~10%) — sensorless force on
non-ideal hardware lands at 3–10% of range across three independent labs, so our number is
in-family, achieved with zero model and zero training.

### 4.3 Dou, Onyemelukwe, Zhang et al. (MIT/Amazon), "NeuralActuator: Neural Actuation Modeling for Robot Dynamics and External Force Perception," RSS 2026 (Outstanding Systems Paper), arXiv:2607.11734
URLs fetched: https://arxiv.org/html/2607.11734v1 (twice, targeted questions) and https://frank-zy-dou.github.io/projects/NeuralActuator/index.html

**The most dangerous prior paper.** A Transformer over histories of commanded targets,
proprioception, *tracking error*, and actuator telemetry, with multi-task heads for
generalized effort, 3D external force, contact probability, and motor condition — evaluated
on OpenManipulator-X (Dynamixel), Franka, and **SO-101 with STS3215**, i.e., our exact
actuator. SO-101 force MAE is 0.47–0.73 N on 300–500 g payload and pick-place tasks
(0.36–0.57 N against a force sensor on OMX; sub-50 g payloads are "close to the noise
floor") — an order better than our RMSE, albeit on small force ranges, with quasi-static
supervision assumptions and no sliding/dynamic-regime analysis. The saving facts, verified
by targeted fetch: on SO-101 it consumes the **signed Present_Load register** as the effort
input ("the raw current registers are not used"), it **cannot run from positions alone**, no
telemetry-free ablation exists, and no claim is made about position-only public corpora —
training needed a fixture-mounted 6-axis F/T sensor and a 94.5-minute purpose-built dataset.
Tracking error appears *as one feature among many*, never isolated, never calibrated, never
given a validity envelope.

**Verdict: For the thesis at large (its entire premise is that actuator telemetry encodes
force), Against our novelty locally.** It kills any claim that "force sensing on
STS3215 without sensors" is new; it does NOT kill (i) corpus retroactivity (LeRobot logs
lack load registers), (ii) the zero-training guard, or (iii) the resolution rule. Every
writeup of ours must cite it and position delta as the telemetry-free, calibration-not-
learning, envelope-honest floor of this space.

### 4.4 Oh, Liu, Tao, Han, Shaw, Funabashi, Salakhutdinov & Pathak (CMU), "FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning," arXiv:2606.12406, 2026
URL fetched: https://arxiv.org/html/2606.12406

NEXT (Neural External Torque Estimation): an LSTM over a 50-step history of **joint
position, velocity, and tracking error**, plus motor current converted to torque via the
manufacturer torque constant, trained in 1 minute on 10 minutes of *contact-free* motion —
labels come free because free space implies zero external torque; deployment residuals then
read out contact. Accuracy: 0.547 ± 0.348 Nm L1 on Franka contact; 0.018 Nm free-space on
the $2,500 AgileX Piper; robot-specific, and dependent on the torque constant (they
recommend F/T calibration if K is off). FIRST then uses the force signal to upsample
task-critical moments, beating force-aware baselines by 17%+ — direct evidence for
"observability, not data scale, is binding." Requires current telemetry at inference; no
retroactive-corpus claim; no resolution/safety rule; no quasi-static/sliding envelope.

**Verdict: For the thesis (their motivation section is nearly our thesis verbatim: commodity
arms + latent force + policy gains), Against solo novelty of tracking-error-as-input — it is
already a named feature here. Differentiators (i) and (iii) survive; the free-motion
self-labeling trick is one we could steal for corpus-scale calibration.**

### 4.5 Liu, Li, Shaw, Tao, Salakhutdinov & Pathak, "FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning," arXiv:2502.17432, 2025
URL fetched: https://arxiv.org/abs/2502.17432 (abs fetched)

The predecessor: low-cost bilateral teleop relaying follower external forces to the leader
arm, plus a curriculum that corrupts vision so the transformer policy learns to *attend* to
force, improving unseen-object generalization by 43% and policy performance by 40% over
force-as-input-without-curriculum. The abstract does not specify the actuators or the force
source (details live in the paper); its role in these notes is the policy-side claim: force
channels only pay off if the learner is forced to use them.

**Verdict: For the thesis** — observability gains require training care to cash out; a
caution for our "just add delta to the policy input" ambitions.

---

## Answers to the thread questions

**(1) Has anyone used position-tracking-error as the force signal on hobby-class servos?**
Yes, in embryo: Hwang/Minami/Ishikawa (Sensors 2018) estimate torque from (θr, θ) alone on a
sealed RC servo via identified transfer functions — filtered tracking error is the signal.
NeuralActuator and FACTR 2 both feed tracking error into learned estimators as *one feature*
(alongside load registers / current, which they require). Nobody publishes raw delta as a
*calibrated linear* force channel (N/count) with a quantified accuracy and validity envelope
on Feetech-class servos. The concept is prior art (cite Hwang); the calibration, envelope,
and scale are not.

**(2) Has anyone made the retroactivity point — force recoverable from already-recorded logs?**
No one states it. Hwang's estimator *could* run on logged (command, position) pairs after
per-unit two-inertia ID, but the paper never notices; NeuralActuator needs load registers
and FACTR 2 needs current, both absent from the public LeRobot corpus; observer methods need
torque commands and models. The claim "the existing public corpus already contains a force
channel" is, on this reading, ours alone. Differentiator (i) survives — strongest of the three.

**(3) What accuracy do current/observer methods achieve on comparable actuators — does 4.3 N RMSE compete?**
On honest low-cost hardware: Yen 2019, ~10% static / 15% dynamic over 20–80 N (current +
friction calibration, $5k cobot, stiction floor stated); Zhang 2020, 0.7–1.0 Nm residual
friction and a 7.8 Nm threshold after LuGre compensation; dVRK-Si LSTM, 2.16 N over ±41 N
(5.27%). Our 4.3 N over 4–78 N (~5.8% of range) is squarely in the 3–10%-of-range band the
field achieves with current sensing plus models or training — competitive *given zero
telemetry and zero training*. It is NOT competitive with learned estimators using richer
telemetry on the same actuator (NeuralActuator: 0.36–0.73 N MAE on SO-101) or added
hardware (SEA retrofit; vision+compliance at 0.2–0.4 N). Sell deployability and
retroactivity, never raw accuracy.

**(4) Does anyone state a resolution-vs-task-margin rule?**
No. The closest artifacts are Haddadin's threshold doctrine (threshold = empirical
disturbance max + safety margin — a false-positive rule about the *sensor's* floor) and
Yen's adoption of the ISO 15066 35 N limit (a regulatory constant). Nobody couples achievable
sensing resolution to the *task's* required force margin as a design rule. Differentiator
(iii) survives, and the two artifacts above are exactly the citations to frame it against.

---

## Summary — what survives, what dies, what to fear

**Survives strongest: (i) retroactivity.** No paper claims force recoverable from
positions+goals already sitting in public teleop corpora. The two 2026 heavyweights that
could have claimed it (NeuralActuator, FACTR 2) both hard-require telemetry — load registers
or current — that LeRobot-era logs never recorded, and neither ablates it away. Hwang 2018
is the one honest asterisk: their RC-servo estimator consumes only (command, position) and
would run on logs, so our framing must be "first to *make and validate* the corpus-scale
retroactivity claim," not "first estimator that could."

**Survives: (iii) the resolution-vs-safety-margin rule.** The field's two existing rules —
empirical noise-floor thresholds (Haddadin) and regulatory constants (Yen/ISO 15066) — are
both sensor-side. A rule that couples counts-per-newton to task force budgets has no prior
statement in anything read here.

**Half-dies: (ii) the 30-line guard.** Thresholded-residual guards are the standard
architecture since De Luca 2005; what remains ours is the degenerate-inputs version — no
model, no current, no training, stock firmware — which is a systems/minimalism contribution
and must be framed as such. **Also half-dead: any novelty claim on "tracking error encodes
force on hobby servos"** — Hwang 2018 published the concept; NeuralActuator and FACTR 2 use
the feature. What remains: the 2.00 N/count *calibration* (delta as a physical virtual-SEA
deflection, not a learned feature), the quantified 4–78 N envelope, and the explicit
failure taxonomy (sliding = non-informative, transport = invalid) that no prior work states.

**The single most dangerous prior paper: NeuralActuator (RSS 2026, arXiv:2607.11734).**
Same actuator (STS3215), same platform (SO-101), better absolute accuracy (0.47–0.73 N MAE),
tracking error already among its inputs, an Outstanding Systems Paper award, and a released
dataset — a reviewer will wave it at every claim we make. Its two verified soft spots are
our whole case: it cannot run without the load-register telemetry the public corpus lacks,
and it needed a force-sensor-instrumented 94-minute dataset to train what our 1-parameter
calibration gets for free. Cite it in paragraph one and define ourselves against it.
Runner-up threat: FACTR 2 (arXiv:2606.12406), which owns the "force sensing improves policy
learning on commodity arms" story and names tracking error as an input; its free-motion
self-labeling trick is also the cheapest path to calibrating delta across a heterogeneous
fleet.

**Net for the thesis:** the 2025–2026 wave (NeuralActuator, FACTR 1/2, Forces-for-free,
sensorless bilateral teleop) is independently converging on "force observability is latent
in hardware you already have, and it is the binding constraint on manipulation learning."
The thesis is not contrarian anymore — it is early-consensus. Our defensible territory
inside that consensus is the corpus-retroactive, telemetry-free, calibrated-and-enveloped
floor of the space, plus the design rule nobody has written down.
