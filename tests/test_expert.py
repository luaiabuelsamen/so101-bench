"""Scripted expert behaviour and recording.

The success-rate test is a regression guard with a deliberately loose bound: the
expert measures 70% pick / 55% place over 40 randomised episodes, and the point
here is to catch a collapse (a broken tool frame, a flipped jaw axis), not to
pin the exact figure.
"""

from __future__ import annotations

import numpy as np
import pytest

from so101_bench import DemoEnv, DomainRandomization, ExpertConfig, PickScene
from so101_bench.expert import HOME, ScriptedExpert


@pytest.fixture(scope="module")
def nominal_expert() -> ScriptedExpert:
    return ScriptedExpert(PickScene(seed=0, randomise=DomainRandomization.off()))


def test_expert_solves_the_nominal_scene(nominal_expert: ScriptedExpert) -> None:
    result = nominal_expert.run()
    assert result.picked
    assert result.placed
    assert result.peak_rise_m > 0.05


def test_grasp_yaw_prefers_an_aligned_heading() -> None:
    scene = PickScene(seed=0, randomise=DomainRandomization.off())
    expert = ScriptedExpert(scene)
    block = scene.reset(block_xy=(0.10, -0.25), block_yaw=0.4)
    heading, _ = expert.choose_grasp_yaw(block, 0.4)
    offset = (heading - 0.4) % np.pi
    assert min(offset, np.pi - offset) < 1e-6


@pytest.mark.slow
def test_success_rate_under_randomisation() -> None:
    env = DemoEnv(seed=0, cameras=())
    results = env.evaluate(20)
    picked = sum(r.picked for r in results)
    placed = sum(r.placed for r in results)
    assert picked >= 10, f"pick rate collapsed: {picked}/20"
    assert placed >= 6, f"place rate collapsed: {placed}/20"


def test_force_mode_produces_a_graded_grip() -> None:
    """Clamping saturates the channel; force mode must not."""
    clamp = DemoEnv(seed=0, cameras=(), expert=ExpertConfig(grip_mode="clamp"))
    force = DemoEnv(seed=0, cameras=(), expert=ExpertConfig(grip_mode="force"))
    clamp_force = np.array([r.grip_force_n for r in clamp.evaluate(6)])
    graded = np.array([r.grip_force_n for r in force.evaluate(6)])
    assert graded.max() < clamp_force.max()


def test_recorder_captures_state_before_action() -> None:
    env = DemoEnv(seed=0, cameras=(), stride=1)
    _, frames = env.rollout()
    assert len(frames) > 50
    states = np.stack(frames.states)
    actions = np.stack(frames.actions)
    assert states.shape[1] == 6
    assert actions.shape == states.shape
    assert np.allclose(states, np.round(states))
    assert np.allclose(actions, np.round(actions))


def test_recorder_stride_subsamples() -> None:
    dense = DemoEnv(seed=0, cameras=(), stride=1)
    sparse = DemoEnv(seed=0, cameras=(), stride=2)
    _, a = dense.rollout()
    _, b = sparse.rollout()
    assert len(b) == pytest.approx(len(a) / 2, abs=1)


def test_delta_recoverable_from_recorded_frames() -> None:
    """`action[t-1] - state[t]` must be reconstructable exactly as in real data."""
    env = DemoEnv(seed=0, cameras=(), stride=1)
    _, frames = env.rollout()
    states = np.stack(frames.states)
    actions = np.stack(frames.actions)
    delta = actions[:-1] - states[1:]
    assert np.isfinite(delta).all()
    assert len(np.unique(np.round(delta[:, 5]))) > 3


def test_rollout_without_recording_leaves_no_recorder_attached() -> None:
    env = DemoEnv(seed=0, cameras=())
    env.rollout(record=False)
    assert env.scene.recorder is None


def test_home_matches_the_scene_keyframe() -> None:
    scene = PickScene(seed=0, randomise=False)
    scene.reset()
    assert scene.model.key("home").ctrl[:5] == pytest.approx(HOME, abs=1e-4)


def test_crush_limit_makes_force_feedback_necessary() -> None:
    """The positive control: with a crush limit, the open-loop clamp must fail.

    Rigid-block pick-and-place does not need force feedback (clamp 23/30 beats
    both regulated experts). A crush limit inverts that -- the clamp's 356 N
    median peak grip destroys the block every time, while the delta-regulated
    grip survives. If this ever stops holding, the task has stopped being
    force-sensitive and the resolution study built on it is meaningless.
    """
    clamp = DemoEnv(
        seed=0, cameras=(), crush_newtons=120.0, expert=ExpertConfig(grip_mode="clamp")
    ).evaluate(10)
    force = DemoEnv(
        seed=0,
        cameras=(),
        crush_newtons=120.0,
        expert=ExpertConfig(grip_mode="force", grip_overshoot_counts=4),
    ).evaluate(10)
    assert sum(r.placed for r in clamp) == 0, "clamp should crush every block"
    assert sum(r.crushed for r in clamp) >= 7
    assert sum(r.placed for r in force) >= 3, "regulated grip should survive"


def test_coarse_quantum_degrades_the_regulated_grip() -> None:
    """Resolution must matter once the quantum exceeds the task's margin."""
    fine = DemoEnv(
        seed=0,
        cameras=(),
        crush_newtons=100.0,
        obs_quantum=1.0,
        expert=ExpertConfig(grip_mode="force", grip_overshoot_counts=4),
    ).evaluate(12)
    coarse = DemoEnv(
        seed=0,
        cameras=(),
        crush_newtons=100.0,
        obs_quantum=16.0,
        expert=ExpertConfig(grip_mode="force", grip_overshoot_counts=4),
    ).evaluate(12)
    assert sum(r.placed for r in fine) > sum(r.placed for r in coarse)


def test_crush_latch_clears_on_reset() -> None:
    env = DemoEnv(seed=0, cameras=(), crush_newtons=50.0,
                  expert=ExpertConfig(grip_mode="clamp"))
    env.rollout(record=False)
    assert env.scene.crushed
    env.scene.reset()
    assert not env.scene.crushed
    assert env.scene.peak_grip_n == 0.0
