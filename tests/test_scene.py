"""Scene, kinematics and force-channel invariants.

These tests pin the facts that were expensive to establish and are easy to break
silently: where the tool frame is, which axis the jaws close along, and that the
observation and command are integer encoder counts.
"""

from __future__ import annotations

import numpy as np
import pytest

from so101_bench.scene import (
    JAW_OPEN,
    JAW_SHUT,
    RAD_PER_TICK,
    TABLE_TOP,
    DomainRandomization,
    PickScene,
)


@pytest.fixture(scope="module")
def scene() -> PickScene:
    return PickScene(seed=0, randomise=DomainRandomization.off())


def test_scene_loads_with_expected_actuators(scene: PickScene) -> None:
    names = [scene.model.actuator(i).name for i in range(scene.model.nu)]
    assert names == ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]


def test_elliptic_friction_cone_is_configured(scene: PickScene) -> None:
    """A pyramidal cone at impratio 1 lets grasped objects slide out."""
    assert scene.model.opt.impratio >= 10.0


def test_finger_pads_exist(scene: PickScene) -> None:
    """Grasping depends on explicit pad geoms, not convexified visual meshes."""
    for side in ("fixed", "moving"):
        for i in range(1, 5):
            assert scene.model.geom(f"{side}_jaw_pad_{i}").id >= 0


def test_reset_places_block_on_the_table(scene: PickScene) -> None:
    block = scene.reset(block_xy=(0.10, -0.25), block_yaw=0.0)
    half_height = scene.model.geom_size[scene.ids.block_geom][2]
    assert block[2] == pytest.approx(TABLE_TOP + half_height, abs=2e-3)
    assert block[:2] == pytest.approx([0.10, -0.25], abs=1e-3)


def test_reset_is_deterministic_for_a_seed() -> None:
    a = PickScene(seed=7).reset()
    b = PickScene(seed=7).reset()
    assert a == pytest.approx(b)


def test_randomisation_respects_the_graspable_width_cap() -> None:
    """The fixed fingertip strikes anything wider than 11.9 mm from above."""
    s = PickScene(seed=3)
    for _ in range(30):
        s.reset()
        assert s.model.geom_size[s.ids.block_geom][0] <= s.dr.max_half_width + 1e-9


def test_ik_reaches_the_block(scene: PickScene) -> None:
    block = scene.reset(block_xy=(0.10, -0.25), block_yaw=0.0)
    q, converged = scene.solve_ik(block + [0, 0, 0.05], np.zeros(5))
    assert converged
    err = np.linalg.norm(scene.tcp_at(q) - (block + [0, 0, 0.05]))
    assert err < 2e-3


@pytest.mark.parametrize("heading", [0.0, 0.4, 0.7, 1.2, -0.7])
def test_ik_yaw_aims_the_closing_axis_not_the_z_axis(
    scene: PickScene, heading: float
) -> None:
    """The jaws separate along the jaw body's local X, so that is what yaw aims.

    ``xmat`` is a live view into ``MjData``; it must be copied before the
    caller's state is restored or the assertion reads the restored pose instead.
    """
    import mujoco

    block = scene.reset(block_xy=(0.10, -0.25), block_yaw=0.0)
    q, _ = scene.solve_ik(block + [0, 0, 0.05], np.zeros(5), yaw=heading)

    saved = scene.data.qpos.copy()
    scene.data.qpos[:5] = q
    mujoco.mj_kinematics(scene.model, scene.data)
    closing_axis = scene.data.xmat[scene.ids.jaw].reshape(3, 3)[:, 0].copy()
    scene.data.qpos[:] = saved
    mujoco.mj_forward(scene.model, scene.data)

    want = np.array([np.cos(heading), np.sin(heading), 0.0])
    assert abs(float(closing_axis @ want)) > 0.95


def test_tool_frame_sits_between_the_pads(scene: PickScene) -> None:
    """The tool is the jaw gap, not the URDF's nominal TCP 71 mm below it."""
    import mujoco

    scene.reset()
    scene.data.qpos[5] = JAW_OPEN
    mujoco.mj_kinematics(scene.model, scene.data)
    tcp = scene.tcp()
    fixed = scene.data.geom_xpos[scene.model.geom("fixed_jaw_pad_3").id]
    moving = scene.data.geom_xpos[scene.model.geom("moving_jaw_pad_3").id]
    assert np.linalg.norm(tcp - 0.5 * (fixed + moving)) < 0.03


def test_state_and_action_are_integer_counts(scene: PickScene) -> None:
    scene.reset()
    scene.hold(np.array([0.1, -1.5, 1.5, 1.5, -1.5, JAW_OPEN]), frames=3)
    assert np.allclose(scene.state_ticks(), np.round(scene.state_ticks()))
    assert np.allclose(scene.action_ticks(), np.round(scene.action_ticks()))


def test_obs_quantum_coarsens_the_observation() -> None:
    s = PickScene(seed=0, randomise=False, obs_quantum=8.0)
    s.reset()
    s.hold(np.array([0.1, -1.5, 1.5, 1.5, -1.5, JAW_OPEN]), frames=3)
    ticks = s.state_ticks()
    assert np.allclose(ticks % 8.0, 0.0)


def test_quantise_off_leaves_state_continuous() -> None:
    s = PickScene(seed=0, randomise=False, quantise=False)
    s.reset()
    s.hold(np.array([0.1, -1.5, 1.5, 1.5, -1.5, JAW_OPEN]), frames=3)
    assert not np.allclose(s.state_ticks(), np.round(s.state_ticks()))


def test_delta_is_action_minus_state(scene: PickScene) -> None:
    scene.reset()
    ctrl = np.array([0.1, -1.5, 1.5, 1.5, -1.5, JAW_OPEN])
    scene.hold(ctrl, frames=2)
    assert scene.delta() == pytest.approx(scene.action_ticks() - scene.state_ticks())


def test_delta_grows_when_the_jaw_is_blocked() -> None:
    """Closing onto the block loads the servo, which shows up as tracking error.

    The grasp pose must be warm-started from the hover solution. Seeding the IK
    from ``zeros(5)`` leaves a 47 mm residual at grasp height -- the arm simply
    does not get there -- and the jaw would then close on empty air.
    """
    from so101_bench.expert import HOME

    s = PickScene(seed=0, randomise=False)
    block = s.reset(block_xy=(0.10, -0.25), block_yaw=0.0)
    grasp = np.array([block[0], block[1], TABLE_TOP + 0.0108])

    hover, _ = s.solve_ik(block + [0, 0, 0.075], HOME, yaw=0.0)
    q, _ = s.solve_ik(grasp, hover, yaw=0.0)
    assert np.linalg.norm(s.tcp_at(q) - grasp) < 3e-3

    # Ramp to each pose. Commanding the grasp configuration directly from home
    # makes the arm swing through the block and knock it clear, after which the
    # jaw closes on empty air and the test measures nothing.
    for start, end in ((HOME, hover), (hover, q)):
        for k in range(45):
            f = (k + 1) / 45
            s.hold(np.concatenate([start + (end - start) * f, [JAW_OPEN]]))
    s.hold(np.concatenate([q, [JAW_OPEN]]), frames=10)

    free = abs(s.delta()[5])
    assert s.grip_force() == 0.0

    for k in range(25):
        f = (k + 1) / 25
        s.hold(np.concatenate([q, [JAW_OPEN + (JAW_SHUT - JAW_OPEN) * f]]))
    s.hold(np.concatenate([q, [JAW_SHUT]]), frames=15)

    assert abs(s.delta()[5]) > free + 5
    assert s.grip_force() > 0.0


def test_grip_force_is_zero_without_contact(scene: PickScene) -> None:
    scene.reset()
    scene.hold(np.array([0.0, -1.57, 1.57, 1.57, -1.57, JAW_OPEN]), frames=5)
    assert scene.grip_force() == 0.0


def test_rad_per_tick_matches_a_4096_count_encoder() -> None:
    assert RAD_PER_TICK == pytest.approx(2 * np.pi / 4096)
