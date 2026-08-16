"""Scripted pick-and-place expert.

The expert is privileged about the *object* -- it reads the block pose from the
simulator, as a demonstrator is allowed to -- but everything it does with the
*gripper* uses only :meth:`~so101_bench.scene.PickScene.delta`, the servo
tracking error in encoder counts. No force sensor is involved, in simulation or
on hardware.

Measured success under the default domain randomisation is 70% pick and 55%
place over 40 episodes; see ``docs/findings.md`` for the failure analysis behind
each constant here.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal

import numpy as np

from .scene import (
    FINGERTIP_BELOW_TCP,
    JAW_OPEN,
    JAW_SHUT,
    TABLE_TOP,
    PickScene,
)

__all__ = ["ExpertConfig", "EpisodeResult", "ScriptedExpert", "HOME"]

#: Home configuration, matching the scene's ``home`` keyframe.
HOME = np.array([0.0, -1.57079, 1.57079, 1.57079, -1.57079])

Hook = Callable[[PickScene], None]


@dataclass(frozen=True)
class ExpertConfig:
    """Tunable parameters of the scripted policy.

    ``grasp_dz`` is the tool height at the grasp relative to the block centre.
    Deeper grasps trade a little pick rate for much better retention: at +4 mm
    the expert picks 15/20 and places 9; at 0 mm it picks 14/20 and places 11.

    ``grip_mode`` selects how the jaw closes. ``"clamp"`` drives it to a fixed
    angle and is the more reliable demonstrator (14/20 against 8/20), but the
    servo saturates and the force channel stops responding to the object.
    ``"force"`` regulates the grip through the quantised channel and is what a
    force-sensitive task needs.
    """

    approach_height: float = 0.075
    lift_height: float = 0.13
    grasp_dz: float = 0.0
    transport_frames: int = 55
    grip_mode: Literal["clamp", "force"] = "clamp"
    grip_target_counts: float = 12.0
    grip_step_rad: float = 0.006
    pick_threshold: float = 0.03
    #: Accept the aligned grasp heading when its IK residual is below this
    #: (metres); only beyond it is the 180-degree flip considered.
    yaw_fallback_tolerance: float = 0.004


@dataclass
class EpisodeResult:
    """Outcome and per-episode conditions, captured during the episode."""

    picked: bool = False
    placed: bool = False
    peak_rise_m: float = 0.0
    grip_counts: float = 0.0
    grip_force_n: float = 0.0
    block_x: float = 0.0
    block_y: float = 0.0
    block_yaw: float = 0.0
    grasp_yaw: float = 0.0
    ik_error_mm: float = 0.0
    block_half_width: float = 0.0
    block_half_length: float = 0.0
    block_mass: float = 0.0
    block_friction: float = 0.0


class _Waypoints:
    """Interpolating waypoint follower emitting joint-target commands."""

    def __init__(self, scene: PickScene, hook: Hook | None = None) -> None:
        self.scene = scene
        self.q = HOME.copy()
        self.jaw = JAW_OPEN
        self.hook = hook

    def move(
        self,
        target: np.ndarray,
        jaw: float | None = None,
        yaw: float | None = None,
        frames: int = 45,
        settle: int = 8,
    ) -> bool:
        scene = self.scene
        q1, converged = scene.solve_ik(target, self.q, yaw=yaw)
        j0 = self.jaw
        j1 = self.jaw if jaw is None else jaw
        q0 = self.q.copy()
        for k in range(frames):
            f = (k + 1) / frames
            scene.hold(
                np.concatenate([q0 + (q1 - q0) * f, [j0 + (j1 - j0) * f]]),
                hook=self.hook,
            )
        self.q, self.jaw = q1, j1
        if settle:
            scene.hold(np.concatenate([self.q, [self.jaw]]), settle, hook=self.hook)
        return converged

    def set_jaw(self, jaw: float, frames: int = 25, settle: int = 10) -> None:
        """Open-loop jaw move, used for releasing and for the clamp grasp."""
        scene, j0 = self.scene, self.jaw
        for k in range(frames):
            scene.hold(
                np.concatenate([self.q, [j0 + (jaw - j0) * (k + 1) / frames]]),
                hook=self.hook,
            )
        self.jaw = jaw
        scene.hold(np.concatenate([self.q, [self.jaw]]), settle, hook=self.hook)

    def close_to_force(
        self,
        target_counts: float,
        step_rad: float,
        frames: int = 120,
        settle: int = 12,
        baseline_frames: int = 10,
    ) -> float:
        """Close until tracking error exceeds its free-space lag by ``target_counts``.

        The baseline subtraction is essential. A proportionally controlled servo
        that is *moving* sits behind its goal whether or not it is touching
        anything: stepping the jaw 0.006 rad per frame is 3.9 counts of command
        travel, so the tracking error reads several counts in free space.
        Thresholding the raw value halts the close almost immediately -- measured
        1 pick in 15, with the jaw stopping at 1-6 counts and no contact at all.
        Only the excess over the free-running lag is load. The same confound
        exists on hardware.

        Returns:
            The tracking error achieved, in counts. It is quantised, so the grip
            is commandable only to the resolution of the encoder.
        """
        scene = self.scene
        lag: list[float] = []
        for k in range(frames):
            if self.jaw <= JAW_SHUT:
                break
            if k >= baseline_frames:
                baseline = float(np.median(lag)) if lag else 0.0
                if abs(scene.delta()[5]) - baseline >= target_counts:
                    break
            self.jaw = max(JAW_SHUT, self.jaw - step_rad)
            scene.hold(np.concatenate([self.q, [self.jaw]]), hook=self.hook)
            if k < baseline_frames:
                lag.append(abs(scene.delta()[5]))
        scene.hold(np.concatenate([self.q, [self.jaw]]), settle, hook=self.hook)
        return float(scene.delta()[5])


class ScriptedExpert:
    """Runs scripted pick-and-place episodes on a :class:`PickScene`."""

    def __init__(self, scene: PickScene, config: ExpertConfig | None = None) -> None:
        self.scene = scene
        self.config = config or ExpertConfig()

    def choose_grasp_yaw(self, block: np.ndarray, yaw: float) -> tuple[float, float]:
        """Select the heading for closing across the block's short side.

        ``yaw + pi`` is used only as a FALLBACK, never as an equally-ranked
        alternative. A rectangular block's short axis is indeed unchanged by a
        180 degree rotation, but this gripper's tool frame is not symmetric about
        it: the fixed jaw's fingertip sits 11.9 mm to one side of the tool centre
        along the closing axis, so flipping by pi mirrors the pads about the tool
        and puts the block on the wrong side of the jaw. Ranking the two purely
        on IK residual therefore picks the flipped heading whenever it is
        marginally easier to reach and then fails to grasp -- on the nominal
        scene it won with a 0.0009 mm residual against 0.88 mm and lifted the
        block 1.3 mm.

        Near-miss headings are not offered at all. The same fingertip clears the
        block only while its half-width across the closing axis stays under
        11.9 mm, and 14 degrees of misalignment on a 25 mm long axis raises the
        effective half-width to 16.5 mm; the tip then lands on the block and
        presses it into the table. IK residual cannot see any of that.

        Returns:
            ``(heading, ik_residual_metres)``.
        """
        target = block + [0, 0, 0.004]

        def residual(candidate: float) -> float:
            q, _ = self.scene.solve_ik(block + [0, 0, 0.075], HOME, yaw=candidate)
            q, _ = self.scene.solve_ik(target, q, yaw=candidate)
            return float(np.linalg.norm(self.scene.tcp_at(q) - target))

        aligned = residual(yaw)
        if aligned <= self.config.yaw_fallback_tolerance:
            return yaw, aligned
        flipped = residual(yaw + np.pi)
        if flipped < aligned:
            return yaw + np.pi, flipped
        return yaw, aligned

    def run(self, hook: Hook | None = None) -> EpisodeResult:
        """Reset the scene and run one full episode."""
        scene, cfg = self.scene, self.config
        scene.reset()
        block = scene.block_pos()
        z0 = float(block[2])
        grasp_yaw, ik_err = self.choose_grasp_yaw(block, scene.block_yaw())

        model = scene.model
        result = EpisodeResult(
            block_x=float(block[0]),
            block_y=float(block[1]),
            block_yaw=scene.block_yaw(),
            grasp_yaw=grasp_yaw,
            ik_error_mm=1000.0 * ik_err,
            block_half_width=float(model.geom_size[scene.ids.block_geom][0]),
            block_half_length=float(model.geom_size[scene.ids.block_geom][1]),
            block_mass=float(model.body_mass[scene.ids.block]),
            block_friction=float(model.geom_friction[scene.ids.block_geom][0]),
        )

        peak = 0.0

        def track(s: PickScene) -> None:
            nonlocal peak
            peak = max(peak, float(s.block_pos()[2]) - z0)
            if hook is not None:
                hook(s)

        way = _Waypoints(scene, track)

        # Descend as deep as the fingertip allows rather than to a fixed offset
        # from the block centre: the tip is 8.8 mm below the tool, and keeping it
        # 2 mm clear of the table seats the block as far into the jaw as geometry
        # permits. Grasps that survived transport held the block within 3-12 mm
        # of the tool; every one that failed sat at 14-23 mm and was shed.
        grasp_z = max(TABLE_TOP + FINGERTIP_BELOW_TCP + 0.002, block[2] + cfg.grasp_dz)
        way.move(block + [0, 0, cfg.approach_height], jaw=JAW_OPEN, yaw=grasp_yaw, frames=55)
        way.move(
            np.array([block[0], block[1], grasp_z]),
            jaw=JAW_OPEN,
            yaw=grasp_yaw,
            frames=45,
        )

        if cfg.grip_mode == "force":
            result.grip_counts = way.close_to_force(
                cfg.grip_target_counts, cfg.grip_step_rad
            )
        else:
            way.set_jaw(JAW_SHUT)
            result.grip_counts = float(scene.delta()[5])
        result.grip_force_n = scene.grip_force()

        way.move(
            scene.block_pos() * [1, 1, 0] + [0, 0, z0 + cfg.lift_height],
            yaw=grasp_yaw,
            frames=45,
        )
        result.picked = peak > cfg.pick_threshold

        # The container walls top out at z = 0.145. Releasing with the tool at
        # box + 0.085 puts the block's underside above them, falling 6 cm, and it
        # bounces back out; drop it from just inside the rim instead.
        box = scene.box_pos()
        way.move(box + [0, 0, 0.16], frames=cfg.transport_frames)
        way.move(box + [0, 0, 0.05], frames=40)
        way.set_jaw(JAW_OPEN, frames=20)
        way.move(box + [0, 0, 0.18], frames=35)

        result.placed = result.picked and scene.block_in_box()
        result.peak_rise_m = peak
        return result
