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

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal

import numpy as np

from .scene import (
    FINGERTIP_BELOW_TCP,
    JAW_OPEN,
    JAW_SHUT,
    RAD_PER_TICK,
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
    grip_mode: Literal["clamp", "force", "oracle"] = "clamp"
    #: Force command for ``force`` mode: jaw travel past the detected contact
    #: point, in encoder counts. ~1.5 N per count at the pads.
    grip_overshoot_counts: float = 8.0
    #: Force command for ``oracle`` mode, in newtons of true contact force.
    grip_target_newtons: float = 15.0
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
    crushed: bool = False
    peak_grip_n: float = 0.0


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

    def close_until_contact(
        self,
        step_rad: float,
        stall_fraction: float = 0.3,
        max_frames: int = 200,
        warmup: int = 4,
    ) -> tuple[float, bool]:
        """Close the jaw until the servo stalls against the object.

        Contact is detected from the *measured* jaw motion, not from the
        tracking error: while the command advances ``step_rad`` per frame, a
        free jaw follows at very nearly that rate and a blocked one stops dead.
        Measured over a full close, the actual jaw moves 0.0100 rad/frame in
        free space against a 0.0100 rad/frame command, and 0.0001 rad/frame once
        the block stops it -- a hundredfold separation, with no calibration.

        Thresholding the tracking error instead is what failed. Its free-space
        value is not zero but a steady lag (-17 counts here), and it is not
        steady at the start of the move: over the first eight frames the servo is
        still accelerating and reads -6 rising to -17, so a baseline measured
        there underestimates the lag and any small margin above it triggers
        immediately. Brushing the block without gripping it moves the error only
        to -19..-21 at under 6 N, well inside that band, while a real stall runs
        to -38 and beyond. Both quantities are available on hardware; the
        measured velocity is simply the better-conditioned one.

        Returns:
            ``(jaw_angle, contact_found)``.
        """
        scene = self.scene
        # Read the QUANTISED joint reading, not `data.qpos`. The raw simulator
        # state is not available to a real robot, and using it here would make
        # the expert immune to `obs_quantum` -- silently defeating the one
        # ablation this environment exists to run.
        #
        # Motion is then integrated over a WINDOW rather than compared frame to
        # frame, and the window widens with the quantum. At 0.006 rad the jaw
        # advances 3.9 counts per frame, so a quantum of 8 counts makes most
        # single-frame differences read zero and a frame-to-frame test would
        # declare a stall immediately. That would manufacture the very effect
        # this environment is meant to measure. Integrating until the expected
        # travel is a small multiple of the quantum gives the detector the best
        # it can do at each resolution, so whatever degradation survives is a
        # property of the channel and not of a naive controller.
        step_counts = step_rad / RAD_PER_TICK
        window = max(3, int(np.ceil(2.0 * scene.obs_quantum / max(step_counts, 1e-9))))
        history: deque[float] = deque(maxlen=window + 1)
        history.append(float(scene.state_ticks()[5]))
        for k in range(max_frames):
            if self.jaw <= JAW_SHUT:
                return self.jaw, False
            self.jaw = max(JAW_SHUT, self.jaw - step_rad)
            scene.hold(np.concatenate([self.q, [self.jaw]]), hook=self.hook)
            history.append(float(scene.state_ticks()[5]))
            if k >= warmup and len(history) == window + 1:
                moved = abs(history[-1] - history[0])
                if moved < stall_fraction * window * step_counts:
                    return self.jaw, True
        return self.jaw, False

    def grip_by_overshoot(
        self,
        overshoot_counts: float,
        step_rad: float,
        settle: int = 15,
        ramp: int = 12,
    ) -> float:
        """Find contact, then squeeze a fixed number of encoder counts past it.

        This is the formulation that works, and it is the one available on
        hardware. Grip force is commanded as an integer count of jaw travel
        beyond the contact point, so it is set by a quantity the bus can actually
        represent: ``Goal_Position`` is an integer register, and the resolution
        of the commanded force is exactly the encoder quantum.

        The earlier formulation -- ramp at a fixed rate and stop when the
        tracking error reaches a target -- fails for a reason worth recording.
        Closing from ``JAW_OPEN`` to ``JAW_SHUT`` is 0.75 rad, and a 0.006 rad
        step over 120 frames covers only 0.72, so some episodes exhausted their
        travel before the threshold could fire while others stopped on a
        transient having touched nothing. Measured 8 picks in 20, with grip
        force scattered from 0 to 306 N. Detecting contact first and then
        commanding a displacement decouples "where is the object" from "how hard
        to squeeze", and only the second needs to be precise.

        Returns:
            Tracking error after the squeeze, in counts.
        """
        scene = self.scene
        _, found = self.close_until_contact(step_rad)
        if not found:
            return float(scene.delta()[5])
        return self._squeeze(self.jaw - overshoot_counts * RAD_PER_TICK, settle, ramp)

    def grip_to_true_force(
        self,
        target_newtons: float,
        step_rad: float,
        max_frames: int = 60,
        settle: int = 15,
    ) -> float:
        """Close until the TRUE contact force reaches ``target_newtons``.

        Privileged: reads the simulator's contact solver, which no real robot
        can. This is the upper bound on what any force controller could achieve
        on this task, so the gap to :meth:`grip_by_overshoot` is the price of
        having only a tracking-error proxy, and the gap from the open-loop clamp
        to this is the total value of force feedback here.
        """
        scene = self.scene
        self.close_until_contact(step_rad)
        for _ in range(max_frames):
            if scene.grip_force() >= target_newtons or self.jaw <= JAW_SHUT:
                break
            self.jaw = max(JAW_SHUT, self.jaw - step_rad * 0.25)
            scene.hold(np.concatenate([self.q, [self.jaw]]), hook=self.hook)
        scene.hold(np.concatenate([self.q, [self.jaw]]), settle, hook=self.hook)
        return float(scene.delta()[5])

    def _squeeze(self, target: float, settle: int, ramp: int) -> float:
        """Ramp the jaw to ``target`` and hold, returning the resulting error."""
        scene = self.scene
        target = max(JAW_SHUT, target)
        start = self.jaw
        for k in range(ramp):
            self.jaw = start + (target - start) * (k + 1) / ramp
            scene.hold(np.concatenate([self.q, [self.jaw]]), hook=self.hook)
        self.jaw = target
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

        # The three grips differ ONLY in what signal sets the squeeze, which is
        # what makes them a measurement rather than three implementations:
        #   clamp  -- no force feedback at all
        #   force  -- the quantised channel the real robot has
        #   oracle -- true contact force, privileged, an upper bound
        if cfg.grip_mode == "force":
            result.grip_counts = way.grip_by_overshoot(
                cfg.grip_overshoot_counts, cfg.grip_step_rad
            )
        elif cfg.grip_mode == "oracle":
            result.grip_counts = way.grip_to_true_force(
                cfg.grip_target_newtons, cfg.grip_step_rad
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

        # A crushed block is a lost episode however tidily it was delivered.
        result.crushed = scene.crushed
        result.peak_grip_n = scene.peak_grip_n
        result.placed = result.picked and scene.block_in_box() and not scene.crushed
        result.peak_rise_m = peak
        return result
