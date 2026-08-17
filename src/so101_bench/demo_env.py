"""Demonstration environment: scene, scripted expert and dataset recording.

    from so101_bench import DemoEnv
    env = DemoEnv(seed=0)
    env.collect(200, root="~/data/pick_place")

One class so that collection, evaluation and policy rollout share a scene, a
randomisation stream and an observation construction. Recorded per control frame:

======================================  ==========================================
``observation.state``                   6 joints, encoder counts, read *before*
                                        the command
``action``                              6 joint goals, encoder counts, integral
``observation.images.<camera>``         one entry per requested camera
``grip_force_N``                        true contact force -- analysis only
======================================  ==========================================

``delta = action[t-1] - state[t]`` is therefore recoverable downstream and is
constructed identically to the public SO-100/101 datasets. Contact force is never
an observation, because the robot cannot observe it; the ``grip_force_N`` column
is ground truth for validating the delta channel and must not be fed to a policy.
"""

from __future__ import annotations

import logging
import shutil
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .expert import EpisodeResult, ExpertConfig, ScriptedExpert
from .scene import DomainRandomization, PickScene

__all__ = ["DemoEnv", "FrameRecorder", "CollectionReport"]

log = logging.getLogger(__name__)

DEFAULT_TASK = "pick and place the block in the box"


@dataclass
class CollectionReport:
    root: str
    kept: int
    attempted: int

    @property
    def yield_fraction(self) -> float:
        return self.kept / self.attempted if self.attempted else 0.0


class FrameRecorder:
    """Accumulates one episode of frames. Rendering dominates the cost."""

    def __init__(self, cameras: Sequence[str] = (), stride: int = 1) -> None:
        self.cameras = tuple(cameras)
        self.stride = max(1, stride)
        self.reset()

    def reset(self) -> None:
        self.states: list[np.ndarray] = []
        self.actions: list[np.ndarray] = []
        self.forces: list[np.float32] = []
        self.images: dict[str, list[np.ndarray]] = {c: [] for c in self.cameras}
        self._seen = 0

    def append(self, state: np.ndarray, action: np.ndarray, scene: PickScene) -> None:
        self._seen += 1
        if (self._seen - 1) % self.stride:
            return
        self.states.append(np.asarray(state, dtype=np.float32).copy())
        self.actions.append(np.asarray(action, dtype=np.float32).copy())
        self.forces.append(np.float32(scene.grip_force()))
        for camera in self.cameras:
            self.images[camera].append(scene.image(camera))

    def __len__(self) -> int:
        return len(self.states)


class DemoEnv:
    """Pick-and-place demonstration environment.

    Args:
        seed: randomisation seed.
        randomise: domain randomisation ranges, or ``False`` to disable.
        cameras: scene cameras to record; empty disables rendering entirely.
        image_size: square render size in pixels.
        stride: record every ``stride``-th control frame.
        expert: expert configuration.
        quantise, obs_quantum, crush_newtons: see
            :class:`~so101_bench.scene.PickScene`.
    """

    def __init__(
        self,
        seed: int = 0,
        randomise: DomainRandomization | bool | None = None,
        cameras: Sequence[str] = ("front", "gripper_fpv"),
        image_size: int = 128,
        stride: int = 2,
        expert: ExpertConfig | None = None,
        quantise: bool = True,
        obs_quantum: float = 1.0,
        crush_newtons: float | None = None,
    ) -> None:
        self.cameras = tuple(cameras)
        self.image_size = image_size
        self.scene = PickScene(
            seed=seed,
            randomise=randomise,
            render=bool(self.cameras),
            width=image_size,
            height=image_size,
            quantise=quantise,
            obs_quantum=obs_quantum,
            crush_newtons=crush_newtons,
        )
        self.expert = ScriptedExpert(self.scene, expert)
        self.recorder = FrameRecorder(self.cameras, stride=stride)

    def rollout(
        self, record: bool = True, hook=None
    ) -> tuple[EpisodeResult, FrameRecorder]:
        """Run one scripted episode.

        ``hook(scene)`` is called every control frame and MAY perturb the
        simulator state -- this is the injection point for disturbance
        (recovery-data) collection: kicking the physics leaves the recorded
        actions as clean supervision while the states show deviations and the
        expert's closed-loop corrections.
        """
        self.recorder.reset()
        self.scene.recorder = self.recorder if record else None
        try:
            result = self.expert.run(hook=hook)
        finally:
            self.scene.recorder = None
        return result, self.recorder

    def evaluate(self, episodes: int) -> list[EpisodeResult]:
        """Run episodes without recording frames. Useful for success rates."""
        return [self.rollout(record=False)[0] for _ in range(episodes)]

    def collect(
        self,
        episodes: int,
        root: str | Path,
        only_success: bool = True,
        task: str = DEFAULT_TASK,
        fps: int = 30,
        overwrite: bool = True,
        max_attempts: int | None = None,
        repo_id: str = "so101_bench/pick_place",
        hook=None,
        per_episode=None,
    ) -> CollectionReport:
        """Write ``episodes`` demonstrations as a LeRobotDataset.

        Args:
            only_success: keep only episodes that ended with the block in the
                box. Note this biases the dataset toward easier block poses.
            max_attempts: give up after this many episodes; ``None`` means keep
                going until ``episodes`` are kept.
        """
        # lerobot validates video codec options even when use_videos=False, and
        # its integer check calls .is_integer() on a plain int.
        import lerobot.datasets as _lerobot_datasets

        _lerobot_datasets.check_video_encoder_parameters_pyav = lambda *a, **k: None
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        root = Path(root).expanduser()
        if overwrite and root.exists():
            shutil.rmtree(root)

        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=fps,
            features=self._features(),
            root=str(root),
            robot_type="so101",
            use_videos=False,
        )

        kept = attempted = 0
        while kept < episodes:
            if max_attempts is not None and attempted >= max_attempts:
                log.warning("stopping after %d attempts with %d kept", attempted, kept)
                break
            if per_episode is not None:
                per_episode(self)      # e.g. resample the expert's grip target
            result, frames = self.rollout(hook=hook)
            attempted += 1
            if only_success and not result.placed:
                continue
            self._write_episode(dataset, frames, task)
            kept += 1
            if kept % 10 == 0:
                log.info("%d/%d kept, %d attempted", kept, episodes, attempted)

        # Flush the writer's buffered episode metadata. Without this, datasets
        # whose episode count never crosses the metadata buffer threshold have
        # an empty meta/episodes, and LeRobotDataset then falls back to a hub
        # lookup that 404s -- small smoke datasets hit this every time.
        dataset.finalize()
        report = CollectionReport(str(root), kept, attempted)
        log.info(
            "wrote %s: %d episodes from %d attempts (%.0f%% yield)",
            report.root,
            report.kept,
            report.attempted,
            100 * report.yield_fraction,
        )
        return report

    def _features(self) -> dict:
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (6,),
                "names": [f"joint_{i}" for i in range(6)],
            },
            "action": {
                "dtype": "float32",
                "shape": (6,),
                "names": [f"joint_{i}" for i in range(6)],
            },
            "grip_force_N": {
                "dtype": "float32",
                "shape": (1,),
                "names": ["grip_force_N"],
            },
        }
        for camera in self.cameras:
            features[f"observation.images.{camera}"] = {
                "dtype": "image",
                "shape": (self.image_size, self.image_size, 3),
                "names": ["height", "width", "channel"],
            }
        return features

    def _write_episode(self, dataset, frames: FrameRecorder, task: str) -> None:
        for i in range(len(frames)):
            frame = {
                "observation.state": frames.states[i],
                "action": frames.actions[i],
                "grip_force_N": np.array([frames.forces[i]], dtype=np.float32),
                "task": task,
            }
            for camera in self.cameras:
                frame[f"observation.images.{camera}"] = frames.images[camera][i]
            dataset.add_frame(frame)
        dataset.save_episode()

    def close(self) -> None:
        self.scene.close()

    def __enter__(self) -> DemoEnv:
        return self

    def __exit__(self, *exc) -> None:
        self.close()
