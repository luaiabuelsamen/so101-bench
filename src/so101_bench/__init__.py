"""Force-aware manipulation benchmark for the low-cost SO-100/SO-101 arm.

A MuJoCo pick-and-place environment with domain randomisation, a scripted
expert, and LeRobot-schema data collection -- built around the proprioceptive
force channel that a position-controlled hobby servo exposes through its own
tracking error.
"""

from .demo_env import CollectionReport, DemoEnv, FrameRecorder
from .expert import EpisodeResult, ExpertConfig, ScriptedExpert
from .scene import DomainRandomization, PickScene

__version__ = "0.1.0"

__all__ = [
    "CollectionReport",
    "DemoEnv",
    "DomainRandomization",
    "EpisodeResult",
    "ExpertConfig",
    "FrameRecorder",
    "PickScene",
    "ScriptedExpert",
    "__version__",
]
