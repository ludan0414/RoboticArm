"""ROS-free pose and motion-segment types for waypoint generation."""

from dataclasses import dataclass, field
from typing import List


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class MotionSegment:
    name: str = ""
    type: str = ""
    target_pose: Pose = field(default_factory=Pose)
    waypoints: List[Pose] = field(default_factory=list)
    keep_tool_down: bool = False
    keep_camera_down: bool = False
    lock_wrist: bool = False
    allow_ompl_fallback: bool = False
    # Legacy Cartesian coverage hint (wire compatibility). Incomplete paths
    # now trigger complete pose-goal replanning rather than partial execution.
    required_cartesian_fraction: float = 0.0
