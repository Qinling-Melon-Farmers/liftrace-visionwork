"""Shared data types for the UAV search mission.

This module deliberately contains no ROS dependencies.  ROS messages are
converted to these internal types by the search manager node.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class MissionState(Enum):
    """High-level states used by the minimum search mission."""

    INIT = "INIT"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    TARGET_HOLD = "TARGET_HOLD"
    RESUME_SEARCH = "RESUME_SEARCH"
    SEARCH_COMPLETE = "SEARCH_COMPLETE"

# 大概这么用：
# sba = MissionState()
# sba.INIT
# sba.SEARCH,就是防止打错字的

@dataclass(frozen=True)
class Waypoint:
    """A position goal expressed in the mission map frame."""

    x: float
    y: float
    z: float

    def as_tuple(self) -> Tuple[float, float, float]:
        # -> 是一种注释，意思返回的都是float
        """Return the waypoint in the form expected by ``publish_goal``."""

        return self.x, self.y, self.z


@dataclass
class SearchContext:
    """Information required to resume an interrupted coverage search."""

    waypoint_index: int
    interrupted_goal: Optional[Waypoint]
    search_altitude: float
