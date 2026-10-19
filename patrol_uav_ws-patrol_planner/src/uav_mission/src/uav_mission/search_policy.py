import math

from .search_types import Waypoint
from .search_types import SearchContext


class SearchPolicy:
    def __init__(
            self,
            min_x,
            max_x,
            min_y,
            max_y,
            lane_spacing,
            altitude
    ):
        self.min_x = float(min_x)
        self.min_y = float(min_y)
        self.max_x = float(max_x)
        self.max_y = float(max_y)
        self.lane_spacing = float(lane_spacing)
        self.altitude = float(altitude)

        self._validate_parameters()

        self._waypoints = self._generate_waypoints()
        self._current_index = 0
        # _current_index, 既表示正在飞往的航点的序号（从0开始），
        # 又表示已经完成的航点的数量

    def _validate_parameters(self):
        values = (
            self.min_x ,
            self.min_y ,
            self.max_x ,
            self.max_y ,
            self.lane_spacing, 
            self.altitude 
            )

        if not all(math.isfinite(value) for value in values):
            raise ValueError("Search parameters must be finite")

        if self.min_x >= self.max_x:
            raise ValueError("min_x must be smaller than max_x")

        if self.min_y >= self.max_y:
            raise ValueError("min_y must be smaller than max_y")

        if self.lane_spacing <= 0.0:
            raise ValueError("lane_spacing must be positive")

        if self.altitude <= 0.0:
            raise ValueError("altitude must be positive")
        if self.altitude > 4.0:
            raise ValueError("altitude exceeds the competition limit")

    def _generate_waypoints(self):
        waypoints = []
        y = self.min_y
        left_to_right = True

        while True:
            if left_to_right:
                start_x = self.min_x
                end_x = self.max_x
            else:
                start_x = self.max_x
                end_x = self.min_x

            waypoints.append(Waypoint(start_x, y, self.altitude))
            waypoints.append(Waypoint(end_x, y, self.altitude))

            if y >= self.max_y:
                break

            y = min(y + self.lane_spacing, self.max_y)
            left_to_right = not left_to_right

        return waypoints

    @property
    def waypoints(self):
        return tuple(self._waypoints)

    @property
    def current_index(self):
        return self._current_index

    @property
    def current_waypoint(self):
        if self._current_index >= len(self._waypoints):
            return None
        return self._waypoints[self._current_index]

    @property
    def is_complete(self):
        return self._current_index >= len(self._waypoints)

    @property
    def coverage_ratio(self):
        if not self._waypoints:
            return 1.0
    
        ratio = self._current_index / len(self._waypoints)
    
        return min(ratio, 1.0)

    def advance(self):
        if self._current_index < len(self._waypoints):
            self._current_index += 1
            return self.current_waypoint
        else:
            return None

    def restore(self, index):
        if not isinstance(index, int):
            raise TypeError("index MUST be an integer")
        if index < 0 or index > len(self._waypoints):
            # index, 既表示正在飞往的航点的序号（从0开始），
            # 又表示已经完成的航点的数量
            raise ValueError("index is outside the range")

        self._current_index = index
        return self.current_waypoint
        # 要是index == N，那么就意味着飞向第N点（从0开始计数），
        # 意味着完成了N个航点，此时会返回None
