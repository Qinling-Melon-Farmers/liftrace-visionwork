import math
# make sure that points are valid, the select them

class CandidatePolicy:
    def __init__(self, minimum_state=2):
        self.minimum_state = minimum_state
        self._known_target_ids = set()

    def is_valid(self, candidate):
        if not candidate.map_valid:
            return False

        if candidate.state < self.minimum_state:
            return False

        if not candidate.class_name.strip():
            return False
        # 需要先过这三道筛
        point = candidate.map_point
        coordinates = (point.x, point.y, point.z)

        return all(math.isfinite(value) for value in coordinates)
        # 返回所有非无穷值？不是，all里面是个bool数组，all true了all输出true

    def is_new(self, candidate):
        return candidate.id not in self._known_target_ids

    def accept(self, candidate):
        if self.is_valid(candidate) and self.is_new(candidate):
            self._known_target_ids.add(candidate.id)
            return True
        return False

    @property
    def known_target_ids(self):
        return frozenset(self._known_target_ids)
    # _known_target_ids（带下划线）：这是规矩，
    # 告诉其他程序员“这个变量是私有的，别直接碰，
    # 要通过上面那个 known_target_ids 来读”。