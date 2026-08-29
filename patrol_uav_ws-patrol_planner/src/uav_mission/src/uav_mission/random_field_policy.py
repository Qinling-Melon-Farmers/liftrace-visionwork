"""Pure validation and geometry helpers for the random-field spawner."""

from dataclasses import dataclass
import math


STANDARD_FOOTPRINT_RADIUS = math.sqrt(0.5 ** 2 + 0.5 ** 2)
RED_CROSS_FOOTPRINT_RADIUS = math.sqrt(0.175 ** 2 + 0.175 ** 2)


@dataclass(frozen=True)
class Footprint:
    name: str
    x: float
    y: float
    radius: float


def validate_seed(seed):
    seed = int(seed)
    if seed <= 0:
        raise ValueError("random field seed must be a fixed positive integer")
    return seed


def validate_bounds(bounds, label):
    values = tuple(float(value) for value in bounds)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("%s bounds must be finite" % label)
    min_x, max_x, min_y, max_y = values
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("%s bounds must have positive area" % label)
    return values


def validate_standard_classes(classes, required_classes):
    classes = tuple(classes)
    required_classes = tuple(required_classes)
    if len(classes) != len(set(classes)):
        raise ValueError("standard_classes contains duplicates")
    if set(classes) != set(required_classes):
        raise ValueError(
            "standard_classes %r do not match profile classes %r" %
            (classes, required_classes))
    # Canonical profile order keeps a seed reproducible even if a launch
    # supplies the same class set in a different textual order.
    return required_classes


def footprint_inside_bounds(x, y, radius, bounds, margin=0.0):
    min_x, max_x, min_y, max_y = bounds
    reserve = float(radius) + float(margin)
    return (min_x + reserve <= x <= max_x - reserve and
            min_y + reserve <= y <= max_y - reserve)


def footprint_clear(x, y, radius, occupied, gap=0.0):
    for item in occupied:
        required = float(radius) + float(item.radius) + float(gap)
        if math.hypot(x - item.x, y - item.y) < required:
            return False
    return True


def plan_footprint_layout(rng, targets, occupied, search_bounds,
                          field_bounds, boundary_margin=0.0, pair_gap=0.0,
                          offset_x=0.0, offset_y=0.0,
                          attempts_per_target=4000, layout_attempts=64):
    """Sample a complete layout, restarting instead of keeping a dead end.

    Sequential rejection sampling can place early targets so that a later
    target has no free pose even though a valid full layout exists.  This
    helper plans every footprint before Gazebo mutation and deterministically
    restarts the whole layout using the same seeded RNG when that happens.
    """
    attempts_per_target = int(attempts_per_target)
    layout_attempts = int(layout_attempts)
    if attempts_per_target <= 0 or layout_attempts <= 0:
        raise ValueError("layout attempt limits must be positive")
    target_specs = [(str(name), float(radius)) for name, radius in targets]
    if any(not math.isfinite(radius) or radius < 0.0
           for _name, radius in target_specs):
        raise ValueError("target footprint radii must be finite and nonnegative")

    for _layout_index in range(layout_attempts):
        trial_occupied = list(occupied)
        planned = []
        for name, radius in target_specs:
            sample = None
            for _attempt in range(attempts_per_target):
                local_x = rng.uniform(search_bounds[0], search_bounds[1])
                local_y = rng.uniform(search_bounds[2], search_bounds[3])
                if not footprint_inside_bounds(
                        local_x, local_y, radius, search_bounds,
                        boundary_margin):
                    continue
                if not footprint_inside_bounds(
                        local_x, local_y, radius, field_bounds,
                        boundary_margin):
                    continue
                world_x = local_x + float(offset_x)
                world_y = local_y + float(offset_y)
                if footprint_clear(
                        world_x, world_y, radius, trial_occupied, pair_gap):
                    sample = (local_x, local_y)
                    break
            if sample is None:
                break
            local_x, local_y = sample
            planned.append((name, local_x, local_y))
            trial_occupied.append(Footprint(
                name, local_x + float(offset_x),
                local_y + float(offset_y), radius))
        if len(planned) == len(target_specs):
            return planned
    return None
