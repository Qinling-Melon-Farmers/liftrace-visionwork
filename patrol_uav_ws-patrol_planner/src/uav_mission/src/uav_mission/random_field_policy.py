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
