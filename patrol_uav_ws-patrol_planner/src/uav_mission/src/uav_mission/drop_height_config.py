"""Independent FC AGL release window; omitted settings preserve legacy generation."""
import math


def release_heights(settings):
    """Return controller min/max and permission min/max, all FC AGL metres."""
    target = settings['drop_agl']
    if isinstance(target, bool) or not isinstance(target, (int, float)) or not math.isfinite(target):
        raise ValueError('drop_agl must be a finite number')
    explicit = ('release_min_agl' in settings, 'release_max_agl' in settings)
    if explicit[0] != explicit[1]:
        raise ValueError('release_min_agl and release_max_agl must be specified together')
    lower = settings.get('release_min_agl', target)
    upper = settings.get('release_max_agl', target + .10)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in (target, lower, upper)) or not 0 < lower <= target <= upper <= 1.1:
        raise ValueError('release heights require finite 0 < min <= drop_agl <= max <= 1.1')
    # Task permission is a broader preauthorization, not the final control gate.
    # Keep its original .27..47 tolerance when the control window is .35..45.
    permission_lower, permission_upper = ((lower - .08, upper + .02) if all(explicit)
                                          else (target - .08, target + .12))
    return lower, upper, permission_lower, permission_upper
