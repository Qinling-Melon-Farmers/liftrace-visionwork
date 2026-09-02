"""Small, ROS-independent contract helpers for pixel-chain video replay."""


REQUIRED_RAW_SOURCES = frozenset(
    {
        "target_detector",
        "circle_detector",
        "cross_detector",
        "landing_detector",
    }
)


def frame_is_complete(frame_state, required_sources=REQUIRED_RAW_SOURCES,
                      require_mapped_perf=False):
    """Return true once one source frame has traversed every required stage.

    The historical PT replay remains pixel-only by default.  Board/RKNN
    replay opts into the mapped/perf evidence without changing any ROS message
    or production topic contract.
    """
    if not frame_state or frame_state.get("image") is None:
        return False
    raw_sources = set(frame_state.get("raw", {}))
    pixel_complete = (
        set(required_sources).issubset(raw_sources)
        and frame_state.get("resolved_search") is not None
        and frame_state.get("resolved_landing") is not None
        and frame_state.get("refined") is not None
    )
    if not pixel_complete:
        return False
    if not require_mapped_perf:
        return True
    return (
        frame_state.get("mapped") is not None
        and frame_state.get("perf") is not None
    )


def mapped_message_is_fail_closed(message):
    """True when a mapped heartbeat contains no valid map projection."""
    if message is None:
        return False
    return all(not bool(detection.map_valid)
               for detection in getattr(message, "detections", ()))


def validate_perf_status(message, expected_name="", expected_backend=""):
    """Return the selected detector backend or raise on degraded evidence."""
    statuses = list(getattr(message, "status", ()) or ())
    if expected_name:
        statuses = [status for status in statuses
                    if getattr(status, "name", "") == expected_name]
    if not statuses:
        raise ValueError("detector perf status is missing")

    status = statuses[0]
    if int(getattr(status, "level", -1)) != 0:
        raise ValueError("detector perf status is not OK")
    values = {
        str(getattr(item, "key", "")): str(getattr(item, "value", ""))
        for item in (getattr(status, "values", ()) or ())
    }
    backend = values.get("backend", str(getattr(status, "message", "")))
    if not backend:
        raise ValueError("detector perf backend is missing")
    if expected_backend and backend != expected_backend:
        raise ValueError(
            "detector perf backend %s does not match %s" %
            (backend, expected_backend))
    return backend


def validate_video_metadata(metadata):
    """Validate the fields needed to preserve source geometry and playback time."""
    if not isinstance(metadata, dict):
        raise ValueError("video metadata must be a JSON object")
    width = int(metadata.get("width", 0))
    height = int(metadata.get("height", 0))
    fps = float(metadata.get("fps", 0.0))
    if width <= 0 or height <= 0:
        raise ValueError("video metadata has invalid dimensions")
    if not (fps > 0.0):
        raise ValueError("video metadata has invalid fps")
    return width, height, fps
