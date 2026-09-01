"""Small, ROS-independent contract helpers for pixel-chain video replay."""


REQUIRED_RAW_SOURCES = frozenset(
    {
        "target_detector",
        "circle_detector",
        "cross_detector",
        "landing_detector",
    }
)


def frame_is_complete(frame_state, required_sources=REQUIRED_RAW_SOURCES):
    """Return true once one source frame has traversed every pixel-only stage."""
    if not frame_state or frame_state.get("image") is None:
        return False
    raw_sources = set(frame_state.get("raw", {}))
    return (
        set(required_sources).issubset(raw_sources)
        and frame_state.get("resolved_search") is not None
        and frame_state.get("resolved_landing") is not None
        and frame_state.get("refined") is not None
    )


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
