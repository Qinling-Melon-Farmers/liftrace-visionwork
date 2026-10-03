"""Shared fail-fast contract for local inference model assets."""

import os


def require_local_model_file(model_path, backend_name):
    """Return an absolute model path or reject an unusable runtime asset."""
    if not isinstance(model_path, str) or not model_path.strip():
        raise ValueError(
            "%s model path is empty; provide the launch model argument"
            % backend_name)
    resolved = os.path.abspath(os.path.expanduser(model_path.strip()))
    if not os.path.isfile(resolved):
        raise ValueError(
            "%s model is not a regular file: %s"
            % (backend_name, resolved))
    return resolved
