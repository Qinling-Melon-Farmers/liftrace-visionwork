"""Pure validation for optional external planner-map feature anchors."""

import math
import os
import xml.etree.ElementTree as ET


def validate_anchor_profile(profile_name, profiles):
    if profile_name not in profiles:
        raise ValueError(
            "unknown planner anchor profile %r (expected one of: %s)" %
            (profile_name, ", ".join(sorted(profiles))))
    profile = profiles[profile_name]
    anchors = profile.get("anchors") or []
    names = set()
    normalized = []
    for item in anchors:
        for key in ("name", "model", "pose"):
            if key not in item:
                raise ValueError("anchor is missing %s: %r" % (key, item))
        name = str(item["name"])
        if name in names:
            raise ValueError("duplicate anchor name %s" % name)
        names.add(name)
        pose = tuple(float(value) for value in item["pose"])
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
            raise ValueError("anchor pose must contain six finite values")
        normalized.append({
            "name": name,
            "model": str(item["model"]),
            "pose": pose,
        })
    return {
        "source_revision": str(profile.get("source_revision", "")),
        "external_feature_dependency": bool(
            profile.get("external_feature_dependency", False)),
        "anchors": normalized,
    }


def resolve_model_sdf(model_name, model_roots):
    """Resolve standard model.sdf or the SDF named by model.config."""
    checked = []
    for root in model_roots:
        model_dir = os.path.join(root, model_name)
        standard = os.path.join(model_dir, "model.sdf")
        checked.append(standard)
        if os.path.isfile(standard):
            return standard
        config_path = os.path.join(model_dir, "model.config")
        if os.path.isfile(config_path):
            try:
                sdf_element = ET.parse(config_path).getroot().find("sdf")
            except (ET.ParseError, OSError) as exc:
                raise ValueError(
                    "invalid model.config for %s: %s" %
                    (model_name, exc))
            if sdf_element is not None and sdf_element.text:
                configured = os.path.join(
                    model_dir, sdf_element.text.strip())
                checked.append(configured)
                if os.path.isfile(configured):
                    return configured
        named = os.path.join(model_dir, "%s.sdf" % model_name)
        checked.append(named)
        if os.path.isfile(named):
            return named
    raise ValueError(
        "model SDF for %s not found; checked=%r" % (model_name, checked))
