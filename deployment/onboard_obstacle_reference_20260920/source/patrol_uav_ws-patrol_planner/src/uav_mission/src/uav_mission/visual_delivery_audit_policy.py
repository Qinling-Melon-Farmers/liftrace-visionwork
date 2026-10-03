"""Pure evidence selection for the passive visual delivery audit."""


def resolve_audit_evidence(evidence, evidence_context,
                           require_evidence_context):
    """Return geometry evidence plus the identity a permission must carry.

    Legacy permissions identify the geometry evidence directly.  In strict
    mode the permission identifies the semantic mission target, while frame
    stability and the evidence stamp still belong to the nested geometry
    evidence.
    """
    if not require_evidence_context:
        if evidence is None:
            return None, "evidence_lock_missing"
        return {
            "geometry_evidence": evidence,
            "align_mode": evidence["align_mode"],
            "target_id": evidence["target_id"],
            "target_class": evidence.get("target_class", ""),
            "compare_target_class": False,
        }, ""

    if evidence_context is None:
        return None, "evidence_context_missing"
    if (not evidence_context.get("context_valid", False) or
            not evidence_context.get("context_active", False) or
            not evidence_context.get("has_semantic_target", False)):
        return None, "evidence_context_invalid"
    geometry_evidence = evidence_context.get("evidence")
    if geometry_evidence is None:
        return None, "evidence_lock_missing"
    if (not evidence_context.get("semantic_geometry_match", False) or
            not evidence_context.get("geometry_target_present", False) or
            not evidence_context.get("geometry_map_valid", False)):
        return None, "evidence_context_geometry_invalid"
    if (evidence_context.get("geometry_target_id") !=
            geometry_evidence.get("target_id") or
            evidence_context.get("geometry_target_class", "") !=
            geometry_evidence.get("target_class", "")):
        return None, "evidence_context_geometry_mismatch"
    align_mode = evidence_context.get("align_mode", "")
    if geometry_evidence.get("align_mode", "") != align_mode:
        return None, "evidence_context_geometry_mismatch"
    return {
        "geometry_evidence": geometry_evidence,
        "align_mode": align_mode,
        "target_id": evidence_context.get("semantic_target_id", 0),
        "target_class": evidence_context.get("semantic_target_class", ""),
        "compare_target_class": True,
    }, ""


def permission_matches_audit_view(permission, view):
    """Match a permission to the identity selected for the audit mode."""
    if (permission["target_id"] != view["target_id"] or
            permission["align_mode"] != view["align_mode"]):
        return False
    return (not view["compare_target_class"] or
            permission["target_class"] == view["target_class"])
