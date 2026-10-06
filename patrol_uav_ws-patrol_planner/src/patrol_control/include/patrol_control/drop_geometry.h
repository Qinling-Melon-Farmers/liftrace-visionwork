#ifndef PATROL_CONTROL_DROP_GEOMETRY_H_
#define PATROL_CONTROL_DROP_GEOMETRY_H_
#include <cmath>

namespace patrol_control {
// Exposure/receipt freshness must both hold. Receipt cannot refresh old pixels.
inline bool dropObservationFresh(double stamp, double now, double max_age) {
    return std::isfinite(stamp) && std::isfinite(now) &&
           std::isfinite(max_age) && max_age > 0 && stamp > 0 &&
           now >= stamp && now-stamp <= max_age;
}
inline bool exactDropPointValid(double x, double y, double z, double ground,
                                double quality, double error) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           std::isfinite(ground) && std::abs(z-ground) <= 1e-4 &&
           std::isfinite(quality) && quality > 0 && quality <= 1 &&
           std::isfinite(error) && error >= 0;
}
inline bool exactDropAligned(double target_x, double target_y,
                             double body_x, double body_y, double limit) {
    return std::isfinite(target_x) && std::isfinite(target_y) &&
           std::isfinite(body_x) && std::isfinite(body_y) &&
           std::isfinite(limit) && limit > 0 &&
           std::hypot(target_x-body_x, target_y-body_y) <= limit;
}

// Immutable action identity, independent of heartbeat/evidence timestamps and
// authorization revision. A geometry snapshot is not a new release authority.
template<class Context>
inline bool sameDropAction(const Context& a, const Context& b) {
    return a.mission_id == b.mission_id && a.decision_seq == b.decision_seq &&
           a.attempt == b.attempt && a.payload_slot == b.payload_slot &&
           a.semantic_target_id == b.semantic_target_id &&
           a.semantic_target_first_seen == b.semantic_target_first_seen &&
           a.semantic_target_class == b.semantic_target_class &&
           a.align_mode == b.align_mode;
}

}
#endif
