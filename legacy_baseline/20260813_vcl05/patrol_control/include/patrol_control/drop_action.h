#ifndef PATROL_CONTROL_DROP_ACTION_H_
#define PATROL_CONTROL_DROP_ACTION_H_

#include <array>

namespace patrol_control {

// Keep transport success separate from the actuator acknowledgement.
enum class DropActionResult {
    kSuccess,
    kInvalidServoId,
    kServiceCallFailed,
    kRejected,
};

inline DropActionResult classifyDropAction(int servo_id,
                                           bool service_call_ok,
                                           bool response_ok) {
    if (servo_id < 1 || servo_id > 3) {
        return DropActionResult::kInvalidServoId;
    }
    if (!service_call_ok) {
        return DropActionResult::kServiceCallFailed;
    }
    return response_ok ? DropActionResult::kSuccess
                       : DropActionResult::kRejected;
}

inline bool dropActionSucceeded(DropActionResult result) {
    return result == DropActionResult::kSuccess;
}

struct DropReleaseGate {
    bool mission_permission_active = false;
    bool mission_permission_fresh = false;
};

inline bool canRequestDrop(bool require_mission_permission,
                           const DropReleaseGate& gate) {
    return !require_mission_permission ||
           (gate.mission_permission_active &&
            gate.mission_permission_fresh);
}

inline std::array<double, 2> projectPixelOffsetToBody(
    double dx_px, double dy_px, double pixel_to_meter_ratio,
    const std::array<double, 4>& pixel_to_body_matrix) {
    return {{
        pixel_to_meter_ratio *
            (pixel_to_body_matrix[0] * dx_px +
             pixel_to_body_matrix[1] * dy_px),
        pixel_to_meter_ratio *
            (pixel_to_body_matrix[2] * dx_px +
             pixel_to_body_matrix[3] * dy_px),
    }};
}

}  // namespace patrol_control

#endif  // PATROL_CONTROL_DROP_ACTION_H_
