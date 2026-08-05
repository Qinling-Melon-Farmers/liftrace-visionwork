#ifndef PATROL_CONTROL_DROP_ACTION_H_
#define PATROL_CONTROL_DROP_ACTION_H_

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

}  // namespace patrol_control

#endif  // PATROL_CONTROL_DROP_ACTION_H_
