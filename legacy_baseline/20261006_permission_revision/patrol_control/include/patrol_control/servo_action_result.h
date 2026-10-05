#ifndef PATROL_CONTROL_SERVO_ACTION_RESULT_H_
#define PATROL_CONTROL_SERVO_ACTION_RESULT_H_
#include "patrol_control/drop_action.h"

namespace patrol_control {
// The fenced service itself freezes/validates the full mission identity before
// invoking raw. Its per-RPC echo is independent from the mission decision_seq.
template <typename Request, typename Response>
DropActionResult classifyServoAction(const Request& request, bool transport_ok,
                                     const Response& response) {
    if (!transport_ok || response.request_id != request.request_id ||
        response.payload_slot != request.payload_slot || !response.terminal) {
        return DropActionResult::kServiceCallFailed;
    }
    if (response.execution_state == Response::NOT_STARTED && !response.res) {
        return DropActionResult::kNotStarted;
    }
    if (response.execution_state == Response::COMPLETED && response.res) {
        return DropActionResult::kSuccess;
    }
    // Invalid combinations, bool-only/unknown and raw-started failure all keep
    // the slot uncertain. Do not infer NOT_STARTED from res=false.
    return DropActionResult::kServiceCallFailed;
}
} // namespace patrol_control
#endif
