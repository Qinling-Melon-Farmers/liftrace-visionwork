# Controller evaluation time in the mission frame (not the camera frame).
std_msgs/Header header
# Exact source image stamp from DropOffset.header.stamp; never a memory heartbeat.
time observation_stamp
time odom_stamp
# Same frozen action/context identity as AlignmentTargetContext.
string mission_id
uint32 decision_seq
uint16 attempt
uint8 payload_slot
uint32 semantic_target_id
time semantic_target_first_seen
string semantic_target_class
# Existing visual mode: drop_circle or drop_cross.
string align_mode
bool valid
# Controller owns compensated FC XY error and settled-speed thresholds.
bool aligned
# Diagnostic only: vision loss must never clear the controller's frozen goal.
bool frozen
geometry_msgs/Point target_fc
float64 horizontal_error_m
float64 horizontal_speed_mps
string reason
