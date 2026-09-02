uint8 ACCEPTED=0
uint8 PLANNING=1
uint8 TRAJECTORY_READY=2
uint8 REPLANNING=3
uint8 TRAJECTORY_FINISHED=4
uint8 FAILED_ATTEMPT=5
uint8 CANCELLED=6

std_msgs/Header header
uint64 event_seq
uint32 goal_seq
uint8 status
uint32 planning_attempt
geometry_msgs/PoseStamped requested_goal
geometry_msgs/PoseStamped effective_goal
float64 distance_to_goal
string reason
