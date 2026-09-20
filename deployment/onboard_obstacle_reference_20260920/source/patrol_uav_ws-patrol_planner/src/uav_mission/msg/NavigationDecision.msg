uint8 SCHEMA_VERSION=1

uint8 SEARCH=0
uint8 APPROACH=1
uint8 ALIGN=2
uint8 RESUME=3
uint8 RETURN_HOME=4
uint8 LAND=5
uint8 HOLD=6
uint8 ABORT=7

std_msgs/Header header
uint8 schema_version
string mission_id
uint32 decision_seq
time deadline
uint8 command
string class_profile
bool has_goal
bool has_target
uint32 target_id
time target_first_seen
time target_observation_stamp
string target_class
uint16 attempt
uint8 payload_slot
geometry_msgs/PoseStamped goal
string reason
