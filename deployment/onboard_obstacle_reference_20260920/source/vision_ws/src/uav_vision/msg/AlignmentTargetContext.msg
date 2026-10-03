uint8 SCHEMA_VERSION=1

# 与 uav_mission/NavigationDecision v1 同值，但消息本身不依赖 uav_mission。
uint8 SEARCH=0
uint8 APPROACH=1
uint8 ALIGN=2
uint8 RESUME=3
uint8 RETURN_HOME=4
uint8 LAND=5
uint8 HOLD=6
uint8 ABORT=7

# 导航 coordinator 冻结并续租的末端对准上下文。
std_msgs/Header header
string source
uint8 schema_version
bool active
string mission_id
uint32 decision_seq
time deadline
uint8 command
string class_profile
string align_mode

# has_target 显式区分“无目标”与合法的 semantic_target_id=0。
bool has_target
uint32 semantic_target_id
time semantic_target_first_seen
time target_observation_stamp
string semantic_target_class
uint16 attempt
uint8 payload_slot

# 语义目标的冻结地图位姿；圆环几何实例必须在同 frame 内按距离关联。
geometry_msgs/PoseStamped target_pose
float32 max_association_distance_m
