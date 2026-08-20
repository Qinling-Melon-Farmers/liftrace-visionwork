# 评测包内部的统一辅助粗候选；不是控制或投递接口。
std_msgs/Header header
uint32 source_id
# AUX_CV / AUX_YOLO
string source
# OpenCV 蓝环为 circle；YOLO 可给出六分类语义提示。
string class_hint
float32 confidence
geometry_msgs/Point map_point
string map_frame
float32 map_quality
# 当前阶段由 map_quality 估算；后续可替换为投影器真实不确定度。
float32 position_uncertainty_m
string uncertainty_source
float32 transform_age_sec
uint8 state
uint32 observe_count
time first_seen
time last_seen
bool valid
string reject_reason
