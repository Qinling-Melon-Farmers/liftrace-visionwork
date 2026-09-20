# 单帧检测结果
Header header
string class_name
float32 class_confidence
float32 geometry_confidence
bool geometry_verified
sensor_msgs/RegionOfInterest roi
geometry_msgs/Point center_px
# true 表示中心来自蓝色圆环/红十字等几何精修，而不是仅来自检测框
bool center_refined
# 中心来源枚举：bbox/circle_geometry/red_cross_geometry/landing_geometry
string center_source
# 标准靶是否已与同帧圆环一一关联；独立几何靶表示自身几何是否有效
bool association_valid
# 空字符串表示当前观测可继续进入地图/候选链；否则为稳定机器可读原因码
string reject_reason
# 地图投影结果；map_valid=false 时 map_point 不可用于导航或投递
bool map_valid
geometry_msgs/Point map_point
string map_frame
float32 map_quality
# 图像时间戳与实际采用 TF 时间戳的绝对差 [s]；无可用 TF 时为 -1
float32 transform_age_sec
