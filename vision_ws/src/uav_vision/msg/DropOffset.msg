# 对准偏差（图像像素坐标系）
Header header
float32 dx_px
float32 dy_px
float32 radius_px
float32 quality
# Experimental exact projection; false in the legacy pixel mode.
# header.stamp is exposure time; header.frame_id is the source optical frame.
# map_point is the FC XY alignment goal AFTER the single slot compensation.
bool map_valid
geometry_msgs/Point map_point
string map_frame
float32 alignment_error_m
float32 alignment_tolerance_m
uint32 target_id
time target_first_seen
