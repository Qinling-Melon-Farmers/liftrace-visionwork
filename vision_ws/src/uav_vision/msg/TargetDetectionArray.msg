# 一帧内的全部检测结果
Header header
# 发布该数组的视觉节点。融合器用它判断同一帧的必要分支是否均已完成；
# 不得用 frame_id 代替该字段，因为 frame_id 仍用于相机/TF 投影。
string source
# 经融合节点输出时，列出该图像时间戳已经返回结果（包括空结果）的原始
# 检测分支；后续节点必须原样透传，用于区分“已处理但未检出”和“未处理”。
string[] completed_sources
TargetDetection[] detections
