# 场景几何工具

当前来源为liftrace-sim本地fork的feat/high-view-current-profile，具体提交见docs/planning/corridor_presentation_20260919/scene_source.json；两份实现按文件同步。仅生成场景与评测几何，不提供另一套飞行控制。

`--door-mode continuous --outer-wall-height 4.0`生成固定两道墙上连续移动的0.80m开口及4m外围墙。可用`--door-centers 8.12 8.58`固定重现；省略新参数保留左右门/旧墙高模式。门中心范围[8.0,8.7]，墙/走廊位置不变，任务仍只用门前后中性航点，不能读取Gate内的真实开口。

用户本轮要求不仿真；新模式只完成静态/单元验证，不能借旧LR矩阵PASS冒称连续门通过。不要重新生成覆盖历史冻结场景。
