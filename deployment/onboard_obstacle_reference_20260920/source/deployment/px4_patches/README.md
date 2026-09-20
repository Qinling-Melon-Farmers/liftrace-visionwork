# PX4自动任务激活修复

对应源码基线PX4 `99c4040`。补丁修复从没有FlightTask的模式（如OFFBOARD）进入自动任务时，将历史EKF重置重复应用到新目标的问题。本机已在R63观察到AUTO.LAND航向目标约90度跳变。

在PX4源码目录应用 `git apply /path/to/0001-initialize-task-reset-counters.patch`，随后按实际飞控板型构建；仿真使用 `make px4_sitl_default`。若补丁已应用，`git apply --reverse --check`可确认。不能用笔记本SITL二进制替代实际板型固件，也不要重复应用。

补丁不禁用EKF重置：有旧任务时继承其计数；没有旧任务时以当前估计计数建立初始参考，后续新重置仍处理。无需修改传感器坐标或重置EKF来掩盖航向跳变。当前任务只构建仿真固件，不刷写实机。
