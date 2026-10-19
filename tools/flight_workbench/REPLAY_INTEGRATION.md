# 2026-10-10 Bag 回放工作台集成

独立 `/replay` 页从主页和 `/logs` 进入，不依赖板端连接。沿用工作台本地库的作业记录、
状态查询和日志展示方式；未重构 SSH/ULog、未修改飞行链。工作台生成只调用随包 `run.sh`，
不启动 ROS 节点、设备或模型推理。

## 启动与本地目录

在 WSL 中沿用已有环境：

```bash
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-board-vision-tests && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && bash tools/flight_workbench/start_workbench.sh --transport local --port 8771'
```

浏览器打开终端打印地址的 `/replay`。若已有工作台在 8771，由操作者正常重载即可；端口被
占用会自动顺延。独立测试应使用临时状态目录与其他端口，不停止现场工作台。

源码 checkout 默认按顺序扫描 `试飞产物/`、工作台 `data/` 及个人状态的 `replay_library/`。
解压分发包默认只扫包内 `data/` 与个人库。跳过 `frames/`、隐藏目录和符号链接，每根最多
2000 个目录、深度 8 层；达到上限会在页面提示。旧仿真 `logs/` 不默认扫描。需要更多目录
时，在启动使用的 YAML 添加配置（不是网页参数；修改后重启本地服务）：

```yaml
replay:
  data_roots:
    flights: /home/xhj/liftrace-worktrees/r2026-board-vision-tests/试飞产物
    extra: /absolute/local/bag-and-replay-data
```

Windows 使用 Windows 本地路径，例如 `flights: 'D:\flight_data'`，或把结果放在解压包
`data/`。原生 Windows 只浏览完整结果，明确提示生成须在已有 ROS Noetic 的 WSL/Linux
后端进行，不把“Windows 装了 WSL”当作生成能力。ULog 的原生功能保留。

## 行为与接口

- `GET /api/replay/library`：预定根、bag 列表、验证完成的结果、能力和作业。
- `GET /api/replay/jobs`：持久状态、阶段、渲染进度、最近 12KB 的 `operation.log`。
- `POST /api/replay/start`：`root`、`path`、`fps`（1–30）、`frame`、`encoder`。
  `encoder` 仅允许 `cpu|auto|nvenc`，默认 `cpu`；传递给原工具 `--encoder`。
- `GET/HEAD /replay/result/<root>/<目录>/...`：HTML 与相对资源，视频分块流式传输，
  单段 Range 返回 206；无效/多段 Range 返回 416。目录 URL 保持末尾 `/`。

相对路径拒绝绝对路径、`..`、Windows 路径和符号链接；生成使用新的 UUID 输出目录，不
覆盖原结果。线程与继承给 `run.sh` 的文件锁限制同一输出库只运行一轮，跨工作台实例也
不能叠加；锁生命周期由 OS 管理。刷新页面仅查询，不启动任务。依赖探测和实际子进程都
只移除工作台借用 pexpect 所加的 `/usr/lib/python3/dist-packages`，保留用户其他
`PYTHONPATH` 项，避免 conda 3.9 误载 Ubuntu 系统 NumPy。子进程开启无缓冲日志。

已有结果需 `summary.json.video_files`、`validation.json` 及对应视频大小一致；新的作业
还需工具成功退出，才显示完成和播放器。导出没有百分比，渲染进度来自实际 `Render n/m`
输出。CSP sandbox 保留；只读结果 GET/HEAD 允许沙箱媒体的 null/缺省 Origin 和 cross-site
标记，Host 与路径检查仍保留。POST 和库/控制 API 继续严格同源。

GPU 只负责 FFmpeg 视频编码，不加速 OpenCV 画面绘制。同一真实 25 秒短片已完成编码
对照：旧 CPU 10.72 秒、新 CPU 9.83 秒、NVENC 11.79 秒，因此默认 CPU；不能据可用
NVENC 推断整条渲染链更快。auto/NVENC 仍可手动选择，实际结果与回退见工具日志。

## 验证

针对测试在已有 `rl_drone` 中执行 `tests/test_replay.py`：现有结果发现、未完成结果过滤、
相对资源、完整/单段/后缀 Range、HEAD、沙箱媒体请求、路径越界/符号链接、同源拒绝、
Windows 仅查看、PYTHONPATH 清理、真实依赖探测、单作业排他、锁继承、encoder 透传、
无缓冲进度和验证完成状态。作业测试使用临时 fixture，不连接板端。

`tests/test_distribution.py` 验证临时 ZIP 解压独立启动、随包 replay/encoder 和原 ULog
源码、API/页面及 `merge_bags.py` 排除；既有 `tests/test_ulog.py` 验证真实合成 ULog
上传、解析与导出。测试只使用临时 ZIP，不清理任何已有交付包。

8771 已验证真实 bag 3 秒切片，经 HTTP 启动作业到四路 NVENC/全解码/ready，
Range 100–199 返回 206/100 bytes，设备 `sessions={}`；Edge 已实际播放并拖动到 100 秒，
主页、日志、回放页和结果四页无 JS 异常。产物位置：
`/home/xhj/liftrace-deliverables/workbench_replay_20261010/http_smoke/result.json`，页面截图在
同一交付根目录的 `screenshots/`。Windows 原生独立包的启动和页面/API检查已通过，
详细范围见 `docs/verification/workbench_replay_20261010/REPORT.md`。

工作台改动范围为 `tools/flight_workbench`（含本说明/测试），本轮未部署设备。
回放工具源码维护在 `tools/bag_replay`；工作台打包白名单含
`video_encoder.py`，明确不含未跟踪的 `merge_bags.py`。
