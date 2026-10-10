# 离线 replay 编码与小缓存对照（2026-10-10）

结论：CLI 默认 `--encoder cpu`，保留显式 `auto` / `nvenc`。RTX4060 Laptop 在 WSL 下实际四路 NVENC 编码、解码预检通过，但本次短片没有总耗时收益。真实 25 秒片段中新版 CPU 两次渲染耗时中位数较旧版少约 **8.3%**；这仅是该片两次中位数，不能保证全 bag、其他场景或机器同样提速。

## 真实片段与方法

- 来源：`试飞产物/flight_debug_0.bag`，原记录总长 232.723 秒；只读取既有 `analysis_flight_latest_20261010/data.json` 并从原 bag 提取 **145～170 秒**相机原始 JPEG。未覆盖原 analysis、未重新渲染整飞行。
- 片段：25 秒、10fps、1 倍速，四路各 250 帧；115 张原始相机图，包括起始前 0.5 秒的状态预热，另有实际里程计、规划 Marker、点云、TF、检测与任务记录。窗口前的最后状态保留，时间戳一起平移；动态 TF 年龄与 30ms 检测匹配规则保持不变。
- 旧版：Git `bf81dc37` 的 `tools/bag_replay/bag_replay.py`，从 `git show HEAD:...` 写入独立临时目录运行；新版：本轮未提交的 replay 文件。相同导出输入、10fps 和 `camera_init` 显示坐标系，既有 conda `rl_drone`，不重跑模型。
- 两轮顺序为旧 CPU → 新 CPU → NVENC，以及 NVENC → 新 CPU → 旧 CPU；计时为 render 函数完整墙钟时间，包含初始化、GPU 预检、CPU 绘图、传帧、编码器收尾和报告。verify 单独计时。CPU 解码/绘图库预先导入，未将冷启动导入差异当作缓存收益。
- 主机：RTX4060 Laptop、WSL Ubuntu20.04、系统 FFmpeg 4.2.7。CPU 为 libx264/veryfast/CRF23；NVENC 为 h264_nvenc/fast/VBR/CQ23。量化参数并不保证两种编码器画质或体积等价。

| 版本 | 第一轮渲染秒 | 第二轮渲染秒 | 渲染中位秒 | verify 中位秒 |
|---|---:|---:|---:|---:|
| 旧 CPU | 10.958 | 10.487 | 10.722 | 1.542 |
| 新 CPU | 10.356 | 9.307 | 9.831 | 1.604 |
| NVENC | 12.715 | 10.868 | 11.792 | 1.989 |

NVENC 预检分别花费 1.608 / 1.629 秒，计入总渲染时间；实际编码阶段的传帧与绘图存在重叠，也受主机负载影响。不能把 `encoder_pipe_seconds` 解释成纯 GPU 核心耗时，不能由单次较快的管道时间推广总耗时收益。

原始计时：[各轮结果](real_25s_runs.json)、[中位数](real_25s_medians.json)。六组输出各四路全部经过一次 FFmpeg 完整解码，并核对每路 250 帧、尺寸和 25 秒时长。视频只保留于 `/tmp/replay_real_encoder_20261010/`，没有复制到此报告目录或 Git。

## 合成短片与优化边界

最初未改旧 CPU 的 8 秒合成四路片耗时 2.382 秒，cProfile 显示 FFmpeg 管道写入等待约 1.282 秒，整帧 `tobytes` 约 0.123 秒；这支持优先检查编码和传帧边界。最终同类短片新 CPU 2.559 秒、NVENC 4.241 秒，不能声称这个合成样例总体提速。

最终合成对照另外使用完全相同的四张预载 BGR 帧重复编码 80 帧，排除 OpenCV 绘制：CPU 1.682 秒，NVENC 2.394 秒，均含进程启动、传帧与收尾，不含 GPU 预检。合成 replay 的一次完整解码校验分别为 CPU 0.641 秒、NVENC 0.714 秒。数据见 [8 秒合成对照](synthetic_8s_benchmark.json)。

新版 CPU 小优化包括：同一 JPEG 被多次重采样时复用缩放结果；两种曲线尺寸复用全程坐标；通过 buffer 传帧，避免每路每帧额外生成完整 bytes；限制 FFmpeg 格式转换线程为 2。地图、检测绘图、图像拼接仍在 CPU 上，没有使用 `cv2.cuda`，没有改变显示坐标、画面布局、1 倍速或视觉判定。

四路视频旧/新 CPU 的第 0、125、249 帧解码后 **逐像素一致**，最大通道差全部为 0，见 [画面抽查](cpu_pixel_comparison.json)。抽查不是逐帧画质保证，真实片的两次中位数也不能把全部收益单独归因于某一缓存；主机负载波动仍在。

## 失败处理、校验与打包

`auto` 实际以输出尺寸预检四路并发 NVENC，预检失败回退 CPU并记录原因；显式 `nvenc` 失败报错。`auto` 仅按能力选编码器，不比较效率，所以本机默认选 CPU。正式编码中途失败不回退、不宣布部分成功：记录 failed summary，返回非零，保留帧，拒绝 verify 成功和清理。所有编码器退出且传入帧数达标后才晋升 `.partial.mp4`。

verify 从一次 FFmpeg 全片解码的 `-progress` 获取实际帧数，再用 ffprobe 仅读 metadata 尺寸、时长、`nb_frames`；没有 `ffprobe -count_frames` 的第二次全片解码。旧 summary 仍可核验。

最终 `python -m unittest test_core test_encoder -v` **14/14 PASS**（1.101 秒），包括：CPU 跳过探测、能力失败自动回退、显式 NVENC 拒绝假成功、真实失败子进程回收、完整 CPU 编码、0 退出码但帧数不足不晋升、中途失败及保留图片、验证帧数失败与陈旧成功记录作废，另保留时序匹配、坐标变换及无相机契约。见 [测试日志](helper_tests.log)。`bash -n run.sh` 通过。

打包运行依赖新增 **`tools/bag_replay/video_encoder.py`**，必须与 `bag_replay.py` 同目录；`test_encoder.py` 是可附带的测试与短合成基准工具。本轮不修改 workbench；主代理同步工作台默认 CPU 和 build_zip 依赖。独立生产 HTTP 三秒真实片测试已由主代理通过，结果在 `/home/xhj/liftrace-deliverables/workbench_replay_20261010/http_smoke/result.json`，本轮未重复。

## 本轮变更记录

日期：2026-10-10。范围：`tools/bag_replay/bag_replay.py`、`video_encoder.py`、`test_encoder.py`、README、run.sh usage，以及本独立报告目录。具体改动：可选编码器、实际并发预检/回退、编码失败处理、CPU 小缓存和单次完整解码验证。验证：14 项测试及上述短片对照通过。遗留：本机 NVENC 没有体现总效率优势，未测试长片或其他硬件。下一步：主代理打包时同步新增运行依赖和默认 CPU；其他主机按显式选项自行选择编码器。不 commit，不改 merge_bags.py、workbench 或共享变更记录。
