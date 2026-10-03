## 完成后清理

两轮报告和四段视频核验完成、仿真结束后，删除 14 个明确目标，删除前文件占用合计 **4,157,575,168 字节，约 3.87GiB**。这是 WSL 文件占用统计，不等于已经压缩 Windows 上的虚拟磁盘文件。

- 删除两份 8 月 `toudi4_coverage_r6_v2_20260813_221737` 图像 bag，保留原 Gate、任务状态、投递记录、manifest、run.log、timeline 及联调总结；另将三投回执和关键状态汇总到 `/home/xhj/cleanup_aug_compact_20261004`。原 Gate FAIL 的历史原因仍如实保留。两包只有原图/debug 图，没有位姿或任务话题，未虚构额外位姿导出。
- 删除六个已归档或 `/tmp` 中的旧构建目录及三个临时编译测试程序。确认其中 `src` 命名子目录为 CMake 目标中间产物；原始源码、相邻 devel 和实验报告均保留。
- 移除三棵干净、远端可恢复的退役工作树：`r2026-board-reference-sync`、`drop-height-pixel-scale-20261003`、`vcl06-local-full-mission`。**没有删除 Git 分支**。
- 保留含未提交改动的 `vcl06-planner-stop-ack`、所有现用研究/整机/板端工作树、本轮原始视频与 bag、近期实飞 bag/ULog 和 H 诊断记录。

精确路径及占用见 [CLEANUP.json](CLEANUP.json)。本次没有清空整个 `/tmp`、删除原始资产或重写 main。
