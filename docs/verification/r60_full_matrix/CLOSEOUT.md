# R60收口

- 实跑：先导seed11完整PASS37/37，随后冻结源码十seed2/10完整PASS、8/10三投；全部收尾，不再启动新仿真。
- 源码：整机9cfb3e5、main集成7379ef5、导航e4a4cc6已推各自功能分支；本轮收口文档另作提交。视觉开发分支同步联合报告，不宣称其整个checkout就是R60运行源码。
- main：保持31d0b2a。本轮未合main、未创建新的main验收tag，原因是十seed鲁棒性未通过。原“修复验证后保留分支方式合main”的目标仍未满足。
- 成功归档：仅归档完整PASS先导run，附报告与十seed关键记录ZIP；失败全量日志不发布Release，原始失败TXT/JSONL/CSV/ULog留在WSL对应run目录。
- 成功Release入口：[R60完整先导PASS与十seed结果](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/releases/tag/sim/r2026-r60-low-corridor-pass)。该Release按实验候选标记，不作为main/实机验收。
- 远端清理：R54失败、R55中断全量Release退役，删除资产总计4881640745字节；小报告/拓扑附件保留在WSL，Git报告、标签、分支均保留，R56成功Release保留。此操作不等于释放宿主磁盘。
- 本地：全部记录仍在WSL项目logs；本轮无bag、无跨盘日志。只清理本任务图形试验的重复临时输出，不删除原始工程或本轮必要失败记录。未再次压缩VHDX。

下一步为[失败分析](FAILURE_ANALYSIS.md)列出的软件修复和新验收，再进行板端/实机验证；本次未实施未验证的飞行补丁。
