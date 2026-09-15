# 分支与worktree清理完成

删除36个本地分支引用、26个远端分支引用；所有被删除引用的提交均由最终保留分支持有，原名称和提交记录在本目录cleanup系列JSON中。main未改动；竞赛综合分支别名快进统一到已有86e382d，重复driver2别名删除。具独立未合入提交的分支保留。认证查询未发现打开的PR。

退休r2026-competition-integrated和r2026-coverage-efficiency-research两个worktree，保留其Git分支来源、历史文档、日志、交付和忽略的非构建资产。归档位于/home/xhj/liftrace-archives下，旧路径是兼容符号链接，不再是Git工作树。如需恢复开发，应从保留分支重新创建worktree，再重建build/devel，旧路径主要供历史报告/日志读取。

保留三个实际工作树：原始liftrace（有用户未提交内容，未触动）、板端frame-fix（保留独有小场配置）、高位研究（当前工作）。基线模型权重路径仍在原始liftrace中。

删除引用、worktree列表、兼容链接和五套历史原始轨迹可读性均已核对。大部分旧数据按要求保留，没有把VHDX宿主空间变化等同逻辑文件清理量。
