# 本机清理及恢复说明

日期：2026-10-19。清理依据：用户明确允许清理冗余仓库、分支、原始bag，并说明目的是释放F盘空间。

## 已完成

- 121份项目bag、active、原包副本及重复链接已删除，删除前实际占用约10.60 GiB。不是移入回收站，不能用Git恢复未入库的bag。
- 旧工作树build/devel/install与独立编译检查缓存、废弃sim副本等50项清理约6.10 GiB。当前整机build/devel和训练数据/模型未清理。
- liftrace-sim工作区已先归档为liftrace-sim-workspace.tar.gz，逐文件对照验证，再删除原目录；Git全部可达历史另存sim_history.bundle。
- 32个本地冗余分支已转为annotated archive/20261019/...标签并推远端；未删除远端分支。当前工作树所用分支保留。见[分支映射](ARCHIVED_BRANCHES.json)。
- 原作者、提交时间、提交拓扑不重写。vision/navigation/sim三个Git bundle与分支/引用清单保存在外置交付目录。
- 分析报告、CSV、参数、已生成回放视频、ULog、原视频及训练集仍保留。历史报告包含已删除bag路径属于预期。

## F盘实际空间

Ubuntu虚拟磁盘位于F:\WSL\Ubuntu20.04\ext4.vhdx。清理后执行WSL的fstrim，F盘可用字节由9,472,397,312增至144,342,327,296（约8.8→134.4 GiB）。这包括早先删除但未回收的虚拟盘空间，不是宣称本次删了125GiB bag。归档新产物还会占用少量空间。

没有关闭WSL，没有删除ext4.vhdx，没有卸载软件或删除原始2025源码。

## 恢复分支与源码

外置目录：`/home/xhj/liftrace-deliverables/competition_closeout_20261019/`。

从远端恢复归档分支（按映射表替换标签名）：

```bash
git fetch origin --tags
git switch -c restored-topic archive/20261019/feat/原分支名
```

从bundle恢复到新目录：

```bash
git bundle verify /path/to/vision_history.bundle
git clone /path/to/vision_history.bundle recovered-vision
```

sim工作副本的未跟踪资源在liftrace-sim-workspace.tar.gz内，先解压到新目录查看，再按sim_history.bundle恢复Git。构建缓存通过正常构建脚本重建。

详细清理清单在外置目录cleanup_executed.json与bag_deletion_manifest.json。bag不可由模型包、Git bundle或视频逆向还原；如确有新分析需求，需要其他持有人副本或重新录制。
