# 2026-10-08 历史保全变更记录

- 日期：2026-10-08（Asia/Shanghai）。
- 改动范围：本目录新文档、JSON、证据副本；共享 Git 新增 35 个 annotated tag；仓库外 /home/xhj/liftrace-archives/20261008-history。
- 具体改动：核实 36 个删除分支名、1 个观察点别名；38 行点保留本地/远端归属；索引 7 个归档、3 移除 checkout、2 退休归档；导出原历史与作者日期。
- 验证结果：tag 类型/解引用通过；bundle verify 和独立空仓导入通过；107 refs 完全一致，908 个去重提交；38 历史点和 2 已恢复 HEAD 的 commit 字节相同。本代理未删除或移动任何既有 ref；前后快照远端跟踪 ref 变化 2 条，最新 reflog 均为其他并发代理的 update by push，已单列记录。无 checkout/index 操作。
- 遗留问题：单 YOLO gate 仅观察点；12 个 manifest HEAD 不在本共享仓；归档产物不等于完整 checkout；未查询来源仓或全面扫描历史秘密。
- 下一步：主代理 review 后决定文档提交及标签 push；本代理不 push、不合并 main、不删除 ref。
- 文档边界：按用户独占写入限制，在本新目录记变更，不改全局台账或其他 B 文档，不 commit。

## 2026-10-08：指定来源仓只读补查

- 日期：2026-10-08（Asia/Shanghai）。
- 改动范围：仅本报告目录的 INDEX.md、summary.json、archive_observations.json，新增 cross_repository_check.json 与补查脚本。
- 具体改动：对视觉共享 Git 中原查无的 12 个 SHA，在指定导航 Git 与 liftrace-sim 逐项 cat-file；记录导航工作树共用 Git、原作者日期及现存 ref；明确 bundle 不含导出后主代理 main@6cd0207fb5ea669416d01c06d6a1b65eac6730e9。
- 验证结果：12/12 在导航 Git 存在且有包含 ref；liftrace-sim 中 0/12；指定仓中未定位数 0。独立 bundle 导入仓查无新 main commit。全部只是只读 Git 查询，无 fetch/push、tag 或来源文件修改，bundle 未重建。
- 遗留问题：旧 bundle 是导出时刻视觉仓快照，不覆盖导航独立 Git 或其后主代理提交；不能据此声称这些历史对象丢失。
- 下一步：本次按用户要求停止。35 tags 由主代理推送；无需本代理继续操作。
