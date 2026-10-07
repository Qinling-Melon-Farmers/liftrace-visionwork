# 2026-10-08 历史分支与工作树找回索引

本次只核查历史归属并保全提交，不改运行代码。共享仓为 /home/xhj/liftrace（liftrace-visionwork）；文档只写在本新目录。新增本地标签，未 commit、未 push、未切分支、未删除或改写任何已有 ref，未操作 root 文件或 index。未仿真、未上板。

## 准确数量与界限

| 项目 | 数量与含义 |
|---|---|
| 原 13 个候选 | **13/13 能定位删除记录 tip**；早期 checkout 观察点不再作为最终 tip 依据 |
| 全部删除分支名 | **36 个**，本地/远端同名去重；精确删除时 SHA 均存在 |
| 仅观察点的额外名字 | **1 个**：feat/oblique-camera-single-yolo-gate；非最终 tip |
| 历史点明细 | **38 行 / 36 个不同提交**，含 VCL06 本地/远端不同 tip 和观察点别名，不等于 38 个分支 |
| 先前已恢复的分支 | **2 个**：旧高位、运动约束；本次未创建或移动 |
| 原始资产目录 | **7 个**：20260905 五个 + 20260915 两个；均可读，不等于 7 个精确分支 tip |
| 工作树记录 | **3 条 20260909 移除 checkout** + **2 条 20260915 退休归档**；不与分支数量相加 |
| 归档 manifest | **182 个文件、77 个不同 HEAD**；65 个在本共享仓存在；其余 12 个经补查全部在导航 Git 存在且有 ref，未丢失 |
| 新增保全 tag | **35 个本地 annotated tag**；重复提交与已有直接 ref 复用 |
| 离线历史 | **908 个去重提交、107 个 ref**；历史资产规模，不是贡献总量 |

精确恢复指能够原样重新建立“记录中该分支删除时”的引用。未记录的分支、更早删除或后来另一个同名分支不在保证范围。分支名是历史关联，作者/提交者以原 Git 对象为准；共享祖先不能算作该分支专属贡献。

### 已纠正的观察点误判

- navigation-mission-visual-delivery 的旧观察点是 2ccd8a088e317541c30c51093c4220863fd73479，删除 tip 是 **8255aa4400ca1be1cb99c236adcb645861081c49**。
- vcl06-execution-integration 本地删除 tip 是 **ad93d364ac7e34e4c58ddf40dce796042ffbfc96**，远端是 **1a494105064525c29ceb6e51a367f5a0d281ef05**；tag 分别用 /local 和 /origin，均保留。
- competition-main-r64-acceptance 的删除 tip 是 **7f67a68d9acc06ff541a8e2d91de0b22d89aaaba**，由 gate/r64-seed11-full 直接承载；其第二父 1c662cea…只是合并源，不是该被删分支的记录 tip。
- gate/vcl04-clean-rerun 注释中的 f4aef95 与实际对象不符：解引用以 **7e626ad93117765d6a76ae67fe70b0d42505f3c8** 为准，第二父 af80056…；本轮未改旧 tag 注释。

## 分支、完整提交与当前承载引用

删除来源副本见 [sources](sources/)；[branch_index.json](branch_index.json) 保留逐项原始来源、local/origin 归属、旧候选观察点、所有包含 ref 以及完整父提交和标题。下表姓名和 ISO 日期均直接来自原 commit。

| 原分支与归属 | 完整提交 | 原作者与作者日期 | 原提交者与提交日期 | 当前直接承载 ref | 判定 |
|---|---|---|---|---|---|
| chore/docs-progress-sync (local/origin) | `0a41ee0c1cc696688af7f67a643de5ab88f6e897` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T00:28:45+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T00:28:45+08:00 | refs/tags/archive/recovered-20261008/chore/docs-progress-sync | 删除时精确 tip |
| chore/docs-responsibility-remap (local/origin) | `2dd1d91f96fb55a66d6f5d189746f163e0aa1829` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T00:38:21+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T00:38:21+08:00 | refs/tags/archive/recovered-20261008/chore/docs-responsibility-remap | 删除时精确 tip |
| chore/freeze-oblique-camera-exploration (local/origin) | `9889591a595114ada9a6cfc11233e31e2cbe7c93` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-23T23:04:16+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-23T23:04:16+08:00 | refs/tags/archive/recovered-20261008/chore/freeze-oblique-camera-exploration | 删除时精确 tip |
| chore/gate-tagging-convention (local/origin) | `0e69fef0626ca95f835f0db5e88fe6ac69252217` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T00:13:51+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T00:13:51+08:00 | refs/tags/archive/recovered-20261008/chore/gate-tagging-convention | 删除时精确 tip |
| chore/local-delivery-check-20260911 (local) | `d55939e51ed12c926e24bbae225d4905c05ffd3c` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T14:04:55+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T14:04:55+08:00 | refs/tags/archive/recovered-20261008/chore/local-delivery-check-20260911 | 删除时精确 tip |
| docs/liftrace-sim-upstream (local) | `9bc376b8c7642fef19a4bb5ff7571a088b5cad76` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T23:22:36+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T23:22:36+08:00 | refs/tags/archive/recovered-20261008/docs/liftrace-sim-upstream | 删除时精确 tip |
| feat/competition-main-r64-acceptance (local/origin) | `7f67a68d9acc06ff541a8e2d91de0b22d89aaaba` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:02:18+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:02:18+08:00 | refs/tags/gate/r64-seed11-full | 删除时精确 tip |
| feat/competition-r64-log-only (local/origin) | `c451c14ce1ff0e6d47f9eaf139a5233651bc6bfa` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T22:15:38+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T22:15:38+08:00 | refs/tags/archive/recovered-20261008/feat/competition-r64-log-only | 删除时精确 tip |
| feat/competition-r64-raster-remote (local) | `cefee116b6e4f7468b2a36a5c016239c1cfc7850` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T22:13:06+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T22:13:06+08:00 | refs/tags/archive/recovered-20261008/feat/competition-r64-raster-remote | 删除时精确 tip |
| feat/competition-r64-readiness (local) | `925e380efff67adb96789ec06232f39b3f0bf300` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T22:32:31+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T22:32:31+08:00 | refs/tags/archive/recovered-20261008/feat/competition-r64-readiness | 删除时精确 tip |
| feat/competition-r64-topology-plan (local/origin) | `d076e64ec37709c9adbc526f81a25d6ce5a3a424` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:51:42+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:51:42+08:00 | refs/tags/archive/recovered-20261008/feat/competition-r64-topology-plan | 删除时精确 tip |
| feat/external-mission-coverage (local/origin) | `f46d45fbc59822abc497d386ae858a48f5a4e2b4` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-13T20:32:50+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-13T20:32:50+08:00 | refs/tags/archive/recovered-20261008/feat/external-mission-coverage | 删除时精确 tip |
| feat/full-random-five-seeds (local/origin) | `f2459e60fe5d313b10236f020d8f60ffc0a4d2d2` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-10T13:00:29+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-10T13:00:29+08:00 | refs/tags/archive/recovered-20261008/feat/full-random-five-seeds | 删除时精确 tip |
| feat/livox-driver2-integration (local/origin) | `86e382d50ffa1d27bce0652ac4d576c73d0bfcec` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T16:20:56+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T16:20:56+08:00 | refs/tags/archive/recovered-20261008/feat/livox-driver2-integration | 删除时精确 tip |
| feat/navigation-mission-visual-delivery (local/origin) | `8255aa4400ca1be1cb99c236adcb645861081c49` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-29T22:35:09+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-29T22:35:09+08:00 | refs/tags/archive/recovered-20261008/feat/navigation-mission-visual-delivery | 删除时精确 tip |
| feat/new-vision-coverage-search (local/origin) | `31b38ad6a4179c5a32672a2968dd245368e1ce99` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-06T07:16:45+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-06T07:16:45+08:00 | refs/tags/archive/recovered-20261008/feat/new-vision-coverage-search | 删除时精确 tip |
| feat/oblique-active-search-stability (local/origin) | `ab6e447a9c984aa4a5a3fc2198c3a53434480fea` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-20T17:52:41+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-20T17:52:41+08:00 | refs/tags/archive/recovered-20261008/feat/oblique-active-search-stability | 删除时精确 tip |
| feat/oblique-camera-search-feasibility (local/origin) | `b60a4d942ba7081d776bb81c434e18193a486b1b` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-15T03:49:37+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-15T03:49:37+08:00 | refs/tags/archive/recovered-20261008/feat/oblique-camera-search-feasibility | 删除时精确 tip |
| feat/oblique-camera-single-yolo-gate (checkout) | `b60a4d942ba7081d776bb81c434e18193a486b1b` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-15T03:49:37+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-15T03:49:37+08:00 | refs/tags/archive/recovered-20261008/feat/oblique-camera-search-feasibility | 仅 checkout 观察点，非最终 tip |
| feat/planner-map-20x20x5-sim (local/origin) | `7c80844c775758437de1dec0a0854950e42dda6b` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-23T23:33:28+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-23T23:33:28+08:00 | refs/tags/archive/recovered-20261008/feat/planner-map-20x20x5-sim | 删除时精确 tip |
| feat/search-planning-wait-bound (local) | `4195ef36c1b01d4d4ad286a4342469a2de420ea8` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T15:58:12+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T15:58:12+08:00 | refs/tags/archive/recovered-20261008/feat/search-planning-wait-bound | 删除时精确 tip |
| feat/vcl04-clean-rerun (local/origin) | `af800567a9b9d6f929ec96f981be8b1a6574fed1` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-28T23:17:48+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-28T23:17:48+08:00 | refs/tags/archive/recovered-20261008/feat/vcl04-clean-rerun | 删除时精确 tip |
| feat/vcl04-r6-bridge (local/origin) | `81090b4aa029d5f96df4b1d864f0e059e6ef7b54` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-13T23:23:49+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-13T23:23:49+08:00 | refs/tags/archive/recovered-20261008/feat/vcl04-r6-bridge | 删除时精确 tip |
| feat/vcl05-search-delivery-strategy (local/origin) | `7419a062f6c26cde8a93aeb1675325ab0bddd85e` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T02:32:52+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-14T02:32:52+08:00 | refs/tags/archive/recovered-20261008/feat/vcl05-search-delivery-strategy | 删除时精确 tip |
| feat/vcl06-execution-integration (origin) | `1a494105064525c29ceb6e51a367f5a0d281ef05` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T19:57:18+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T19:57:18+08:00 | refs/tags/archive/recovered-20261008/feat/vcl06-execution-integration/origin | 删除时精确 tip |
| feat/vcl06-execution-integration (local) | `ad93d364ac7e34e4c58ddf40dce796042ffbfc96` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T13:12:51+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T13:12:51+08:00 | refs/tags/archive/recovered-20261008/feat/vcl06-execution-integration/local | 删除时精确 tip |
| feat/vcl06-next-integration (local/origin) | `8341dd18b67bfb23510223257e2718f0fbd2d875` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T03:22:40+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T03:22:40+08:00 | refs/tags/archive/recovered-20261008/feat/vcl06-next-integration | 删除时精确 tip |
| feat/vcl06-random-start-gate (local) | `2247b10ce53b76f919218b9dd5fdedf9c96c16b1` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T02:40:59+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T02:40:59+08:00 | refs/tags/archive/recovered-20261008/feat/vcl06-random-start-gate | 删除时精确 tip |
| feat/vcl06-vision-context (local) | `37cb2a91106c7c7b35e77aa9caf15b89aef97a20` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T03:21:24+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T03:21:24+08:00 | refs/tags/archive/recovered-20261008/feat/vcl06-vision-context | 删除时精确 tip |
| feat/vdeploy-camera-extrinsic (local/origin) | `f0b1b8be8df7a4ba716ff68b452eb522ef3b0c07` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-05T19:49:44+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-05T19:49:44+08:00 | refs/tags/archive/recovered-20261008/feat/vdeploy-camera-extrinsic | 删除时精确 tip |
| feat/vsim04-ks2a543-camera-baseline (local/origin) | `56f06670530b92b6403a5b52069ed4fa20ec5247` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-05T21:36:33+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-05T21:36:33+08:00 | refs/tags/archive/recovered-20261008/feat/vsim04-ks2a543-camera-baseline | 删除时精确 tip |
| feat/vsim04-navigation-events (local) | `277a0a157c86bddc107dc103421cd68dc6989f95` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T03:06:50+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T03:06:50+08:00 | refs/tags/archive/recovered-20261008/feat/vsim04-navigation-events | 删除时精确 tip |
| feat/vsim04-navigation-handoff-final (local/origin) | `bdf229693f0747754a3dbda00b38f8aaa339e604` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T19:13:22+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T19:16:16+08:00 | refs/tags/archive/recovered-20261008/feat/vsim04-navigation-handoff-final | 删除时精确 tip |
| feat/vsim04-performance-surface (local/origin) | `d6f4cc372c030558755703464a545e6140821cde` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T01:40:11+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T01:40:11+08:00 | refs/tags/archive/recovered-20261008/feat/vsim04-performance-surface | 删除时精确 tip |
| feat/vsim04-stability (local) | `0deeb1ecaa7c3deaba31c4ba3d98e51ca7429fa3` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-29T19:30:34+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-29T19:30:34+08:00 | refs/tags/archive/recovered-20261008/feat/vsim04-stability | 删除时精确 tip |
| feat/vsim04-video-cd-validation (local/origin) | `96c47fdf99bbe9f35c9e6d69a294c17dab8fb062` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T16:57:05+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T16:57:05+08:00 | refs/tags/archive/recovered-20261008/feat/vsim04-video-cd-validation | 删除时精确 tip |
| fix/pr3-premerge-hygiene (local) | `1a494105064525c29ceb6e51a367f5a0d281ef05` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T19:57:18+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-02T19:57:18+08:00 | refs/tags/archive/recovered-20261008/feat/vcl06-execution-integration/origin | 删除时精确 tip |
| fix/vcl06-sim-lidar-map-readiness (local/origin) | `b83342de6ab5649e57203662910393bd658505f9` | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T13:14:06+08:00 | Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-08-31T13:15:55+08:00 | refs/tags/archive/recovered-20261008/fix/vcl06-sim-lidar-map-readiness | 删除时精确 tip |

## 先前已恢复的两个分支

- `feat/high-view-search-research`：`cefa6b2ae8d69c820b4dcb88b5116a4e0d7b5678`；作者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-10-06T01:56:06+08:00；提交者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-10-06T01:56:06+08:00；直接 ref：`refs/heads/feat/high-view-search-research`、`refs/remotes/origin/feat/high-view-search-research`、`refs/tags/archive/20261008/high-view-search-research`。
- `feat/motion-constraints-20261004`：`d523c53cc5a29f8ec492a2256c2ca2e22e293998`；作者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-10-05T01:12:37+08:00；提交者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-10-05T01:12:37+08:00；直接 ref：`refs/heads/feat/motion-constraints-20261004`、`refs/remotes/origin/feat/motion-constraints-20261004`、`refs/tags/archive/20261008/motion-constraints-20261004`。

详情见 [already_restored.json](already_restored.json)。旧高位和 1004 分支保持各自历史；不凭代码同步宣称祖先继承，不把 461/443 等包含共享祖先的数量相加。

## 工作树资产

| 原始归档目录 | 内容与恢复界限 | manifest / 不同 HEAD |
|---|---|---|
| /home/xhj/liftrace/logs/worktree_archive_20260905/vcl06-sim-lidar-map-readiness | 日志/运行产物；不能仅凭目录名断定完整源码或删除 tip | 6 / 1 |
| /home/xhj/liftrace/logs/worktree_archive_20260905/vcl06-integration | 日志/运行产物；不能仅凭目录名断定完整源码或删除 tip | 16 / 10 |
| /home/xhj/liftrace/logs/worktree_archive_20260905/vcl06-execution-integration | 日志/运行产物；不能仅凭目录名断定完整源码或删除 tip；有 uncommitted_mid360_sim.patch，未提交产物未入 bundle | 9 / 2 |
| /home/xhj/liftrace/logs/worktree_archive_20260905/pr3-premerge-hygiene | 日志/运行产物；不能仅凭目录名断定完整源码或删除 tip；另有 ROS 日志，无 manifest.yaml | 0 / 0 |
| /home/xhj/liftrace/logs/worktree_archive_20260905/vcl06-local-full-mission | 日志/运行产物；不能仅凭目录名断定完整源码或删除 tip | 11 / 2 |
| /home/xhj/liftrace-archives/r2026-competition-integrated_20260915 | 退休工作树保留文件、文档、日志及忽略资产；不保证完整 checkout，原版本依据退休记录 HEAD | 105 / 56 |
| /home/xhj/liftrace-archives/r2026-coverage-efficiency-research_20260915 | 退休工作树保留文件、文档、日志及忽略资产；不保证完整 checkout，原版本依据退休记录 HEAD | 35 / 21 |

[archive_observations.json](archive_observations.json) 按每个 HEAD 保存来源 manifest 绝对路径、字段、原 commit 元数据与现有 ref。12 个原标 exists=false 的对象仅在视觉共享仓不存在；本次指定仓补查已全部在导航 Git 找到，并以 external_sources 补录原作者、日期和现存引用。没有联网 fetch 或修改来源仓。

### 移除 checkout 与退休记录

详情见 [worktree_records.json](worktree_records.json)。工作树退休或删除不等于分支提交消失。

- `/home/xhj/liftrace-worktrees/r2026-main-integration`：HEAD `4fb5ffcf244cc8b4bafe3b8c95d0e993a25edc33`；作者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:06:39+08:00；提交者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:06:39+08:00。
- `/home/xhj/liftrace-worktrees/r64-main-acceptance`：HEAD `ce63b8dda04c7a87c99bbf633d308bfc80f33065`；作者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:52:23+08:00；提交者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:52:23+08:00。
- `/home/xhj/liftrace-worktrees/vdeploy-final-closeout-plan`：HEAD `65172469299733faef3ac060a44cf53030039c55`；作者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:06:39+08:00；提交者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-09T21:06:39+08:00。
- `/home/xhj/liftrace-worktrees/r2026-competition-integrated`：HEAD `86e382d50ffa1d27bce0652ac4d576c73d0bfcec`；作者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T16:20:56+08:00；提交者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-11T16:20:56+08:00。
  归档 `/home/xhj/liftrace-archives/r2026-competition-integrated_20260915` 存在；旧路径当前兼容链接：True。
- `/home/xhj/liftrace-worktrees/r2026-coverage-efficiency-research`：HEAD `4da9614190834bf2af6b34a6a04d6577a01c875c`；作者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-12T07:32:05+08:00；提交者 Qinling-Melon-Farmers <xiaohuaijin@mail.nwpu.edu.cn> / 2026-09-12T07:32:05+08:00。
  归档 `/home/xhj/liftrace-archives/r2026-coverage-efficiency-research_20260915` 存在；旧路径当前兼容链接：True。

20261008 导航资产另存 /home/xhj/liftrace-archives/20261008-worktrees/vcl06-planner-stop-ack.tar.gz、status、HEAD、patch；HEAD 为 c12ee0c3d26077d5c778719a9006c8fa6755507b，归属导航仓。本次仅读取已有 HEAD/记录与归档文件名，不解包，不给未提交产物赋予 Git 作者时间，不把导航仓独立成果算入本视觉 bundle。

## 离线 bundle 与验证

- 文件：`/home/xhj/liftrace-archives/20261008-history/liftrace-history-20261008.bundle`。
- 大小：**274,511,258 字节（261.79 MiB）**。
- 快照时刻：2026-10-07T17:08:01.538787+00:00（UTC）；后续主代理提交不会自动纳入；**此 bundle 不包含随后完成的 main@6cd0207fb5ea669416d01c06d6a1b65eac6730e9**。
- 将全部 107 个提交历史 ref 按原对象 ID 拷入仓库外 export.git，再执行 git bundle create ... --all；包含分支、远端跟踪分支、原 annotated tag、新保全 tag 与 refs/original/*。
- 共享仓的 3 个 Codex 临时 tree ref 不是已提交历史；独立小仓验证 root --all 会纳入其未提交树。因此只在导出快照中排除，原 ref 未动。见 [excluded_noncommit_refs.json](excluded_noncommit_refs.json)。
- 不复制工作区、index、reflog、Git config、credential helper 文件或凭据目录。仅检查历史对象路径未出现常见明确凭据文件名；没有做全历史内容的秘密审计，也不据此宣称历史内容已获全面审查。
- git bundle verify 在空 bare 仓通过；从 bundle 导入无 alternates 的独立 verify.git 成功，107 个 ref 名与对象 ID 完全一致；38 个历史点及 2 个已恢复 HEAD 的原 commit 字节一致。
- 仓外保存 bundle_create.log、bundle_verify.log、bundle_import.log 和 refs_snapshot.json。export.git 借用原仓对象，不能单独当离线备份；bundle 与独立 verify.git 可离线读取。
- --all 只保全导出 ref 可达的历史，不代表所有无 ref 的悬空对象；本次已找回点均可达。日志、模型、工作树未提交文件及导航归档 tar 不装入 bundle。

## 作者日期、归属与工作量边界

[commits.jsonl](commits.jsonl) 按 SHA 去重记录 bundle 中全部 **908 个提交**的原作者/邮箱/ISO 日期、提交者/邮箱/ISO 日期、父提交与标题。[bundle_refs.json](bundle_refs.json) 保存完整 ref 对象映射；[tags_created.json](tags_created.json) 是新增标签清单。新增 tagger 和保全日期属于本次动作，原 commit 的作者时间保持原样。

本轮工作为 36 个删除分支名的精确定位、1 个观察点别名、7 个资产目录索引、5 条 checkout 记录核查、35 个本地标签和一次离线导出/验证。908 个去重提交是保全历史范围，不是本轮新增开发或个人贡献量；没有把分支可达祖先数量相加。没有新增运行功能代码或 Git commit。

## 本轮变更记录

见 [CHANGE_RECORD.md](CHANGE_RECORD.md)。用户限定文档独占本新目录，因此不改 root 全局台账、不改 B 其他文档、不自动提交。主代理 review 后决定提交和 push。

## 并发工作区观察

本轮未删除任何既有 ref。前后快照有 2 个远端跟踪 ref 发生变化：origin/feat/board-deployment-flight-20260920、origin/feat/ev-continuity-20261004；最新 reflog 都是 update by push，属于其他并发代理的操作，本轮未执行 fetch/push，不回退其更新。bundle 快照已包含这两个更新后的 ref，末检与现状一致。root 仍为 feat/r2026-main-integration。

summary_draft.json 与 proposed_tag 字段保留计划阶段信息；最终数量以 summary.json 为准，实际承载以 preservation_refs/current_containing_refs 和 tags_created.json 为准。37 项初始 tag 计划中，两个重复提交复用引用，实际新增 35 个。

## 2026-10-08 指定来源仓只读补查

此前视觉共享 Git 查无的 **12/12 个 commit 均在 /home/xhj/liftrace-controlwork-nav 的导航 Git 中存在，且有现存 ref 承载；没有找不到的对象**。/home/xhj/liftrace-sim 中这 12 个 SHA 均不存在。这不改变视觉 bundle 的边界，导航对象未导入视觉仓，也未追加到 bundle。

只检查用户指定目录。导航两个现存工作树 drop-height-board-reference-20261003、high-view-liveness-20260919 与 /home/xhj/liftrace-controlwork-nav 共用 /home/xhj/liftrace-controlwork-nav/.git，去重后查询一个导航对象库；另查一个 liftrace-sim 对象库。没有联网 fetch、push、工作树切换或新增 ref。

完整查询路径、cat-file 结果、原作者/提交者/ISO 日期/父提交、直接 ref 与包含 ref 见 [cross_repository_check.json](cross_repository_check.json)。下表列的是现存包含引用（提交可能是其祖先，不把它当作 ref 的 tip）。

| 完整提交 | 源 Git 仓 | 现存包含 ref |
|---|---|---|
| 0e2d57a2b0e4ef637bb71095e2e6288580556634 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| 1928f75fdba6ea0ffcd645f74351d2f6a2b95497 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission；refs/remotes/origin/feat/vcl06-local-full-mission |
| 22ddb8af08363bc22e6a45c944a18b4eb3e5aa07 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| 41e5b3319aac782f0ee491d79d638e56287722e2 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission；refs/remotes/origin/feat/vcl06-local-full-mission |
| 5f06918de66fe5aa62a3ce6fc3f286c36ad13ef9 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| 63e838da72e2604530f3db6f609cfdc5322f27f9 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| 666c880cb5556c36329b580b4db604c293799cbc | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| 957dd371b3a81ee55f029669790bd53b34b5e76a | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| afad81a37a0d7e64f15d2b6850dac932a515cb2f | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| b69ba72f06281621c5b10e440a3a94f5cf5e4642 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission；refs/remotes/origin/feat/vcl06-local-full-mission |
| d883b7357012fcc54339e0fc811faf0ba6510857 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |
| fbaf58b4630ffa21c810636a938a3588d68f0550 | /home/xhj/liftrace-controlwork-nav/.git | refs/heads/feat/vcl06-local-full-mission；refs/remotes/fork/feat/vcl06-local-full-mission |

### bundle 快照边界补充

现有 bundle 的导出快照时刻是 2026-10-07T17:08:01.538787+00:00（UTC），当时 refs/heads/main 为 2f405bef1b980fc8ba7b99958185538f660e28ee。主代理之后完成的 **main@6cd0207fb5ea669416d01c06d6a1b65eac6730e9 不在此 bundle 中**，已在原独立 bundle 导入仓用 cat-file 确认对象不存在。未重建 bundle，未修改其导出 ref 清单、原大小或原验证记录。

本次仅修改本报告目录；35 个保全 tags 留由主代理推送。补查完成后停止。


## 主代理发布补记

2026-10-08：35个本地保全annotated tag已原样推送origin（archive/recovered-20261008/前缀），旧高位/运动约束原分支仍在；本报告由试飞分支发布。未删除/重写历史。离线bundle仍保持原导出时刻，后续main6cd0207f另已在本地根工作区和远端一致。
