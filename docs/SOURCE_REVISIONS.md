# 源码与验收记录

当前R62阶段运行源码8bedcc0c376b815eb3590e740ee9634210f5c3f5：seed11建图/起飞/搜索/panzer首投/恢复已实跑。run r2026_r62_operational_seed11_20260909_030841；此后为回归/文档/打包工具，未改飞行行为。完整Gate未完成，不合main。

R61源码提交：`7b76517`（回退/实测几何/规则场地）、`a627170`（CAD装配纠正）、`140623f`（支架可见连接）；随后为报告、模型依赖与两类打包工具。均无新的飞行验收。最终本地包以各自BUNDLE_MANIFEST中的完整revision为准。R61静态预览run为r2026_r61_seed11_final_preview_20260909_000031。

| 用途 | 实跑源码/版本 | 结论 |
|---|---|---|
| R56 | cc899f2bc4816caff9f9059adb0208705476f81b | 历史完整37/37 PASS |
| R57/0.85m | 703541505f8dac5f2db7be333eda9c2460ed3d70 | 完整37/37 PASS |
| R58最后轮 | 59ed04b8d310af141670cb47ba1078759c8e02eb | 三投后外墙接触FAIL |
| R59 | e633a9be9fb1232e0b2bcdba5d6b68fb0d61bfd6 | 专项到触垫，原始FAIL，最终disarm未确认 |
| R60先导与十seed | 9cfb3e56b11af23b4bc764b48e53454b32d015dc | 先导PASS；十seed2/10完整PASS |
| R60 main集成源码镜像 | 7379ef5ba30a0a6709e9b5b6b0f20aa0880f2b27 | 对应R60功能源码，未合main |
| R60导航分支源码镜像 | e4a4cc6b631423d9f92daaf6151986241e78c9af | 已推导航fork现有分支 |
| main验收基线 | 31d0b2aaa2c5bd9f9554008e42301cfbde666cfa | 保留R56验收及模型说明，本轮不更新 |

十seed飞行期间未更改HEAD/飞行配置；其后提交为报告、图形与记录工具，不冒充实跑源码。R56后17次提交/逐文件参数对照见[修订清单](verification/r60_full_matrix/revision_inventory.json)。

精简分支feat/r2026-competition-integrated包含导航+视觉且无机械PWM与旧参考副本；原始资产和legacy快照在来源分支。新旧功能分支、历史Git标签继续保留，本轮没有删除分支/标签，也没有为2/10创建main验收标签。
