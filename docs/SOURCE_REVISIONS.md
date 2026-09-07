# 源码与交付版本

| 用途 | 实际修复/验证提交 |
|---|---|
| 精简整机实际成功运行 | cc899f2bc4816caff9f9059adb0208705476f81b |
| 导航来源 | 286322f9b03fdcdab9b6b3efad1f02df4d3ca61c |
| 视觉来源 | 0704d50fbe3eac7b90230dc3c00fca749b8a814e |
| 保留原始资产的集成修复 | 5a472a3455f229a78cd707037dd26d64bb7b0308 |

报告/归档与合并提交晚于实跑，不能当作飞行源码。main 通过 feat/r2026-main-integration 的 --no-ff 合并接收已验证内容；对应 gate/vcl06-r56-full-competition，所有功能分支保留。完整成功源另由 sim/r2026-r56-contact-map-pass 标记。

I/O 中断与失败尝试不重写为 PASS；原先 main 的 44359e8ba426ed91ef42c4951b5ab2fae8924027 是此次合入前基线。最新交付状态以远端标签/分支和 [交付说明](verification/r56_final/DELIVERY.md) 为准。

精简导入起点为导航 987b57c6、视觉 1b328c13，仅作历史来源；后续 R52—R56 修复已更新其运行代码。原始机载资产与冻结旧包留在来源分支和 main 集成分支，精简分支不带旧包快照或机械 PWM 实现。
