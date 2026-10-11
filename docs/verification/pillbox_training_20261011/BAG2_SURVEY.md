# 第二轮为何没有提前结束高位搜索

2026-10-11，只读`flight_debug_0_10111009_2.bag`既有提取。时间为相对bag起点。

结论：误识地堡产生第二处panzer，触发任务记忆的panzer冲突挂起。提前结束要求bridge、panzer、red_cross三类未冲突线索同时齐备；不是简单数原始检测框。

| 时刻 | 实录字段与事件 |
| --- | --- |
| SURVEY全程 | required_classes为bridge、panzer、red_cross |
| 114.366s | navigation_conflicted_classes=[panzer]，记录navigation_evidence_conflict |
| 同时 | panzer ID3约(5.713,-1.157)、ID5约(4.412,-0.056)，source=vision，支持计数8/3；ID5对应误识的地堡位置 |
| 115.934s | red_cross已进入navigation_support，source=vision |
| 117.001s | bridge、red_cross都有vision支持，panzer仍挂起 |
| 136.389/136.778s | 进入DESCEND/发出下降指令；未记录SURVEY_INTERRUPTED_TOP3 |

真实bridge位置约(3.502,-1.317)仍有vision支持，不能归因bridge被全部剔除；SURVEY没有hint_expired记录。survey_progress_m=0.15用于停滞监视，不是最低巡查进度门槛。first_hint_ready与阶段top_hints空字段也不能直接当作最终中断资格。

来源方法为uav_mission/high_view_full.py的中断集合检查、uav_high_view/navigation_memory.py的冲突挂起与interrupt过滤，读取导航38647c92相关实现与B一致；旧部署manifest不能证明完整板端revision，但关键冲突状态与未提前中断是bag实录。

这是分类错误向任务时间传导的表现：正确桥梁和红十字已存在，错误第二panzer仍会妨碍中断。不能为了节时取消冲突保护，把两个同名物体或同一物体多类输出当作三类齐全；应优先减少地堡/帐篷冒充高分类。当前未改HINT阈值、冲突处理或现场模型。
