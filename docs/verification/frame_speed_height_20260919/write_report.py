"""Build the completed experiment report from archived measurements."""
from pathlib import Path
import json
D=Path(__file__).resolve().parent;R=D.parents[2]
ms=json.loads((D/'summary.json').read_text());A,B,C=ms
align=json.loads((D/'alignment_history.json').read_text());sequence=json.loads((D/'landing_sequence.json').read_text());ground=json.loads((D/'landing_ground_stats.json').read_text())
def f(v):return '未完成' if v is None else f'{v:.3f}'
def tail(m):return sum(m['milestone_durations_s'][k] for k in ('last_release_recovery','fast_transfer_to_staging','staging_descent','corridor_to_H'))
gain=tail(A)-tail(B);pct=100*gain/tail(A)
lines=['# 固定起飞系：走廊提速、3m搜索与降落复盘','',
       '**本批完整工程Gate为1/3通过：C完成198.615秒；A、B在H着陆阶段失败。走廊组合确有阶段节时，3m没有体现额外搜索节时；尚不能定为最终稳定部署方案。**',
       '',f'第三投到发出H降落指令：A {tail(A):.3f}s → B {tail(B):.3f}s，减少{gain:.3f}s（{pct:.2f}%）。C三投比B晚{C["third_commit_s"]-B["third_commit_s"]:.3f}s；C唯一完成不代表升高修好了降落。',
       '', '[图表与校正录像总览](index.html) · [XY轴/初始机头截图](frame_axes.png) · [全部指标](summary.json) · [原始run索引](runs.json)',
       '', '## 1. 比较口径与实际结果','',
       '导航来源`liftrace-controlwork/feat/high-view-liveness-20260919@2b29a5b`；整机飞行源码三轮均为`feat/high-view-search-research@7c00d7b`。坐标场景工具来自轻量fork`74d101a`。后续提交仅记录本轮结果与录像后处理；未合并main、未部署机载。',
       '', '三轮均为seed2672冻结靶位、同一连续80cm门/树布局、4m外围墙、1.5m内部墙、同一模型与低空门控。A为2.6m+原低走廊；B为2.6m+0.9m走廊/门间提速；C只在B上把高位设为3m。B的高度与速度共同改变，不能拆称纯速度消融。',
       '', '| 方案 | Gate | 三投提交/s | 到H降落指令/s | 完整验收/s | 最终guard墙接触 |', '|---|---|---:|---:|---:|---:|']
for m in ms:lines.append(f'| {m["label"]} | {m["status"]} | {f(m["third_commit_s"])} | {f(m["land_command_mission_s"])} | {f(m["completed_mission_s"])} | {m["collisions"]} |')
lines += ['', '三轮均完成三投、三次恢复、9/9投后点和两门。A的Gate首因是0.7m区域限高；约41ms后守护盒接触Wall_9。B首因为守护盒接触Wall_11。后续落地/解除武装/任务完成未满足项不代表多个独立故障。C所有现有Gate检查通过，但整机越树顶复核仍有下述保守投影疑点。',
          '', '旧286.456s是旧地图/门位/朝向下的历史成功，仅作项目参考。C比该数值少87.841s（30.66%），**不是同地图单因素完赛增益**；新地图A、B没有成功完赛时间，不能计算成功整场配对节时。开发中止轮另列[excluded_runs.json](excluded_runs.json)，不替换失败。',
          '', '![耗时对比](comparison.png)', '', '![阶段分解](stage_comparison.png)',
          '', '## 2. 节时具体来自哪里','', '| 阶段/s | A | B | C |', '|---|---:|---:|---:|']
names={'last_release_recovery':'第三投ACK后恢复','fast_transfer_to_staging':'快速转场至走廊准备点','staging_descent':'准备点下降','corridor_to_H':'进廊至H降落指令'}
for key,label in names.items():lines.append('| '+label+' | '+' | '.join(f(m['milestone_durations_s'][key]) for m in ms)+' |')
lines += ['', 'A最后H前航点初次规划等待31.720s、最多36次尝试，是其尾段慢的重要组成。该问题属于首条轨迹生成，不是已有轨迹的投影停滞。B/C高度窗口改变后该航段明显缩短。12s新等待上限只用于SEARCH/RESUME，不能声称所有RETURN_HOME等待都已限定12s。',
          '', '| 实际移动速度中位数(m/s) | A | B | C |','|---|---:|---:|---:|']
for label,keys in [('普通巡航',['CRUISE']*3),('走廊空旷段',['CORRIDOR','CORRIDOR_OPEN','CORRIDOR_OPEN']),('门口慢档',['CORRIDOR','DOOR','DOOR'])]:
    lines.append('| '+label+' | '+' | '.join(f(m['following_speed_stats'].get(k,{}).get('median_mps')) for m,k in zip(ms,keys))+' |')
lines += ['', '0.40m/0.15m是跟踪前视距离，不是m/s。全场巡航规划上限仍为1.2m/s、前视1m；投递与恢复没有为提速而缩短。门间按位置提速，离门0.75m进入慢档、离开0.95m后恢复快档；H前0.8m和入口下降保守。A/B前段参数相同，三投时间约13.4s差异不归因于投后走廊配置，单次视觉/规划时序波动必须保留。',
          '', '## 3. 冲突线索修复的验证范围','',
          '开发轮首次panzer线索距pillbox模型中心仅1.6cm，距真正panzer约1.58m。修复三轮均再次出现两处不同ID的panzer位置，一处近pillbox、一处近真正panzer，支持分类混淆而非整个地图旋转错轴。没有保存开发轮误分类瞬间的原始机载帧，不能把旋转敏感性、边缘裁切或关联过程中的某一个单独认定为唯一原因。',
          '', '实现保留每类最多两处冲突位置；从合格高位线索中隔离，但用于低空局部复核。单点运动上限25s、重捕上限15s；开始复核75s后不再发新复核段，最后重捕至多再15s；600s总deadline、已投槽位和新鲜释放门控不重置。不可达/错类/超时换另一处，耗尽才进入补搜。',
          '', '**本组三轮均先访问了较近的正确panzer点，并通过低空重捕完成第三投；没有进入LOW_COVERAGE。** 因已确认正确点，不再飞另一处。先访问错误点、再换第二点的分支有单测，尚没有本批实跑样本；补搜末段近墙减速也未因本组三轮绕开补搜而得到动态覆盖，不能声称边界停滞已彻底实跑修复。',
          '', '高位类冲突仍阻止“合格三类齐备”提前中断，所以本组三轮未产生TOP3提前中断。当前先处理可靠线索，再复核歧义位置；没有进行包含歧义假设的整任务最短时间排序。低空类别仍依赖现有候选与释放链，并非重新训练了分类器。',
          '', '## 4. H升高、触地反弹与碰撞判定','',
          '**用户提出的“先落地再突然抬起”得到日志支持；但未发现代码把地面接触直接计成墙碰撞。** `contact_policy.py`按对象名称排除ground_plane与两个H垫，保留的是`competition_guard_collision`对Wall_9/Wall_11的接触。该55cm盒本身已经包含鲁棒膨胀，视频中较小的可见机身不一定已碰墙；应称“仿真守护碰撞盒接触墙”，不等同实机外廓碰撞。',
          '', '| 方案 | 首次接近支撑高度/ROS s | 随后墙接触/ROS s | 接近地面向下速度峰值(m/s) | 接近地面后的回弹速度峰值(m/s) |', '|---|---:|---:|---:|---:|']
for seq,g in zip(sequence,ground):
    contact=seq['contact_events'][0]['ros_stamp'] if seq['contact_events'] else None
    lines.append(f'| {seq["label"]} | {f(seq["first_near_support_ros"])} | {f(contact)} | {abs(g["descent_min_vz_mps"]):.3f} | {max(0,g["rebound_max_vz_mps"]):.3f} |')
lines += ['', 'A/B的FC最低高度约0.229m，守护盒底面到达约5mm H垫表面，然后在约0.8–0.9秒内出现抬升和墙接触。原始地面接触点/法向力/冲量没有归档，因此几何接近支撑面和回弹轨迹不能单独证明冲击是唯一根因。C以更快的接近地面速度成功停住，说明“下降快”不是充分解释。',
          '', 'PX4 ULog与ROS位姿末段拟合时钟误差小于毫米量级位置残差，证实A/B此时任务轨迹仍指令下降，NED向下速度达到0.7m/s，不是主动CV升高。不能用AUTO.LAND之后仍发布的外部ROS位置命令代替PX4内部实际任务目标。[时间顺序与内部设定点](landing_sequence.json)。',
          '', '**H识别合法升高应有独立阶段约束。** 当前区域限高不区分LAND/H识别状态，设计上存在把合法识别升高与穿门高度混用的风险；但A这次超限发生在自动下降/反弹过程，并非新下发合法识别升高。不能因此追改A为PASS，B放宽到1.2m后仍发生盒-墙接触。',
          '', '下一步优先做有限范围的近地检查：记录地面接触摘要、PX4落地判据/推力/高度创新、LIO与飞控估计；用同一H接近条件对比更缓的AUTO.LAND末段，区分接触动力学与估计/控制反馈。H识别高度应独立配置，只在明确H安全区域及识别阶段允许，异常反弹仍单列。此阶段未修改降落控制、判碰过滤或门槛来掩盖失败，未新增仿真。',
          '', '![接近支撑面及升降指令](landing_ground_sequence.png)', '', '![55cm盒与墙体局部关系](landing_contact_xy.png)',
          '', '| LAND期间（接触前；C为完整LAND窗口）最大误差/m | LIO XY | LIO Z | 飞控XY | 飞控Z |','|---|---:|---:|---:|---:|']
for m in ms:
    p=m['landing_estimation'];lines.append(f'| {m["label"]} | {p["lio_pose"]["pre_contact_xy_error_max_m"]:.3f} | {p["lio_pose"]["pre_contact_z_error_max_m"]:.3f} | {p["mavros_pose"]["pre_contact_xy_error_max_m"]:.3f} | {p["mavros_pose"]["pre_contact_z_error_max_m"]:.3f} |')
lines += ['', 'C虽成功，近地飞控估计也并非完全无误差。A/B ULog确认EV_CTRL=9、HGT_REF=0、GPS_CTRL=0、BARO_CTRL=1实际生效；启动回读文件的success=false不能证明参数未设置。降落时未出现新的XY/Z/yaw重置，但不能仅凭计数排除历史重置处理或其他估计/控制问题。',
          '', '## 5. 历史投递对齐精度','',
          '从R64、全随机31–35、32/34四轮对照、旧seed2672及本次三轮的原始日志，去重重算69次成功模拟释放。以ACK时间戳插值真实FC位置，与各run真值文件中的靶标模型中心比较；径向距离可跨旋转坐标系比较。它不是逐帧人工标注的图案中心误差，也不是载荷释放口/真实快递落点误差。',
          '', '| 批次 | 次数 | 中位数/cm | P95/cm | 最大/cm |', '|---|---:|---:|---:|---:|']
for name,s in align['groups'].items():lines.append(f'| {name} | {s["n"]} | {100*s["p50_m"]:.2f} | {100*s["p95_m"]:.2f} | {100*s["max_m"]:.2f} |')
s=align['all'];lines += ['', f'合计中位数 **{100*s["p50_m"]:.2f}cm**、P95 **{100*s["p95_m"]:.2f}cm**、最大 **{100*s["max_m"]:.2f}cm**。{s["within_5cm"]}/{s["n"]}在5cm内，{s["within_10cm"]}/{s["n"]}在10cm内，{s["within_15cm"]}/{s["n"]}在15cm内。不能宣称稳定的厘米以内对准。之后应分别校核图案几何中心、低位精修与实际释放口杆臂/机构偏置；高置信度不等于高落点精度。',
          '', '本次每投误差见下表（cm）：','', '| 方案 | 目标 | 机心到模型靶心 |', '|---|---|---:|']
for m in ms:
    for c in m['commit_times']:lines.append(f'| {m["label"]} | {c["target"]} | {100*c["fc_target_distance_at_ack_m"]:.2f} |')
lines += ['', '[全部69条样本及分组](alignment_history.json) · [计算脚本](alignment_history.py)', '', '![历史对齐距离](alignment_history.png)',
          '', '## 6. 为什么原始视频只有约140秒','',
          'C原始follow.mp4为149.3s、overview.mp4为149.4s。录制器固定按10fps编码，实际保留的图像并非每0.1仿真秒一帧，没有把缺少的时间空档补回，所以直接播放会压缩时间。不能由原始视频长度推算完赛速度。',
          '', '**已校正C双视角录像为209.1s（3分29.1秒）**，覆盖ROS 1.247–210.342s，包含启动准备。任务首指令到完整验收为198.615s（3分18.615秒）。第三投约101.15s，剩余到完成约97.465s，整场不是两分多钟。三份presentation.mp4均依据图像ROS时间补帧保持，加入阶段、高度、投递数与最终Gate标签。',
          '', '若按每投额外预留10s机构预算，在C成绩上加30s约为228.615s（3分48.615秒）；只是暂定预算，不是实机成绩或保证。当前raw mock ACK没有真实快递弹道/风扰/机构延迟。70s粗成本在early_return关闭时不是强制的释放前10s准入，后续机械完成反馈仍须对接。',
          '', '## 7. 相机方向按用户最新确认处理','',
          '固定起飞系：新X=旧Y，新Y=-旧X，Z向上，初始机头+X。相机可机械调整安装朝向，因此最终方案按原先FOV方向配置，不把本次未转相机得到的82.2%/88.7%当作必须接受的限制。',
          '', '恢复原先相对场地视角可在新机体系内将相机安装yaw调整−90°，同时更新仿真安装姿态、ROS相机外参和像素到机体系映射；仅机械转动且保留相机原始像素坐标时，内参K/D不因换机体系而交换。旧像素方向矩阵[[0,-1],[-1,0]]对应新安装应按旋转得到[[-1,0],[0,1]]。必须先做几何校验，不能只把显示图像旋转90°。本次只是确认后续安装基准，三轮记录保留当时实际朝向，不追改成绩、不在本次检查中追加飞行。',
          '', '原方向的理想覆盖约96.8%/99.7%（2.6m/3m）；当前三轮实际未转相机，理论为82.2%/88.7%。这些是无倾斜/无遮挡/完整名义航线的几何覆盖，不是识别召回。C目标高度3m的实测峰值3.216m；若3m被作为硬上限，当前3m设定不能称满足该硬上限。',
          '', '![坐标与初始机头](frame_axes.png)', '', '![视场方向几何对照](frame_fov.png)',
          '', '## 8. 不越树顶、走廊高度与资源边界','',
          '未取消全高障碍柱。三轮机心越树轮廓采样为0，但55×55×40cm已膨胀盒与树箱保守凸包投影存在重叠。A/B/C最深约9.36/6.05/10.83cm，不能等同此前毫米级边缘误差，也不能据现有Gate PASS宣称整机不越树顶已完整验收。它是保守投影指标，不是高空物理碰撞；需对照实际外廓/树形与跟踪余量，优先研究近障碍弯道前视或速度，而不是全局继续膨胀到封死80cm门。',
          '', '![最紧树边投影](body_projection_closeups.png)',
          '', '| 方案 | 走廊最高FC/m | LAND前最大合成倾角/度 | 走廊最高守护盒顶/m |', '|---|---:|---:|---:|']
for m in ms:
    g=m['corridor_geometry'];lines.append(f'| {m["label"]} | {g["max_agl_m"]:.3f} | {g["cruise_max_combined_tilt_deg"]:.2f} | {g["max_guard_top_agl_m"]:.3f} |')
lines += ['', 'B/C走廊巡航满足本次0.9m目标/1.2m验收窗口，守护盒顶均低于1.5m内部墙高。仍保留55cm鲁棒盒，没有按裸机尺寸缩小判碰。新增候选至多每类两点，不运行额外大模型；全组三轮共同采用22×12×3.8m、5cm地图，体素数较原3m高地图增加26.7%，需要后续端侧评估；这不是板端实跑。日志保留关键数据和两机位，无全场bag。',
          '', '## 9. 图表、录像与复现','',
          '323项完整任务测试、70项高位库测试、79项轻量场景测试通过；此前整机构建通过。三轮单实例运行及最终清理均确认零ROS/Gazebo/PX4/RViz残留。影像质量与链接校验见[验证清单](validation.json)。']
for m in ms:
    folder='2672_'+m['label'];rel='../../../logs/'+Path(m['run']).name+'/presentation.mp4'
    lines += ['', '### '+m['label'], '', f'[按ROS时间校正的双视角录像]({rel}) · [指标]({folder}/metrics.json) · [Gate]({folder}/gate_status.json)', '', f'![完整航迹高度速度]({folder}/flight_charts.png)', '', f'![按阶段航迹]({folder}/route_stages.png)', '', f'![阶段与目标事务]({folder}/phase_target_timeline.png)', '', f'![跟踪档位与实速]({folder}/speed_profile.png)', '', f'![三维轨迹]({folder}/route_3d.png)', '', f'![高度和姿态]({folder}/height_tilt.png)', '', f'![近地估计]({folder}/landing_diagnostic.png)']
lines += ['', '优先顺序：先区分近地接触/估计/落地检测问题，再核对实际整机越树余量与对齐误差；保留走廊阶段收益，按用户确认恢复原相机FOV安装。当前证据不支持为了提速默认升到3m，也不足以宣布最终稳定方案或替换正赛部署。']
(D/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
cards=[]
for m in ms:
    folder='2672_'+m['label'];rel='../../../logs/'+Path(m['run']).name+'/presentation.mp4'
    cards.append(f'<section><h2>{m["label"]} — {m["status"]}</h2><video controls preload="metadata" src="{rel}"></video><p><a href="{folder}/metrics.json">完整指标</a></p>'+''.join(f'<img loading="lazy" src="{folder}/{name}.png">' for name in ['flight_charts','route_stages','phase_target_timeline','speed_profile','route_3d','height_tilt','landing_diagnostic','progress'])+'</section>')
html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>固定起飞系三轮对比</title><style>body{max-width:1400px;margin:30px auto;padding:0 20px;font:17px/1.7 sans-serif;background:#f4f6f8;color:#203044}img,video{display:block;width:100%;margin:20px 0}section{border-top:2px solid #bccbd5;margin-top:45px}a{color:#165aa2}.note{padding:18px;background:#fff1d5}</style><h1>固定起飞系：走廊提速与3m对比</h1><p>C工程Gate PASS，198.615秒；A/B在H降落阶段失败。投后至H指令缩短44.8%，3m未显示额外搜索节时。</p><p class="note">C的工程Gate通过不等于严格整机越树顶复核通过；55cm包络仍有保守投影重叠。原始149秒视频不能计时，请使用下方ROS时间校正录像。</p><p><a href="REPORT.md">完整报告</a> · <a href="alignment_history.json">69次历史对齐数据</a> · <a href="landing_sequence.json">H降落时间顺序</a></p>'''
html+=''.join(f'<img src="{name}.png">' for name in ['frame_axes','comparison','stage_comparison','paired_paths','alignment_history','landing_ground_sequence','landing_contact_xy','body_projection_closeups','frame_fov'])+''.join(cards)+'</html>'
(D/'index.html').write_text(html,encoding='utf-8')
print('REPORT.md and index.html written')
