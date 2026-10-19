"""板端导航前视生成策略（2026-10-13）。

仍读取、校验旧现场 YAML 中的 following_speed_profile/corridor_speed_schedule；
本轮生成策略统一输出两档：巡航 1.0m，其余普通导航 0.6m。旧 YAML 显式写入
0.2/0.4 等低档，也只在新生成结果中转换；不自动写回或覆盖现场源 YAML。

BOUNDARY_REVISIT、LOW_COVERAGE、普通返航、走廊下降/门区/近 H 导航使用 0.6m。
投后前往走廊入口的首段巡航仍为 1.0m，入口到达后的下降和走廊为 0.6m。
low_coverage_lead_m 仅由本轮板端生成启用；历史运行入口缺省时保持原行为。

单位是规划跟随距离，不是速度或首轨迹接纳距离。manager 在 ALIGN/LAND 也会
写入通用 /px4_max_distance=0.6；普通 Aligning 三维位置控制（包括直接下降）
会受该限幅影响，independent_landing 有例外，不能称 H/投递直接控制完全不受影响。
高度、槽位补偿、释放阈值和独立下降速度参数均不修改；两档对起飞及直接下降
的响应变化仍待动态验收。坐标和门锚点保留。同步到其他分支仅应用
差异补丁，不拷贝整份 field.yaml/competition_speed.yaml 覆盖分支地形或高度。
"""
from dataclasses import asdict, replace
from uav_mission.execution_speed import FollowingSpeed
from uav_mission.corridor_speed import CorridorSpeedConfig


def navigation_following_profile(values=None):
    profile = FollowingSpeed(**(values or {}))
    return asdict(replace(profile, cruise_lead_m=1.0, precision_lead_m=.6,
                          boundary_lead_m=.6, corridor_lead_m=.6, low_coverage_lead_m=.6))


def navigation_corridor_schedule(values):
    CorridorSpeedConfig(**values)
    return {**values, 'open_lead_m': .6, 'door_lead_m': .6}
