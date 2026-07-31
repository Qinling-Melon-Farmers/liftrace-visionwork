#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compute_free_waypoints.py
=========================
解析 toudi3.world（toudi2 走廊主体 + 箱子 + 树 + 标准靶），输出：

  1) 场地边界（四面外墙 AABB）
  2) 每面墙（含 Wall_15/20/22 的分段碰撞，带各自 z 范围）的绝对几何
  3) 半高障碍（big_box 箱、松树）平面位置
  4) 门洞 / 通道 / 出口分析（无人机可通行性判定）
  5) 推荐无障碍航路点（起飞 / 检测 / 穿越 / 返航 / 降落），逐点验证安全余量
  6) ASCII 俯视图（x 向右，y 向上）

只读解析 world 文件，不修改任何工程代码。
坐标系：Gazebo 世界系（x 东，y 北），单位米。

几何约定：
  - toudi2 墙：多数碰撞为 0~2.5 m 全高；Wall_15 有半高段
    （z∈[0,0.5] 低墙、z∈[1.3,2.5] 上段、z∈[0.5,2.5] 中高段），
    Wall_20/22 顶部有 z∈[2.0,2.5] 横梁。飞行高度上限 1.7 m 时，
    横梁与 0~0.5 m 低墙不构成障碍，其余按平面障碍处理；
  - big_box 箱：size 1.2 x 0.8 x 1.25，z∈[0,1.25]，半高障碍
    （飞行高度 >1.3 m 可飞越，否则按平面障碍绕开）；
  - juniper_Tree：pine_tree.dae 原始网格 x/y 跨距 124.9（单位 1 BU = 1 in），
    world 缩放 0.2 0.2 0.3 -> 树冠直径约 0.63 m、树高约 1.5 m
    （模型注释 "approx 5ft tall, 20 inches diameter" 印证）。
    保守取直径 0.8 m（半径 0.4）、高 1.5 m，视为半高圆形障碍；
  - 标准靶（dibao/qiaoliang/tanke/zhangpeng/zhuangjiache）：贴地薄板，
    对无人机无碰撞，仅记录坐标（投递目标点）。

无人机参数（任务描述）：机身宽约 0.5 m（半径 0.25），飞行高度 0.65~1.7 m，
机身垂直半高按 0.15 m。安全判据（机体边缘到障碍）：
  - 严格档：>= 0.5 m 余量（航点中心到障碍 >= 0.75 m）
  - 安全档：>= 0.3 m 余量（航点中心到障碍 >= 0.55 m）
  - 最小档：仅能通过（航点中心到障碍 >= 0.30 m）
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# ----------------------------------------------------------------------------
# 配置
# ----------------------------------------------------------------------------
WORLD_PATH = Path("/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world")

UAV_RADIUS = 0.25        # 机身平面半径
UAV_HZ = 0.15            # 机身垂直半高
FLY_Z_MIN, FLY_Z_MAX = 0.65, 1.7

MARGIN_STRICT = 0.5      # 机体到障碍 >= 0.5 m（推荐）
MARGIN_SAFE = 0.3        # 机体到障碍 >= 0.3 m（安全）
D_STRICT = UAV_RADIUS + MARGIN_STRICT   # 0.75 m（中心距）
D_SAFE = UAV_RADIUS + MARGIN_SAFE       # 0.55 m
D_MIN = UAV_RADIUS + 0.05               # 0.30 m

TREE_RADIUS = 0.4        # 保守（真实约 0.32）
TREE_HEIGHT = 1.5
BOX_ZMAX = 1.25


# ----------------------------------------------------------------------------
# 工具
# ----------------------------------------------------------------------------
def parse_pose(text):
    """'x y z rx ry rz' -> (x, y, z, yaw)。本场景 roll/pitch 均为 0。"""
    v = [float(t) for t in text.strip().split()]
    return v[0], v[1], v[2], v[5]


def rot_xy(dx, dy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c - dy * s, dx * s + dy * c


def box_aabb(cx, cy, sx, sy, yaw):
    """中心 (cx,cy)、尺寸 (sx,sy)、偏航 yaw 的 box 的世界 AABB。"""
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx = (sx * c + sy * s) / 2.0
    hy = (sx * s + sy * c) / 2.0
    return cx - hx, cx + hx, cy - hy, cy + hy


def dist_point_rect(px, py, xmin, xmax, ymin, ymax):
    dx = max(xmin - px, 0.0, px - xmax)
    dy = max(ymin - py, 0.0, py - ymax)
    return math.hypot(dx, dy)


def dist_point_circle(px, py, cx, cy, r):
    return max(0.0, math.hypot(px - cx, py - cy) - r)


# ----------------------------------------------------------------------------
# 障碍物
# ----------------------------------------------------------------------------
class Wall:
    """墙段：AABB + 垂直范围 [zmin, zmax]。"""
    def __init__(self, name, aabb, zmin, zmax):
        self.name = name
        self.xmin, self.xmax, self.ymin, self.ymax = aabb
        self.zmin, self.zmax = zmin, zmax

    @property
    def cx(self): return (self.xmin + self.xmax) / 2
    @property
    def cy(self): return (self.ymin + self.ymax) / 2
    @property
    def w(self): return self.xmax - self.xmin
    @property
    def h(self): return self.ymax - self.ymin
    @property
    def full(self): return self.zmax >= 2.4 and self.zmin <= 0.1

    def dist(self, px, py):
        return dist_point_rect(px, py, self.xmin, self.xmax, self.ymin, self.ymax)

    def __repr__(self):
        hi = "全高" if self.full else f"z[{self.zmin:.2f},{self.zmax:.2f}]"
        return (f"{self.name}: x[{self.xmin:.3f},{self.xmax:.3f}] "
                f"y[{self.ymin:.3f},{self.ymax:.3f}] ({hi})")


class Box:
    def __init__(self, name, aabb):
        self.name = name
        self.xmin, self.xmax, self.ymin, self.ymax = aabb
        self.zmin, self.zmax = 0.0, BOX_ZMAX

    def dist(self, px, py):
        return dist_point_rect(px, py, self.xmin, self.xmax, self.ymin, self.ymax)

    def __repr__(self):
        return (f"{self.name}: x[{self.xmin:.3f},{self.xmax:.3f}] "
                f"y[{self.ymin:.3f},{self.ymax:.3f}] z[0,{self.zmax:.2f}]")


class Tree:
    def __init__(self, name, cx, cy):
        self.name = name
        self.cx, self.cy = cx, cy
        self.r = TREE_RADIUS
        self.zmin, self.zmax = 0.0, TREE_HEIGHT

    def dist(self, px, py):
        return dist_point_circle(px, py, self.cx, self.cy, self.r)

    def __repr__(self):
        return f"{self.name}: ({self.cx:.3f}, {self.cy:.3f}), r={self.r:.2f}, z<= {self.zmax:.2f}"


# ----------------------------------------------------------------------------
# 解析
# ----------------------------------------------------------------------------
def model_pose(model):
    p = model.find("pose")
    return parse_pose(p.text) if p is not None else (0.0, 0.0, 0.0, 0.0)


def walk_models(parent, px, py, pz, pyaw, out):
    for model in parent.findall("model"):
        mx, my, mz, myaw = model_pose(model)
        ox, oy = rot_xy(mx, my, pyaw)
        out.append((model, px + ox, py + oy, pz + mz, pyaw + myaw))
        walk_models(model, px + ox, py + oy, pz + mz, pyaw + myaw, out)


def extract_geometry(world):
    walls, boxes, trees, targets = [], [], [], []
    landing = None
    all_models = []
    walk_models(world, 0.0, 0.0, 0.0, 0.0, all_models)

    for model, ax, ay, az, ayaw in all_models:
        name = model.get("name", "")

        if name == "toudi2":
            for link in model.findall("link"):
                lname = link.get("name", "")
                lp = link.find("pose")
                lx, ly, lz, lyaw = parse_pose(lp.text) if lp is not None else (0, 0, 0, 0)
                lx += ax; ly += ay; lz += az; lyaw += ayaw
                for col in link.findall("collision"):
                    box = col.find("geometry/box")
                    if box is None:
                        continue
                    size = [float(t) for t in box.find("size").text.split()]
                    v = [0.0, 0.0, 0.0]
                    cp = col.find("pose")
                    if cp is not None:
                        v = [float(t) for t in cp.text.split()]
                    ox, oy = rot_xy(v[0], v[1], lyaw)
                    zc = lz + v[2]
                    walls.append(Wall(f"{name}/{lname}",
                                      box_aabb(lx + ox, ly + oy, size[0], size[1], lyaw),
                                      zc - size[2] / 2, zc + size[2] / 2))
            continue

        if "big_box" in name.lower():
            size = [1.2, 0.8, 1.25]
            link = model.find("link")
            if link is not None:
                box = link.find("collision/geometry/box")
                if box is not None:
                    size = [float(t) for t in box.find("size").text.split()]
            boxes.append(Box(name, box_aabb(ax, ay, size[0], size[1], ayaw)))
            continue

        if "juniper" in name.lower():
            trees.append(Tree(name, ax, ay))
            continue

        if name in ("dibao", "qiaoliang", "tanke", "zhangpeng", "zhuangjiache"):
            targets.append((name, ax, ay))
            continue

        if name == "landing_h":
            landing = (ax, ay)
            continue

    return walls, boxes, trees, targets, landing


# ----------------------------------------------------------------------------
# 航点验证
# ----------------------------------------------------------------------------
def obstacle_blocks(ob, z):
    """障碍在高度 z（航点）处是否与机身垂直范围相交。"""
    return not (ob.zmax < z - UAV_HZ or ob.zmin > z + UAV_HZ)


def check_waypoint(x, y, z, walls, boxes, trees):
    """返回 (最近障碍名, 平面距离)。高度上可飞越的障碍（如箱顶/横梁之下）不计。"""
    best_d, best_n = 1e9, None
    for w in walls:
        if not obstacle_blocks(w, z):
            continue
        d = w.dist(x, y)
        if d < best_d:
            best_d, best_n = d, w.name
    for b in boxes:
        if not obstacle_blocks(b, z):
            continue
        d = b.dist(x, y)
        if d < best_d:
            best_d, best_n = d, b.name
    for t in trees:
        if not obstacle_blocks(t, z):
            continue
        d = t.dist(x, y)
        if d < best_d:
            best_d, best_n = d, t.name
    return best_n, best_d


def grade(d):
    if d >= D_STRICT:
        return "OK "
    if d >= D_SAFE:
        return "~   "          # 安全档
    if d >= D_MIN:
        return "-   "          # 最小档（仅能通过）
    return "XX  "


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def main():
    world = ET.parse(str(WORLD_PATH)).getroot().find("world")
    walls, boxes, trees, targets, landing = extract_geometry(world)

    print("=" * 84)
    print("toudi3.world 障碍物与无障碍航路点分析")
    print("=" * 84)

    # ---------- 1. 场地边界 ----------
    print("\n【1】场地边界（四面外墙：Wall_1 南 / Wall_4+6 西 / Wall_11 东 / Wall_9 北）")
    ext = {w.name.split("/")[1] for w in walls}
    outer = [w for w in walls if w.name.split("/")[1] in ("Wall_1", "Wall_4", "Wall_6", "Wall_9", "Wall_11")]
    bx = (min(w.xmin for w in outer), max(w.xmax for w in outer))
    by = (min(w.ymin for w in outer), max(w.ymax for w in outer))
    print(f"  场地范围: x ∈ [{bx[0]:.3f}, {bx[1]:.3f}]   y ∈ [{by[0]:.3f}, {by[1]:.3f}]")
    print(f"  场地尺寸: 宽 {bx[1]-bx[0]:.3f} m × 深 {by[1]-by[0]:.3f} m")
    print(f"  内部净空: x ∈ [{bx[0]+0.15:.3f}, {bx[1]-0.15:.3f}]   y ∈ [{by[0]+0.15:.3f}, {by[1]-0.15:.3f}]")

    # ---------- 2. 墙段 ----------
    print(f"\n【2】墙段清单（绝对坐标，共 {len(walls)} 段）")
    order = ["Wall_1", "Wall_11", "Wall_13", "Wall_14", "Wall_15", "Wall_16",
             "Wall_20", "Wall_22", "Wall_4", "Wall_6", "Wall_9"]
    grouped = {}
    for w in walls:
        grouped.setdefault(w.name.split("/")[1], []).append(w)
    for n in order:
        for w in sorted(grouped.get(n, []), key=lambda q: (q.cx, q.cy)):
            kind = "横墙" if w.h <= 0.2 else "纵墙"
            print(f"  {w}  {kind}")

    # ---------- 3. 半高障碍 ----------
    print(f"\n【3】半高障碍")
    print("  箱子（z ∈ [0, 1.25]，飞行高度 >1.3 m 可飞越）:")
    for b in boxes:
        print(f"  {b}")
    print("  松树（r=0.4 m 保守值, z ∈ [0, 1.5]，飞行高度 >1.6 m 可飞越）:")
    for t in trees:
        print(f"  {t}")

    # ---------- 4. 标准靶 ----------
    print(f"\n【4】标准靶（投递目标，贴地无障碍）")
    for n, tx, ty in targets:
        print(f"  {n}: ({tx:.3f}, {ty:.3f})")
    if landing:
        print(f"  起降标志 landing_h（= 任务 Takeoff/Land 点）: ({landing[0]:.3f}, {landing[1]:.3f})")

    # ---------- 5. 门洞 / 通道 / 出口 ----------
    print(f"\n【5】门洞 / 通道 / 出口分析（无人机宽 0.5 m）")
    W = {}
    for k, v in grouped.items():
        W[k] = v

    print("\n  (a) 外墙开口检查:")
    for k, label in [("Wall_1", "南墙"), ("Wall_11", "东墙"), ("Wall_9", "北墙")]:
        segs = W[k]
        cont = len(segs) == 1 and segs[0].full
        print(f"    {label} {k}: {len(segs)} 段, {'连续全高、无缺口' if cont else '需检查'}")
    w4, w6 = W["Wall_4"][0], W["Wall_6"][0]
    print(f"    西墙: Wall_4(南段 y<={w4.ymax:.3f}) + Wall_6(北段 y>={w6.ymin:.3f}) "
          f"重叠 {w4.ymax-w6.ymin:.3f} m -> 连续全高、无缺口")
    print("    四角封闭性: 西南/东南/西北/东北 均为墙角重叠封闭 -> 场地无门，"
          "北墙外（y>6.945）从内部不可达")

    print("\n  (b) 内部隔断（Wall_13 线 / Wall_15 线 / Wall_20 / Wall_22）:")
    w13 = W["Wall_13"][0]
    w14 = W["Wall_14"][0]
    print(f"    Wall_13 横向隔墙: x∈[{w13.xmin:.3f},{w13.xmax:.3f}] y∈[{w13.ymin:.3f},{w13.ymax:.3f}] 全高")
    gap_west = w13.xmin - w14.xmin
    gap_east = W["Wall_11"][0].xmin - w13.xmax
    print(f"      西端与 Wall_14 衔接间隙: {gap_west:.3f} m {'(相连封闭)' if gap_west < 0.15 else '(可通行)'}")
    print(f"      东端与东墙间隙: {gap_east:.3f} m {'(几乎封闭)' if gap_east < 0.15 else '(可通行)'}")

    print("    Wall_15 组合墙（西段带半高段，y∈[4.225,4.375]）:")
    for w in sorted(W["Wall_15"], key=lambda q: q.xmin):
        print(f"      {w}")
    print("      对飞行高度 0.65~1.7 m 的净效果:")
    print("        x∈[-3.744,-3.280]: 中高段 z∈[0.5,2.5] -> 全挡")
    print("        x∈[-3.280,-2.480]: 上段 z∈[1.3,2.5] -> 高度 0.65~1.3 m 可通行（净高 0.65 m）")
    print("        x∈[-2.480,-1.994]: 全高 -> 全挡")

    for k in ("Wall_20", "Wall_22"):
        fulls = [w for w in W[k] if w.full]
        fulls.sort(key=lambda q: q.cy)
        beam = [w for w in W[k] if not w.full]
        if len(fulls) == 2:
            gap = fulls[1].ymin - fulls[0].ymax
            print(f"    {k}: 南柱 y∈[{fulls[0].ymin:.3f},{fulls[0].ymax:.3f}]、北柱 "
                  f"y∈[{fulls[1].ymin:.3f},{fulls[1].ymax:.3f}]、横梁 z∈[2.0,2.5]")
            print(f"      门洞 y∈[{fulls[0].ymax:.3f},{fulls[1].ymin:.3f}] 宽 {gap:.3f} m，"
                  f"x∈[{fulls[0].xmin:.3f},{fulls[0].xmax:.3f}]（深 {fulls[0].w:.3f} m），净高 2.0 m")
            verdict = "可通过" if gap >= D_STRICT and fulls[0].w >= D_STRICT else \
                      ("勉强" if gap >= D_MIN and fulls[0].w >= D_MIN else "无法通过")
            print(f"      无人机判定: 门宽 {gap:.3f} m、深 {fulls[0].w:.3f} m -> {verdict}"
                  "（深 0.15 m < 机宽 0.5 m，实际仅供行人）" if verdict == "无法通过" else
                  f"      无人机判定: 门宽 {gap:.3f} m、深 {fulls[0].w:.3f} m -> {verdict}")

    print("\n  (c) 北部区域（y∈[5.725,6.795]）可达性:")
    w16 = W["Wall_16"][0]
    west_inner = w6.xmax          # 西墙内壁 x=-4.335
    east_face = w16.xmin          # Wall_16 西边缘 x=-3.744（墙本体以东不可通行）
    bw = east_face - west_inner
    center_x = (west_inner + east_face) / 2
    print(f"    北墙 Wall_9 无开口；Wall_20/22 门洞无人机无法通过")
    print(f"    唯一入口: 西部条带（西墙内壁 x={west_inner:.3f} 与 Wall_16 西边缘 x={east_face:.3f} 之间）")
    print(f"      通道宽 {bw:.3f} m，中心线 x={center_x:.3f}，中心线距两侧障碍 {bw/2:.3f} m "
          f"-> {'(>=0.55 可安全通行)' if bw/2 >= D_SAFE else '(飞机 0.5 m 可过但余量不足，须低速谨慎)'}")
    for b in boxes:
        if b.xmin < west_inner and b.xmax > east_face:
            print(f"      通道内被 {b.name} 完全堵死 (y∈[{b.ymin:.3f},{b.ymax:.3f}])，"
                  f"需飞越(高度>1.3 m)或向东绕行")

    # ---------- 6. 航路点 ----------
    print(f"\n【6】推荐无障碍航路点")
    print(f"  判据: 中心到障碍 >= {D_STRICT:.2f} m(严格 0.5 余量) / "
          f">= {D_SAFE:.2f} m(安全 0.3 余量) / >= {D_MIN:.2f} m(最小) / XX 冲突")
    waypoints = [
        # (名称, x, y, z, 固定? 说明)
        ("起飞点 Takeoff(任务固定)", 0.000, 0.000, 1.5, True, "landing_h 位置，距松树0 中心 0.75 m"),
        ("检测点1 dibao", -0.602, -1.041, 1.5, True, "任务固定"),
        ("检测点2 qiaoliang", -1.903, -0.023, 1.5, True, "任务固定"),
        ("检测点3 zhuangjiache", 1.016, 0.256, 1.5, True, "任务固定"),
        ("进入点 W1 西部条带南口", -4.04, -0.5, 1.5, False, "通道中心线 x=-4.04, 箱子3_0 以南"),
        ("通道点 W2 两箱之间", -4.04, 1.5, 1.5, False, "通道中心线"),
        ("绕箱点 W3 (big_box3 东侧)", -2.3, 2.5, 1.5, False, "绕开堵死通道的箱子"),
        ("通道点 W4 回西部条带", -4.04, 3.5, 1.5, False, "通道中心线"),
        ("出口点 W5 北区入口", -4.04, 6.0, 1.5, False, "转入北部区域"),
        ("北区航点 W6", -1.5, 6.26, 1.5, False, "北区垂直中线（距两侧各 0.54 m，低速）"),
        ("北区航点 W7 (近原试飞点)", 0.3, 6.26, 1.5, False, "原 (0.3,6.5) 靠北墙过近，居中留余量"),
        ("返航点(任务固定)", 0.000, 0.000, 1.0, True, "与起飞点同位置"),
        ("降落点(任务固定)", 0.000, 0.000, 0.5, True, "与起飞点同位置"),
    ]
    print(f"  {'航点':<28}{'x':>8}{'y':>8}{'z':>6}  最近障碍            距离   判定")
    results = {}
    for name, x, y, z, fixed, note in waypoints:
        n, d = check_waypoint(x, y, z, walls, boxes, trees)
        results[(x, y, z)] = (n, d)
        print(f"  {name:<28}{x:>8.3f}{y:>8.3f}{z:>6.2f}  {str(n):<20}{d:>6.2f}   {grade(d)}  {note}")

    # ---------- 7. 俯视图 ----------
    print(f"\n【7】ASCII 俯视图（x 向右 →, y 向上 ↑；网格 0.5 m）")
    print("  图例: # 墙 / B 箱子 / T 树 / t 靶 / H 起降 / o 推荐航点")
    draw_map(walls, boxes, trees, targets, landing, [(w[1], w[2]) for w in waypoints if not w[4]])

    # ---------- 8. 结论 ----------
    print(f"\n【8】结论要点")
    print(" 1. 场地是四面墙封闭围场（x∈[-4.485,3.515], y∈[-2.905,6.945]，墙高 2.5 m）：")
    print("    北墙外（y>6.945）从内部不可达；(0.5,7.5)/(0.3,7.2) 被北墙 Wall_9 阻挡属预期。")
    print(" 2. 北墙 Wall_9 连续 8 m 无开口；Wall_20/22 门洞宽 0.9 m 但深仅 0.15 m，")
    print("    0.5 m 宽无人机无法穿过（仅供行人，无人机只能从门洞两侧绕柱）。")
    print(" 3. 走廊出口（北区 y≈6~6.8）唯一通道 = 西部条带 x≈-4.04（宽 0.59 m），")
    print("    通道内两个箱子（y∈[-0.17,0.63] 与 y∈[2.05,2.85]）需飞越(>1.3 m)或向东绕行。")
    print(" 4. 北部区域受北墙(6.795)与 Wall_13(5.725)夹制，可用高度带仅 1.07 m：")
    print("    建议航点 y≈6.26（中线，距两侧各 0.54 m），(0.3,6.5) 距北墙仅 0.30 m 太贴墙。")
    print(" 5. 主厅内任务点（起飞/3 检测/返航/降落）中，起飞点距松树较近（0.75 m 中心距），")
    print("    起飞/降落建议朝向避开松树方向或先平移后升降。")


def draw_map(walls, boxes, trees, targets, landing, wps):
    xmin, xmax = -4.8, 3.8
    ymin, ymax = -3.2, 7.2
    step = 0.5
    nx = int((xmax - xmin) / step) + 1
    ny = int((ymax - ymin) / step) + 1
    grid = [["."] * nx for _ in range(ny)]
    for w in walls:
        if not w.full:
            ch = "-"
        else:
            ch = "#"
        for i in range(ny):
            for j in range(nx):
                x = xmin + j * step
                y = ymin + i * step
                if w.xmin - 0.05 <= x <= w.xmax + 0.05 and w.ymin - 0.05 <= y <= w.ymax + 0.05:
                    if w.full:
                        grid[i][j] = "#"
                    elif grid[i][j] == ".":
                        grid[i][j] = "-"
    for b in boxes:
        for i in range(ny):
            for j in range(nx):
                x = xmin + j * step
                y = ymin + i * step
                if b.xmin <= x <= b.xmax and b.ymin <= y <= b.ymax:
                    grid[i][j] = "B"
    for t in trees:
        for i in range(ny):
            for j in range(nx):
                x = xmin + j * step
                y = ymin + i * step
                if math.hypot(x - t.cx, y - t.cy) <= t.r + 0.05:
                    grid[i][j] = "T"
    for n, tx, ty in targets:
        j = round((tx - xmin) / step); i = round((ty - ymin) / step)
        if 0 <= i < ny and 0 <= j < nx:
            grid[i][j] = "t"
    if landing:
        j = round((landing[0] - xmin) / step); i = round((landing[1] - ymin) / step)
        if 0 <= i < ny and 0 <= j < nx:
            grid[i][j] = "H"
    for x, y in wps:
        j = round((x - xmin) / step); i = round((y - ymin) / step)
        if 0 <= i < ny and 0 <= j < nx and grid[i][j] == ".":
            grid[i][j] = "o"
    print("   " + "".join(f"{int(xmin + j*step):>3}" if j % 2 == 0 else "   " for j in range(nx)))
    for i in range(ny - 1, -1, -1):
        print(f" {ymin + i*step:>4.1f} " + "".join(grid[i]))


if __name__ == "__main__":
    main()
