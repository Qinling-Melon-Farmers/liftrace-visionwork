#!/usr/bin/env python3
"""契约测试：旧 standard+tank 输出合并与三帧语义投票。"""
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
path = ROOT / "top_level_scripts" / "board_legacy_rknn_dual_viewer.py"
spec = importlib.util.spec_from_file_location("legacy_dual", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


standard = [(1, 2, 11, 12, 0, 0.8), (3, 4, 13, 14, 3, 0.7)]
tank = [(5, 6, 15, 16, 0, 0.6)]
merged = module.combine_detections(standard, tank)
assert [row[4] for row in merged] == [0, 3, 4]
assert module.standard_vote([[0], [0], [3]]) == "bridge"
assert module.standard_vote([[], [], []]) == "Nothing"
assert module.standard_vote([[0], [0]]) == ""
print("PASS legacy dual mapping and vote contract")
