#!/usr/bin/env python3
"""Inspect toudi3.world model/link poses for corridor geometry."""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world"

with open(path) as f:
    content = f.read()

# Top-level models
models = re.findall(r"<model name='([^']+)'>\s*<pose>([^<]+)</pose>", content)
print("=== Top-level models ===")
for name, pose in models:
    print(f"{name}: {pose.strip()}")

# Find toudi2 nested links
for m in re.finditer(r"<model name='toudi2'>(.*?)</model>", content, re.S):
    block = m.group(1)
    print("\n=== toudi2 nested links ===")
    links = re.findall(r"<link name='([^']+)'>.*?<pose>([^<]+)</pose>", block, re.S)
    for name, pose in links:
        print(f"{name}: {pose.strip()}")
