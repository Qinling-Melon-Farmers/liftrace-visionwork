#!/usr/bin/env python3
import torch

for label, path in [
    ("vision_ws/yolo11n.pt", "/home/xhj/liftrace/vision_ws/yolo11n.pt"),
    ("liftrace/yolo11n.pt (root)", "/home/xhj/liftrace/yolo11n.pt"),
    ("vision_ws/yolo26n.pt", "/home/xhj/liftrace/vision_ws/yolo26n.pt"),
]:
    d = torch.load(path, map_location="cpu", weights_only=False)
    print(f"=== {label} ===")
    print(f"  epoch: {d['epoch']}")
    print(f"  date: {d['date']}")
    print(f"  model type: {type(d.get('model', '?')).__name__}")
    ta = d.get('train_args', {})
    if ta:
        print(f"  train_args.data: {ta.get('data', '?')}")
        print(f"  train_args.model: {ta.get('model', '?')}")
        print(f"  train_args.name: {ta.get('name', '?')}")
        print(f"  train_args.epochs: {ta.get('epochs', '?')}")
        print(f"  train_args.imgsz: {ta.get('imgsz', '?')}")
    tm = d.get('train_metrics', {})
    if tm:
        mAP50 = tm.get('metrics/mAP50(B)', '?')
        mAP50_95 = tm.get('metrics/mAP50-95(B)', '?')
        print(f"  best mAP50: {mAP50}")
        print(f"  best mAP50-95: {mAP50_95}")
    print()
