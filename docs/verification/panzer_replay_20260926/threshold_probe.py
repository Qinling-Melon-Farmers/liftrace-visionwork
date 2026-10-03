"""Bag source-image threshold/appearance probe; offline files only, no ROS node.

extract: ROS Python (rosbag bindings); infer: rl_drone Python (PT GPU).
Appearance probes are diagnosis, never applied to the flight camera stream.
"""
import argparse
import json
from pathlib import Path


def extract(args):
    import rosbag
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    frames_dir = out / "frames"
    frames_dir.mkdir(exist_ok=True)
    stamps = {}
    with rosbag.Bag(args.bag) as bag:
        begin = bag.get_start_time()
        for _, msg, received in bag.read_messages(topics=[args.detection_topic]):
            if msg.source == "target_detector":
                stamp = msg.header.stamp.to_nsec()
                if stamp in stamps:
                    raise ValueError("duplicate source image stamp")
                stamps[stamp] = {"stamp_ns": stamp,
                    "t": msg.header.stamp.to_sec() - begin,
                    "received": received.to_sec() - begin}
        found = set()
        for _, msg, _ in bag.read_messages(topics=[args.image_topic]):
            stamp = msg.header.stamp.to_nsec()
            if stamp not in stamps:
                continue
            if stamp in found:
                raise ValueError("duplicate camera stamp")
            found.add(stamp)
            path = frames_dir / (str(stamp) + ".jpg")
            path.write_bytes(bytes(msg.data))
            stamps[stamp]["image"] = str(path)
    if set(stamps) != found:
        raise ValueError("not every YOLO source image exists exactly in bag")
    data = {"bag": args.bag, "begin": begin,
            "frames": sorted(stamps.values(), key=lambda f: f["stamp_ns"])}
    (out / "frames.json").write_text(json.dumps(data, indent=2))
    print("Exact source images extracted:", len(found), flush=True)


def appearance(image, mode):
    import cv2
    import numpy as np
    if mode == "gamma065":
        lut = np.rint((np.arange(256) / 255.0) ** .65 * 255).astype(np.uint8)
        return cv2.LUT(image, lut)
    if mode == "clahe":
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        lab[:, :, 0] = cv2.createCLAHE(clipLimit=2., tileGridSize=(8,8)).apply(lab[:, :, 0])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    if mode.startswith("rot"):
        return np.ascontiguousarray(np.rot90(image, int(mode[3:]) // 90))
    return image


def infer(args):
    import cv2
    import numpy as np
    import ultralytics
    from ultralytics import YOLO
    out = Path(args.output)
    data = json.loads((out / "frames.json").read_text())
    model = YOLO(args.model)
    rows = []
    for index, frame in enumerate(data["frames"]):
        image = cv2.imread(frame["image"])
        if image is None:
            raise ValueError("unreadable image: " + frame["image"])
        t = frame["t"]
        probe = (13.5 <= t <= 17.5) or (50.5 <= t <= 53.5)
        modes = ["raw", "gamma065", "clahe", "rot90", "rot180", "rot270"] if probe else ["raw"]
        for mode in modes:
            transformed = appearance(image, mode)
            result = model.predict(transformed, conf=args.conf, imgsz=args.imgsz,
                                   device=args.device, verbose=False)[0]
            dets = [{"class_id": int(b.cls.item()),
                     "class": result.names[int(b.cls.item())],
                     "confidence": float(b.conf.item()),
                     "xyxy": b.xyxy[0].cpu().numpy().tolist()}
                    for b in result.boxes]
            rows.append({"t": t, "stamp_ns": frame["stamp_ns"], "mode": mode,
                         "image": frame["image"], "shape": list(transformed.shape),
                         "detections": dets})
        if index % 100 == 0:
            print("inferred", index, "/", len(data["frames"]), flush=True)
    result = {"bag": data["bag"], "model": args.model, "imgsz": args.imgsz,
              "device": args.device, "conf_floor": args.conf,
              "ultralytics": ultralytics.__version__, "rows": rows}
    (out / "probe.json").write_text(json.dumps(result, indent=2))
    summary = {}
    for segment, bounds in {"outbound": (13.5,17.5), "return": (50.5,53.5)}.items():
        summary[segment] = {}
        for mode in ["raw","gamma065","clahe","rot90","rot180","rot270"]:
            sampled = [r for r in rows if r["mode"] == mode and bounds[0] <= r["t"] <= bounds[1]]
            best = [max((d["confidence"] for d in r["detections"] if d["class"] == "panzer"), default=0) for r in sampled]
            summary[segment][mode] = {"samples":len(best),
                "hits_by_threshold":{str(c):sum(x>=c for x in best) for c in (.05,.1,.2,.3,.4,.5,.6,.7)},
                "peak":max(best, default=0)}
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["extract","infer"])
    p.add_argument("--output", required=True)
    p.add_argument("--bag")
    p.add_argument("--model")
    p.add_argument("--detection-topic",default="/uav_vision/detections")
    p.add_argument("--image-topic",default="/camera/image_raw/compressed")
    p.add_argument("--conf",type=float,default=.05)
    p.add_argument("--imgsz",type=int,default=640)
    p.add_argument("--device",default="0")
    args=p.parse_args()
    if args.mode=="extract": extract(args)
    else: infer(args)