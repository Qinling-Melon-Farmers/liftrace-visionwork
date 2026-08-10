#!/bin/bash
cd /home/orangepi/board_eval
python3 - <<'PY'
import os, numpy as np, cv2
from rknnlite.api import RKNNLite

MODEL = "/home/orangepi/Visual/src/yolov5_detect/best_rknn_model/best-rk3588.rknn"
IMG = os.path.expanduser("~/Visual/src/yolov5_detect/scripts/panzer.jpg")

rt = RKNNLite()
print("load_rknn:", rt.load_rknn(MODEL))
print("init_runtime:", rt.init_runtime())

img = cv2.imread(IMG)
print("img", img.shape)
h, w = img.shape[:2]
r = min(640 / w, 640 / h)
nw, nh = round(w * r), round(h * r)
resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
top, left = (640 - nh) // 2, (640 - nw) // 2
canvas[top:top + nh, left:left + nw] = resized

variants = {
    "rgb_f32_0_1": (cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0)[None, ...],
    "bgr_f32_0_1": (canvas.astype(np.float32) / 255.0)[None, ...],
    "rgb_u8": cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)[None, ...],
    "bgr_u8": canvas[None, ...],
    "rgb_f32_0_255": cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32)[None, ...],
}
for name, tensor in variants.items():
    outs = rt.inference(inputs=[tensor])
    out = outs[0]
    arr = np.asarray(out)
    print("=== variant", name, "shape", arr.shape, "dtype", arr.dtype,
          "min %.3f max %.3f" % (arr.min(), arr.max()) if arr.size else "")
    if arr.ndim == 3 and arr.shape[0] == 1:
        a = arr[0]
        print("   squeeze shape", a.shape)
        if a.shape[0] < a.shape[1]:  # (84, 8400)
            aT = a.T
            print("   transposed (8400,84): top5 conf:", np.sort(aT[:, 4:].max(axis=1))[-5:])
        else:
            print("   raw (8400,84): top5 conf:", np.sort(a[:, 4:].max(axis=1))[-5:])
rt.release()
PY
