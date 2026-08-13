#!/usr/bin/env python3
"""六分类 merged_standard 模型 → RKNN fp16 转换（匹配板端 viewer 输入约定）。

流程: best.pt → ONNX → RKNN fp16
输入约定: letterbox 640 + RGB + float32/255（0-1 输入，NPU 不再归一化）
"""
import sys
from pathlib import Path

BASE = Path("/home/xhj/liftrace")
PT = BASE / "vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt"
OUT_DIR = Path("/home/xhj/liftrace/vision_ws/test_data/board_eval_20260716/models")
OUT_RKNN = OUT_DIR / "merged_standard_fp16.rknn"
CALIB_DIR = BASE / "vision_ws/test_data/yolo_dataset_v5_6cls_redcross_standard_20260713/images/val"

IMGSZ = 640


def export_onnx():
    from ultralytics import YOLO
    onnx_path = PT.with_suffix(".fp16.onnx")
    if not onnx_path.exists():
        model = YOLO(str(PT))
        model.export(format="onnx", imgsz=IMGSZ, opset=12)
    return onnx_path


def make_dataset_txt():
    """用 val 集前 8 张图做 fp16 校准。"""
    imgs = sorted(CALIB_DIR.glob("*.jpg"))[:8]
    if not imgs:
        imgs = sorted(CALIB_DIR.glob("*.png"))[:8]
    if not imgs:
        raise RuntimeError(f"no calib images in {CALIB_DIR}")
    txt = OUT_DIR / "calib_fp16.txt"
    txt.write_text("\n".join(str(p) for p in imgs) + "\n")
    return txt


def convert():
    from rknn.api import RKNN
    onnx = export_onnx()
    calib = make_dataset_txt()
    print(f"onnx: {onnx}\ncalib: {calib}\nout: {OUT_RKNN}")

    rknn = RKNN(verbose=False)
    try:
        # 输入 0-1 浮点：mean=0 std=1；fp16 混合量化
        rknn.config(
            mean_values=[[0, 0, 0]],
            std_values=[[1, 1, 1]],
            quantized_dtype="fp16",
            target_platform="rk3588",
        )
        ret = rknn.load_onnx(model=str(onnx), outputs=None)
        if ret != 0:
            raise RuntimeError(f"load_onnx ret={ret}")
        ret = rknn.build(do_quantization=True, dataset=str(calib))
        if ret != 0:
            raise RuntimeError(f"build ret={ret}")
        ret = rknn.export_rknn(str(OUT_RKNN))
        if ret != 0:
            raise RuntimeError(f"export ret={ret}")
        print(f"CONVERTED {OUT_RKNN}")
    finally:
        rknn.release()


if __name__ == "__main__":
    convert()
