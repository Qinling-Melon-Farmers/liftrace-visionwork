"""Five-class export/package preparation. Local CPU/simulator only; no hardware access."""
import argparse
import ast
import importlib.metadata
import json
import os
import re
import shutil
import sys
import tarfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path

os.environ["YOLO_AUTOINSTALL"] = "false"
NAMES = ["bridge", "panzer", "pillbox", "tent", "red_cross"]
ROOT = Path(__file__).resolve().parents[2]
CONTRACT = {
    "task": "detect", "imgsz": [640, 640], "channels": 3,
    "input_layout": "NHWC", "input_color_space": "RGB",
    "input_dtype": "float32", "input_normalize": True,
    "box_format": "xywh", "output_channels": 9,
    "output_contract": "yolo11_decoded_no_objectness",
    "names": dict(enumerate(NAMES)),
}
FUNCTIONS = [
    "_letterbox_bgr", "_to_model_input", "_xywh_to_xyxy", "_scale_boxes",
    "_iou_xyxy", "_nms_classwise", "_candidate_arrays", "_decode_outputs",
    "_validate_output_contract",
]


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def check(condition, message):
    if not condition:
        raise ValueError(message)


def versions():
    result = {}
    for name in ["ultralytics", "torch", "onnx", "onnxruntime", "rknn-toolkit2"]:
        try:
            result[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result[name] = None
    import torch
    result["torch_loaded"] = torch.__version__
    return result


def board_contract(metadata, detector_config, decoder):
    import yaml
    meta = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    for key, expected in CONTRACT.items():
        check(meta.get(key) == expected, "Board metadata mismatch: " + key)
    cfg = yaml.safe_load(detector_config.read_text(encoding="utf-8"))
    for key, expected in [
        ("imgsz", 640), ("input_layout", "NHWC"), ("input_color_space", "RGB"),
        ("input_dtype", "float32"), ("input_normalize", True),
    ]:
        check(cfg.get(key) == expected, "Board runtime config mismatch: " + key)
    import cv2
    import numpy as np
    tree = ast.parse(decoder.read_text(encoding="utf-8"))
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in FUNCTIONS]
    check(len(nodes) == len(FUNCTIONS), "Board decoder functions unavailable")
    ns = {"np": np, "cv2": cv2}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(decoder), "exec"), ns)
    # Execute production preprocessing on a non-square, color-distinct image.
    image = np.empty((301, 503, 3), dtype=np.uint8)
    image[:] = [17, 89, 203]
    x, scale, pad, _ = ns["_to_model_input"](image, 640)
    check(x.shape == (1, 640, 640, 3) and x.dtype == np.float32,
          "Actual preprocessing shape/dtype mismatch")
    np.testing.assert_allclose(x[0, 320, 320], np.array([203, 89, 17]) / 255, atol=1e-7)
    np.testing.assert_allclose(x[0, 0, 0], np.array([114, 114, 114]) / 255, atol=1e-7)
    return ns, {
        "status": "PASS", "metadata_source": str(metadata.resolve()),
        "runtime_config": str(detector_config.resolve()), "decoder": str(decoder.resolve()),
        "letterbox": {"size": [640, 640], "padding_bgr": [114, 114, 114],
                      "interpolation": "INTER_LINEAR", "auto_stride": False},
        "input": "RGB float32 normalized 0..1 NHWC",
        "onnx_input": [1, 3, 640, 640], "onnx_output": [1, 9, 8400],
        "output": "xywh pixels in letterbox image + 5 class probabilities; no objectness/NMS",
        "normalization_owner": "board Python divides by 255; RKNN mean=0 std=1",
        "hardware_validated": False,
    }


def load_model(weights):
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(4)
    check(weights.is_file() and weights.stat().st_size > 0, "Missing PT: " + str(weights))
    model = YOLO(str(weights), task="detect")
    actual = [model.names[i] for i in range(len(model.names))]
    check(actual == NAMES, "Class order mismatch: " + repr(actual))
    check(model.task == "detect", "Only detection models are supported")
    head = model.model.model[-1]
    check(type(head).__name__ == "Detect" and int(head.nc) == 5,
          "Unsupported detection head: " + type(head).__name__)
    check(not getattr(head, "end2end", False), "End-to-end/NMS head incompatible with board decoder")
    model.model.float().eval()
    with torch.inference_mode():
        raw = model.model(torch.zeros(1, 3, 640, 640))
    prediction = raw[0] if isinstance(raw, (tuple, list)) else raw
    check(list(prediction.shape) == [1, 9, 8400], "Unexpected PT output: " + str(prediction.shape))
    architecture = {
        "task": model.task, "model_type": type(model.model).__name__,
        "head": type(head).__name__, "nc": int(head.nc),
        "reg_max": int(head.reg_max), "stride": head.stride.tolist(),
        "output_shape": list(prediction.shape),
        "parameter_count": sum(p.numel() for p in model.model.parameters()),
        "modules": [type(m).__name__ for m in model.model.model],
        "yaml": model.model.yaml,
    }
    return model, architecture


def read_handoff(path, weights):
    data = json.loads(path.read_text(encoding="utf-8"))
    check(data.get("final") is True, "Training agent has not marked these weights final")
    check(data.get("agent_id") == "01a129b2-938c-78b0-973b-170523fa50d1",
          "Unexpected training agent identity")
    check(Path(data["best_pt"]).resolve() == weights.resolve(), "Handoff PT path differs")
    check(data.get("names") == NAMES, "Handoff class order differs")
    return data


def toolkit_onnx_compatibility():
    """Restore only the public dtype mapping required by Toolkit2 in this process."""
    import types
    import onnx
    applied = False
    if not hasattr(onnx, "mapping"):
        from onnx import _mapping
        lookup = {key: value.np_dtype for key, value in _mapping.TENSOR_TYPE_MAP.items()}
        check(lookup[onnx.TensorProto.FLOAT].name == "float32",
              "ONNX float dtype mapping cannot be restored")
        inverse = {}
        for key, dtype in sorted(lookup.items()):
            inverse.setdefault(dtype, key)
        check(inverse[lookup[onnx.TensorProto.FLOAT]] == onnx.TensorProto.FLOAT,
              "ONNX inverse float mapping cannot be restored")
        check(inverse[lookup[onnx.TensorProto.INT64]] == onnx.TensorProto.INT64,
              "ONNX inverse int64 mapping cannot be restored")
        onnx.mapping = types.SimpleNamespace(TENSOR_TYPE_TO_NP_TYPE=lookup,
                                             NP_TYPE_TO_TENSOR_TYPE=inverse)
        applied = True
    return {"onnx_mapping_process_shim": applied,
            "environment_packages_modified": False}


def rknn_config(runtime, precision):
    # FP32 request must actually succeed; never silently substitute FP16/tfloat32.
    dtype = {"fp32": "float32", "fp16": "float16"}[precision]
    rc = runtime.config(target_platform="rk3588", float_dtype=dtype,
                        mean_values=[[0, 0, 0]], std_values=[[1, 1, 1]])
    check(rc == 0, "RKNN config failed: " + str(rc))


def inspect_command(args):
    _, contract = board_contract(args.metadata, args.detector_config, args.decoder)
    _, architecture = load_model(args.weights.resolve())
    print(json.dumps({"names": NAMES, "architecture": architecture,
                      "board_contract": contract, "versions": versions(),
                      "final_model_confirmed": False}, ensure_ascii=False, indent=2))


def validate_onnx(path):
    import numpy as np
    import onnx
    import onnxruntime as ort
    graph = onnx.load(str(path))
    onnx.checker.check_model(graph)
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 4
    opts.inter_op_num_threads = 1
    session = ort.InferenceSession(str(path), sess_options=opts, providers=["CPUExecutionProvider"])
    check(len(session.get_inputs()) == len(session.get_outputs()) == 1, "Expected one input/output")
    check(session.get_inputs()[0].shape == [1, 3, 640, 640], "Unexpected ONNX input shape")
    check(session.get_inputs()[0].type == "tensor(float)", "ONNX input must be FP32")
    check(session.get_outputs()[0].shape == [1, 9, 8400], "Unexpected ONNX output shape")
    check(session.get_outputs()[0].type == "tensor(float)", "ONNX output must be FP32")
    out = session.run(None, {session.get_inputs()[0].name: np.zeros((1, 3, 640, 640), np.float32)})
    check(out[0].shape == (1, 9, 8400) and np.isfinite(out[0]).all(), "Invalid ONNX execution")
    return session


def pt_onnx_validation(model, session, images, ns):
    import cv2
    import numpy as np
    import torch
    model.model.float().eval()
    rows = []
    for path in images:
        image = cv2.imread(str(path))
        check(image is not None, "Unreadable PT/ONNX validation image: " + str(path))
        tensor, _, _, _ = ns["_to_model_input"](image, 640)
        tensor = np.ascontiguousarray(tensor.transpose(0, 3, 1, 2))
        with torch.inference_mode():
            output = model.model(torch.from_numpy(tensor))
        prediction = output[0] if isinstance(output, (tuple, list)) else output
        prediction = prediction.numpy()
        reference = session.run(None, {session.get_inputs()[0].name: tensor})[0]
        np.testing.assert_allclose(prediction, reference, rtol=1e-3, atol=0.03)
        rows.append({"image": str(path.resolve()),
                     "max_abs": float(np.abs(prediction - reference).max())})
    return {"status": "PASS", "rtol": 1e-3, "atol": 0.03, "samples": rows}


def simulator_validation(runtime, session, images, ns, meta):
    import cv2
    import numpy as np
    check(runtime.init_runtime() == 0, "CPU simulator initialization failed")
    rows = []
    for path in images:
        image = cv2.imread(str(path))
        check(image is not None, "Unreadable validation image: " + str(path))
        x, scale, pad, _ = ns["_to_model_input"](image, 640)
        outputs = runtime.inference(inputs=[x], data_format=["nhwc"])
        ns["_validate_output_contract"](outputs, meta)
        check(outputs[0].shape == (1, 9, 8400) and np.isfinite(outputs[0]).all(),
              "Invalid simulator output")
        ref = session.run(None, {session.get_inputs()[0].name: x.transpose(0, 3, 1, 2)})
        decode = lambda out: ns["_decode_outputs"](
            out, 5, 0.5, 640, image.shape, scale, pad, iou_threshold=0.45, box_format="xywh")
        observed, expected = decode(outputs), decode(ref)
        check(len(observed) == len(expected), "Simulator/ONNX detection count differs: " + str(path))
        unmatched = list(expected)
        matched = []
        for det in observed:
            compatible = [i for i, other in enumerate(unmatched)
                          if other["class_id"] == det["class_id"]]
            check(bool(compatible), "Simulator/ONNX class mismatch: " + str(path))
            i = max(compatible, key=lambda k: float(ns["_iou_xyxy"](
                det["bbox"], unmatched[k]["bbox"][None, :])[0]))
            other = unmatched.pop(i)
            iou = float(ns["_iou_xyxy"](det["bbox"], other["bbox"][None, :])[0])
            delta = abs(det["score"] - other["score"])
            check(iou >= 0.98 and delta <= 0.02,
                  "Simulator/ONNX detection mismatch: IoU=%g score_delta=%g" % (iou, delta))
            matched.append({"class_name": NAMES[det["class_id"]], "score": det["score"],
                            "box": det["bbox"].tolist(), "iou_to_onnx": iou,
                            "score_delta": delta})
        rows.append({"image": str(path.resolve()), "output_shape": list(outputs[0].shape),
                     "max_abs": float(np.abs(outputs[0] - ref[0]).max()), "detections": matched})
    return {"status": "PASS", "hardware_tested": False, "backend": "Toolkit2 CPU simulator",
            "conf_threshold": 0.5, "iou_threshold": 0.45,
            "match_min_iou": 0.98, "match_max_score_delta": 0.02, "samples": rows,
            "scope": "I/O and decoder consistency only; not dataset accuracy or NPU acceptance"}


def export_command(args):
    import yaml
    weights = args.weights.resolve()
    check(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.model_id) is not None,
          "model-id must contain only ASCII letters, numbers, underscore or hyphen")
    if args.workflow_smoke:
        check(args.model_id.startswith("smoke_"), "Smoke model-id must start with smoke_")
        handoff = None
    else:
        check(args.handoff is not None, "Final export requires the training agent handoff JSON")
        handoff = read_handoff(args.handoff, weights)
        for name in ["training_recipe", "source_statistics", "test_summary"]:
            value = getattr(args, name)
            if name == "test_summary" and value is None:
                continue  # Evaluation may run in parallel; package still requires it.
            check(value is not None and value.is_file() and value.stat().st_size > 0,
                  "Missing training handoff attachment: " + name)
            check(value.suffix.lower() in [".yaml", ".yml", ".json", ".md", ".txt", ".csv"],
                  "Training attachments must be text manifests/recipes/summaries: " + name)
    ns, contract = board_contract(args.metadata, args.detector_config, args.decoder)
    model, architecture = load_model(weights)
    check(bool(args.image), "At least one explicit validation image is required")
    for image in args.image:
        check(image.is_file(), "Missing validation image: " + str(image))
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "RUNNING", "names": NAMES, "source_weights": str(weights),
        "model_id": args.model_id, "purpose": "workflow_smoke" if args.workflow_smoke else "final_candidate",
        "final_model_confirmed": handoff is not None, "handoff": handoff,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "rknn_status": "not_attempted", "hardware_validated": False,
        "toolkit_version": importlib.metadata.version("rknn-toolkit2"),
        "versions": versions(), "requested_precision": args.precision,
        "architecture": architecture, "board_contract": contract, "files": {},
        "onnx_output": [1, 9, 8400], "input": contract["input"],
    }
    runtime = None
    try:
        from rknn.api import RKNN
        runtime = RKNN(verbose=False)
        report["toolkit_compatibility"] = toolkit_onnx_compatibility()
        try:
            rknn_config(runtime, args.precision)
        except Exception as exc:
            report.update(status="BLOCKED", rknn_status="blocked_precision", error=str(exc))
            raise RuntimeError("Requested " + args.precision +
                               " is unavailable; no fallback model exported. " + str(exc)) from exc
        pt = out / (args.model_id + ".pt")
        before = weights.stat()
        shutil.copyfile(weights, pt)
        after = weights.stat()
        check((before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns),
              "Source PT changed during copy; wait for completed training handoff")
        # Export the private copy: Ultralytics writes ONNX beside the input PT.
        model, copied_architecture = load_model(pt)
        check(copied_architecture == architecture, "Copied PT architecture differs")
        path = Path(model.export(format="onnx", imgsz=640, batch=1, dynamic=False,
                                 opset=12, simplify=False, nms=False, half=False, device="cpu"))
        check(path.resolve().parent == out, "ONNX was written outside the private export directory")
        session = validate_onnx(path)
        pt_onnx = pt_onnx_validation(model, session, args.image, ns)
        report["files"].update(pt=pt.name, onnx=path.name)
        check(runtime.load_onnx(model=str(path)) == 0, "RKNN load_onnx failed")
        check(runtime.build(do_quantization=False) == 0, "RKNN build failed")
        rknn = out / (args.model_id + "_" + args.precision + ".rknn")
        check(runtime.export_rknn(str(rknn)) == 0, "RKNN export failed")
        check(rknn.is_file() and rknn.stat().st_size > 0, "Empty RKNN export")
        report.update(rknn_status="exported", rknn=str(rknn), effective_precision=args.precision)
        meta = dict(CONTRACT, description="Five-class candidate; H excluded from positive classes",
                    version=args.model_id, model_file=rknn.name, candidate=True,
                    hardware_validated=False, rknn_float_dtype={"fp16": "float16", "fp32": "float32"}[args.precision])
        (out / "metadata.yaml").write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")
        report["files"].update(rknn=rknn.name, metadata="metadata.yaml")
        sim = simulator_validation(runtime, session, args.image, ns, meta)
        write_json(out / "rknn_simulator.json", sim)
        write_json(out / "validation.json", {
            "status": "PASS", "pt_structure": "PASS", "onnx_structure_execution": "PASS",
            "preprocessing_contract": "PASS", "simulator_decoder_consistency": "PASS",
            "pt_onnx_numeric": pt_onnx,
            "hardware_validated": False, "accuracy_test": "see separate training test summary",
        })
        for key in ["training_recipe", "source_statistics", "test_summary"]:
            source = getattr(args, key)
            if source:
                destination = key + source.suffix
                check(destination not in report["files"].values(), "Duplicate attachment name")
                shutil.copyfile(source, out / destination)
                report["files"][key] = destination
        report.update(status="PASS", candidate=True,
                      package_ready=all(key in report["files"] for key in
                                        ["training_recipe", "source_statistics", "test_summary"]),
                      pending_attachments=([ "test_summary" ] if "test_summary" not in report["files"] else []))
    except Exception as exc:
        if report["status"] != "BLOCKED":
            report.update(status="FAIL", error=repr(exc))
        raise
    finally:
        if runtime:
            runtime.release()
        write_json(out / "export.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


def package_command(args):
    import yaml
    source = args.export.resolve()
    check(not (source / "export.json").is_symlink(), "Symlink export manifest rejected")
    report = json.loads((source / "export.json").read_text(encoding="utf-8"))
    check(report.get("status") == "PASS" and report.get("rknn_status") == "exported",
          "Only successful current exports can be packaged")
    check(report.get("purpose") == "final_candidate" and report.get("final_model_confirmed") is True,
          "Smoke/historical/unconfirmed models cannot be packaged as the final candidate")
    check(report.get("hardware_validated") is False, "This local-only flow cannot assert NPU acceptance")
    check(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", report["model_id"]) is not None,
          "Unsafe archive directory name")
    files = report["files"]
    summary = getattr(args, "test_summary", None)
    if summary is not None:
        handoff = report.get("handoff") or {}
        if str(handoff.get("test_summary_status", "")).startswith("pending"):
            check(summary.resolve() != Path(handoff.get("test_summary", "/unavailable")).resolve(),
                  "Provisional training-complete notice is not a completed evaluation summary")
            expected = handoff.get("final_test_summary_expected")
            if expected:
                check(summary.resolve() == Path(expected).resolve(),
                      "Use the final evaluation summary designated by the training handoff")
        check(summary.is_file() and not summary.is_symlink() and summary.stat().st_size > 0,
              "Missing/unsafe test summary attachment")
        check(summary.suffix.lower() in [".yaml", ".yml", ".json", ".md", ".txt", ".csv"],
              "Test summary must be a text report")
        destination = source / ("test_summary" + summary.suffix)
        if summary.resolve() != destination.resolve():
            check(not destination.exists(), "Test summary destination already exists")
            shutil.copyfile(summary, destination)
        files["test_summary"] = destination.name
        report.update(package_ready=True, pending_attachments=[])
        write_json(source / "export.json", report)
    required = ["pt", "onnx", "rknn", "metadata", "training_recipe", "source_statistics", "test_summary"]
    entries = []
    for attachment in getattr(args, "evaluation_attachment", None) or []:
        check(attachment.is_file() and not attachment.is_symlink() and attachment.stat().st_size > 0,
              "Missing/unsafe evaluation attachment")
        check(attachment.suffix.lower() in [".yaml", ".yml", ".json", ".md", ".txt", ".csv"],
              "Evaluation attachments must be text summaries/manifests")
        destination = source / ("evaluation_" + attachment.name)
        if attachment.resolve() != destination.resolve():
            if destination.exists():
                import filecmp
                check(filecmp.cmp(attachment, destination, shallow=False),
                      "Evaluation attachment destination has different content")
            else:
                shutil.copyfile(attachment, destination)
        recorded = report.setdefault("evaluation_attachments", [])
        if destination.name not in recorded:
            recorded.append(destination.name)
    if report.get("evaluation_attachments"):
        write_json(source / "export.json", report)
    for key in required:
        check(key in files, "Missing required package attachment: " + key)
        name = files[key]
        path = source / name
        check(Path(name).name == name and path.is_file() and not path.is_symlink()
              and path.stat().st_size > 0, "Missing/unsafe package entry: " + key)
        entries.append(name)
    for name in report.get("evaluation_attachments", []):
        path = source / name
        check(Path(name).name == name and path.is_file() and not path.is_symlink()
              and path.stat().st_size > 0, "Missing/unsafe extra evaluation attachment")
        entries.append(name)
    meta = yaml.safe_load((source / files["metadata"]).read_text(encoding="utf-8"))
    for key, expected in CONTRACT.items():
        check(meta.get(key) == expected, "Package metadata mismatch: " + key)
    check(meta.get("model_file") == files["rknn"], "Metadata points at another model")
    check(files["rknn"] == report["model_id"] + "_" + report["effective_precision"] + ".rknn",
          "RKNN precision/filename mismatch")
    check(report.get("names") == NAMES, "Export class order mismatch")
    # Reopen packaged PT/ONNX to catch accidental mixed artifacts.
    _, arch = load_model(source / files["pt"])
    check(arch == report["architecture"], "Package PT architecture differs from export record")
    validate_onnx(source / files["onnx"])
    for name in ["validation.json", "rknn_simulator.json"]:
        check((source / name).is_file() and not (source / name).is_symlink(),
              "Symlink validation record rejected")
        data = json.loads((source / name).read_text(encoding="utf-8"))
        check(data.get("status") == "PASS", "Validation failed: " + name)
        entries.append(name)
    # No archive entry may refer to a local symlink or a whole dataset.
    for name in entries + ["export.json"]:
        check((source / name).is_file() and not (source / name).is_symlink(),
              "Archive entries must be ordinary files")
        if Path(name).suffix not in [".pt", ".onnx", ".rknn"]:
            text = (source / name).read_text(encoding="utf-8")
            credential_key = r'(?im)["\']?(?:api[_-]?key|access[_-]?token|auth[_-]?token|password|client[_-]?secret|private[_-]?key|credentials)["\']?\s*[:=]\s*[^\s,}]+'
            check(re.search(credential_key, text) is None
                  and re.search(r"https?://[^/\s:@]+:[^/\s@]+@", text) is None
                  and "-----BEGIN PRIVATE KEY-----" not in text,
                  "Credential-like content rejected in " + name)
    destinations = [args.output.resolve()]
    if getattr(args, "zip_output", None):
        destinations.append(args.zip_output.resolve())
    for destination in destinations:
        check(not destination.exists(), "Output already exists: " + str(destination))
    if len(destinations) == 2:
        check(destinations[1].suffix == ".zip", "ZIP output must end in .zip")
    readme = (Path(__file__).parent / "README_candidate_zh.md").read_text(encoding="utf-8")
    readme = readme.replace("@MODEL@", files["rknn"]).replace("@MODEL_ID@", report["model_id"])
    readme = readme.replace("@PRECISION@", report["effective_precision"])
    (source / "README_替换与回滚.md").write_text(readme, encoding="utf-8")
    (source / "classes.txt").write_text(
        "\n".join("%d: %s" % item for item in enumerate(NAMES)) +
        "\n\n五类顺序必须保持一致；H仅为负样本，降落H由独立landing_detector处理。\n",
        encoding="utf-8")
    entries += ["README_替换与回滚.md", "classes.txt", "export.json"]
    destination = args.output.resolve()
    check(destination.suffixes[-2:] == [".tar", ".gz"], "Output must end in .tar.gz")
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents overwriting any previous release.
    with destination.open("xb") as handle:
        with tarfile.open(fileobj=handle, mode="w:gz") as archive:
            for name in entries:
                archive.add(source / name, arcname=report["model_id"] + "/" + name, recursive=False)
    print("CANDIDATE archive (NPU untested): " + str(destination))
    if getattr(args, "zip_output", None):
        zip_path = args.zip_output.resolve()
        zip_path.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(zip_path, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            for name in entries:
                archive.write(source / name, arcname=report["model_id"] + "/" + name)
        print("CANDIDATE ZIP (NPU untested): " + str(zip_path))


def common(parser):
    parser.add_argument("--metadata", type=Path, default=ROOT / "vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml")
    parser.add_argument("--detector-config", type=Path, default=ROOT / "vision_ws/src/uav_vision/config/target_detector_rknn.yaml")
    parser.add_argument("--decoder", type=Path, default=ROOT / "vision_ws/src/uav_vision/scripts/target_detector_rknn.py")
    parser.add_argument("--weights", type=Path, required=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("inspect")
    common(p)
    p.set_defaults(func=inspect_command)
    p = sub.add_parser("export")
    common(p)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--model-id", required=True)
    p.add_argument("--precision", choices=["fp32", "fp16"], default="fp16")
    p.add_argument("--image", type=Path, action="append", required=True)
    p.add_argument("--handoff", type=Path)
    p.add_argument("--training-recipe", type=Path)
    p.add_argument("--source-statistics", type=Path)
    p.add_argument("--test-summary", type=Path)
    p.add_argument("--workflow-smoke", action="store_true", help="Historical PT regression; never a final release")
    p.set_defaults(func=export_command)
    p = sub.add_parser("package")
    p.add_argument("--export", type=Path, required=True)
    p.add_argument("--test-summary", type=Path, help="Attach completed evaluation before packaging")
    p.add_argument("--evaluation-attachment", type=Path, action="append",
                   help="Include another completed text evaluation/manifest, never datasets")
    p.add_argument("--zip-output", type=Path, help="Also create a Windows-friendly ZIP")
    p.add_argument("--output", type=Path, required=True)
    p.set_defaults(func=package_command)
    args = parser.parse_args()
    try:
        args.func(args)
    except Exception as exc:
        print("BLOCKED/FAILED: " + str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
