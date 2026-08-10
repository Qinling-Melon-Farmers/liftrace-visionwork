#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""板端新旧视觉链推理性能统一执行器。

按序（避免 NPU 争用）运行三路评测并采样 CPU，汇总到 perf_report.json：
  1. legacy_rknn_dual    : 旧链 RKNN 路（standard 4 类 + tank, RKNNLite 双模型）
  2. new_chain_rknn      : 新链 RKNN 路（merged_standard_fp32.rknn 六分类, RKNNLite）
  3. legacy_pytorch_cpu  : 旧链 PyTorch 路（best.pt + tank.pt, torch CPU）

只做推理回放，不启动 ROS、不做任何执行机构操作。
"""
import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time

OUT_DIR = os.path.expanduser("~/board_eval")
REPORT_PATH = os.path.join(OUT_DIR, "perf_report.json")
NEW_MODEL = os.path.join(OUT_DIR, "merged_standard_fp32.rknn")
STD_RKNN = "/home/orangepi/Visual/src/yolov5_detect/best_rknn_model/best-rk3588.rknn"
TANK_RKNN = "/home/orangepi/Visual/src/yolov5_detect/tank_rknn_model/best-rk3588.rknn"
STD_PT = "/home/orangepi/Visual/src/yolov5_detect/best.pt"
TANK_PT = "/home/orangepi/Visual/src/yolov5_detect/tank.pt"


def read_proc_stat():
    with open("/proc/stat") as handle:
        fields = handle.readline().split()
    totals = sum(int(value) for value in fields[1:])
    idle = int(fields[4]) + int(fields[5])  # idle + iowait
    return totals, idle


def read_proc_pid(pid):
    with open("/proc/%d/stat" % pid) as handle:
        parts = handle.read().split()
    ticks = int(parts[13]) + int(parts[14])  # utime + stime
    rss_mb = int(parts[23]) * os.sysconf("SC_PAGE_SIZE") / 1048576.0
    return ticks, rss_mb


class CpuSampler(threading.Thread):
    """每秒采样: 目标进程 CPU%（相对单核 100%）与系统总 CPU%."""

    def __init__(self, pid):
        super().__init__(daemon=True)
        self.pid = pid
        self.process_pcts = []
        self.sys_pcts = []
        self.rss_mbs = []
        # 注意: 不要用 self._stop 名字, 会遮蔽 threading.Thread._stop 内部方法
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        ncpu = os.cpu_count() or 1
        try:
            prev_pt, prev_idle = read_proc_stat()
            prev_ticks, prev_rss = read_proc_pid(self.pid)
        except (OSError, ValueError):
            return
        while not self._stop_event.wait(1.0):
            try:
                cur_pt, cur_idle = read_proc_stat()
                cur_ticks, cur_rss = read_proc_pid(self.pid)
                d_total = cur_pt - prev_pt
                if d_total > 0:
                    self.sys_pcts.append((1.0 - (cur_idle - prev_idle) / d_total) * 100.0)
                    self.process_pcts.append((cur_ticks - prev_ticks) * 100.0 * ncpu / d_total)
                    self.rss_mbs.append(cur_rss)
                prev_pt, prev_idle = cur_pt, cur_idle
                prev_ticks, prev_rss = cur_ticks, cur_rss
            except (OSError, ValueError):
                continue


def run_test(name, cmd, models, report_entries, cpu_summary, log_path):
    print("===== BEGIN %s =====" % name, flush=True)
    started = time.perf_counter()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    sampler = CpuSampler(proc.pid)
    sampler.start()
    load_times = []
    load_pattern = re.compile(r"MODEL_READY\s+(\S+).*(?:load_ms=([0-9.]+))?")
    with open(log_path, "w", encoding="utf-8") as log:
        for line in proc.stdout:
            log.write(line)
            sys.stdout.write(line)
            match = load_pattern.search(line)
            if match:
                elapsed = (time.perf_counter() - started) * 1000.0
                if match.group(2):
                    load_times.append((match.group(1), float(match.group(2))))
                else:
                    load_times.append((match.group(1), round(elapsed, 1)))
    returncode = proc.wait()
    sampler.stop()
    try:
        sampler.join(timeout=3)
    except Exception as exc:  # noqa: BLE001 采样线程收尾异常不应中断评测
        print("WARN sampler join failed: %r" % exc, flush=True)
    elapsed_s = time.perf_counter() - started
    sys_cpu = sampler.sys_pcts
    proc_cpu = sampler.process_pcts
    rss = sampler.rss_mbs
    cpu_summary[name] = {
        "proc_cpu_pct": {"mean": round(sum(proc_cpu) / len(proc_cpu), 1) if proc_cpu else None,
                         "max": round(max(proc_cpu), 1) if proc_cpu else None},
        "sys_cpu_pct": {"mean": round(sum(sys_cpu) / len(sys_cpu), 1) if sys_cpu else None,
                        "max": round(max(sys_cpu), 1) if sys_cpu else None},
        "proc_rss_mb": {"mean": round(sum(rss) / len(rss), 1) if rss else None,
                        "max": round(max(rss), 1) if rss else None},
        "wall_seconds": round(elapsed_s, 2),
    }
    report_entries[name] = {
        "models": models,
        "load_times_ms": load_times,
        "returncode": returncode,
        "log": log_path,
    }
    print("===== END %s rc=%d wall=%.1fs =====" % (name, returncode, elapsed_s), flush=True)
    return returncode


def summarize(path, run_meta):
    if not os.path.exists(path):
        run_meta["error"] = "report json missing: %s" % path
        print("WARN report json missing: %s" % path, flush=True)
        return run_meta
    with open(path, encoding="utf-8") as handle:
        entry = json.load(handle)
    run_meta.update({
        "throughput_from_total_p50_fps": entry["derived"]["throughput_from_total_p50_fps"],
        "wall_fps": entry["derived"]["wall_fps"],
        "frames_measured": entry["sampling"]["frames_measured"],
        "frames_sampled": entry["sampling"]["frames_sampled"],
        "avg_total_ms": entry["metrics_ms"]["total_ms"]["mean"],
        "p50_total_ms": entry["metrics_ms"]["total_ms"]["p50"],
        "p95_total_ms": entry["metrics_ms"]["total_ms"]["p95"],
        "avg_infer_ms": entry["metrics_ms"].get("infer_ms", {}).get("mean"),
        "median_detections": entry["derived"]["median_detections"],
        "class_hist_total": entry["derived"]["class_hist_total"],
        "first_detections": entry["derived"]["first_detections"],
    })
    return run_meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, help="板端测试视频路径")
    parser.add_argument("--max-frames-pt", type=int, default=300,
                        help="PyTorch 路最多处理帧数（CPU 太慢，全量不现实）")
    parser.add_argument("--stride", type=int, default=1, help="RKNN 路采样步长")
    parser.add_argument("--output-width", type=int, default=1280)
    parser.add_argument("--skip-rknn", action="store_true")
    parser.add_argument("--skip-pt", action="store_true")
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    report = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "board": "orangepi RK3588 Ubuntu 20.04 aarch64",
        "video": args.video,
        "entries": {},
        "cpu": {},
    }

    if not args.skip_rknn:
        run_test(
            "legacy_rknn_dual",
            [sys.executable, os.path.join(OUT_DIR, "board_legacy_rknn_dual_viewer.py"),
             "--video", args.video,
             "--standard-model", STD_RKNN, "--tank-model", TANK_RKNN,
             "--json", os.path.join(OUT_DIR, "perf_legacy_rknn.json"),
             "--output-video", os.path.join(OUT_DIR, "out_legacy_rknn.mp4"),
             "--output-width", str(args.output_width),
             "--stride", str(args.stride), "--warmup", "5", "--no-window",
             "--u8-input"],
            {"standard_rknn": STD_RKNN, "tank_rknn": TANK_RKNN},
            report["entries"], report["cpu"],
            os.path.join(OUT_DIR, "log_legacy_rknn.txt"))
        summarize(os.path.join(OUT_DIR, "perf_legacy_rknn.json"),
                  report["entries"]["legacy_rknn_dual"])

        run_test(
            "new_chain_rknn",
            [sys.executable, os.path.join(OUT_DIR, "board_realtime_rknn_viewer.py"),
             "--video", args.video, "--json", os.path.join(OUT_DIR, "perf_new_rknn.json"),
             "--output-video", os.path.join(OUT_DIR, "out_new_rknn.mp4"),
             "--output-width", str(args.output_width),
             "--stride", str(args.stride), "--warmup", "5", "--no-window",
             NEW_MODEL],
            {"merged_standard_fp32_rknn": NEW_MODEL},
            report["entries"], report["cpu"],
            os.path.join(OUT_DIR, "log_new_rknn.txt"))
        summarize(os.path.join(OUT_DIR, "perf_new_rknn.json"),
                  report["entries"]["new_chain_rknn"])

    if not args.skip_pt:
        run_test(
            "legacy_pytorch_cpu",
            [sys.executable, os.path.join(OUT_DIR, "board_legacy_pt_video.py"),
             "--video", args.video,
             "--standard-model", STD_PT, "--tank-model", TANK_PT,
             "--json", os.path.join(OUT_DIR, "perf_legacy_pt.json"),
             "--output-video", os.path.join(OUT_DIR, "out_legacy_pt.mp4"),
             "--output-width", str(args.output_width),
             "--max-frames", str(args.max_frames_pt), "--warmup", "5"],
            {"standard_pt": STD_PT, "tank_pt": TANK_PT},
            report["entries"], report["cpu"],
            os.path.join(OUT_DIR, "log_legacy_pt.txt"))
        summarize(os.path.join(OUT_DIR, "perf_legacy_pt.json"),
                  report["entries"]["legacy_pytorch_cpu"])

    with open(REPORT_PATH, "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    print("REPORT_WRITTEN " + REPORT_PATH, flush=True)


if __name__ == "__main__":
    sys.exit(main())
