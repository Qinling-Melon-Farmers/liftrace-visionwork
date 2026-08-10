#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""板端摄像头 GUI 稳定性与时延测试 (香橙派 + KS2A543 USB 相机)

阶段1: 30s GUI 显示, 统计 fps / 丢帧 / 帧间隔抖动 / read 耗时 / read->imshow 往返时延
阶段2: 60s 稳定性, 统计 fps / 丢帧 / 抖动 / 内存 / 温度
结果写入 /tmp/cam_report.json
"""
import os
import signal
import sys
import time
import json
import queue
import threading
import statistics

os.environ.setdefault("DISPLAY", ":0")

import cv2
import numpy as np

W, H = 1280, 720
PHASE1_S = 30.0
PHASE2_S = 60.0
WARMUP_FRAMES = 10

_TIME_LIMIT = {"deadline": 0.0}


def _alarm_handler(signum, frame):
    print("[guard] SIGALRM fired: forcing exit", flush=True)
    _TIME_LIMIT["deadline"] = 0.0  # 触发后让循环退出


def read_temp():
    """读 SoC 温度(摄氏度), 失败返回 None"""
    for i in range(10):
        p = "/sys/class/thermal/thermal_zone%d/temp" % i
        try:
            with open(p) as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            continue
    return None


def read_rss_mb():
    """读本进程 RSS (MB), 失败返回 None"""
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS"):
                    return round(int(line.split()[1]) / 1024.0, 1)
    except Exception:
        pass
    return None


def open_cam():
    """尝试 MJPG 1280x720; 协商失败则降级为相机默认格式。返回 (cap, info)"""
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if cap is None or not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if cap is None or not cap.isOpened():
        return None, {}
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    time.sleep(0.3)
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if fw != W or fh != H:
        # 协商失败 -> 降级默认格式
        cap.release()
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(0)
        if cap is None or not cap.isOpened():
            return None, {}
        time.sleep(0.3)
        fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join([chr((fourcc >> (8 * i)) & 0xFF) for i in range(4)])
    info = {
        "resolution": "%dx%d" % (fw, fh),
        "fourcc_code": fourcc_str,
        "cap_prop_fps": cap.get(cv2.CAP_PROP_FPS),
        "mjpg1280x720_negotiated": (fw == W and fh == H),
    }
    return cap, info


class CamReader:
    """线程读帧, 防止 V4L2 select() 永久阻塞导致进程卡死"""

    def __init__(self, cap):
        self.cap = cap
        self.q = queue.Queue(maxsize=4)
        self.stopped = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stopped:
            try:
                ret, frame = self.cap.read()
            except Exception:
                ret, frame = False, None
            try:
                self.q.put((ret, frame), timeout=1.0)
            except queue.Full:
                try:
                    self.q.get_nowait()
                    self.q.put((ret, frame), timeout=0.5)
                except Exception:
                    pass

    def read(self, timeout=2.0):
        """返回 (ret, frame); 超时无帧返回 (False, None)"""
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return (False, None)

    def stop(self):
        self.stopped = True


def pct(vals, p):
    if not vals:
        return None
    s = sorted(vals)
    idx = min(len(s) - 1, int(len(s) * p))
    return s[idx]


def run_phase(duration, use_gui, tag):
    cap, cap_info = open_cam()
    if cap is None:
        return {"error": "cannot open camera"}
    reader = CamReader(cap)

    win = "cam_gui_test_%s" % tag
    if use_gui:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, W // 2, H // 2)

    read_times = []   # cap.read 耗时
    r2i_times = []    # read -> imshow 往返(端到端)
    intervals = []    # 相邻帧到达间隔
    dropped = 0
    stalls = 0        # read 超时次数(流无数据)
    n = 0
    t_prev = None
    temps = []
    rss = []
    t0_start = time.perf_counter()
    last_sample = 0.0

    # warm-up 丢弃前 WARMUP_FRAMES 帧
    for _ in range(WARMUP_FRAMES):
        ok, _f = reader.read()
        if not ok:
            dropped += 1

    deadline = time.perf_counter() + duration + 20.0
    _TIME_LIMIT["deadline"] = deadline
    signal.signal(signal.SIGALRM, _alarm_handler)
    signal.alarm(int(duration) + 25)

    while True:
        if time.perf_counter() > deadline:
            print("[guard] wall-clock deadline exceeded, break", flush=True)
            break
        t0 = time.perf_counter()
        ret, frame = reader.read()
        t1 = time.perf_counter()
        if not ret or frame is None:
            dropped += 1
            if (t1 - t0) >= 1.9:
                stalls += 1
            continue
        n += 1
        read_times.append(t1 - t0)
        if t_prev is not None:
            intervals.append(t0 - t_prev)
        t_prev = t0
        if use_gui:
            cv2.imshow(win, frame)
            cv2.waitKey(1)
        t2 = time.perf_counter()
        r2i_times.append(t2 - t0)

        if t0 - last_sample >= 5.0:
            last_sample = t0
            temps.append(read_temp())
            rss.append(read_rss_mb())

        if t0 - t0_start >= duration:
            break

    signal.alarm(0)
    elapsed = time.perf_counter() - t0_start
    if use_gui:
        cv2.destroyAllWindows()
    reader.stop()
    cap.release()

    res = {
        "tag": tag,
        "duration_s": round(elapsed, 2),
        "cap_info": cap_info,
        "frames_ok": n,
        "frames_dropped": dropped,
        "read_stalls": stalls,
        "fps": round(n / elapsed, 2) if elapsed > 0 else 0.0,
        "interval_jitter_std_ms": round(statistics.pstdev(intervals) * 1000, 3) if len(intervals) > 1 else None,
        "read_ms": {
            "mean": round(statistics.mean(read_times) * 1000, 3) if read_times else None,
            "max": round(max(read_times) * 1000, 3) if read_times else None,
            "p95": round(pct(read_times, 0.95) * 1000, 3) if read_times else None,
            "min": round(min(read_times) * 1000, 3) if read_times else None,
        },
        "read_to_imshow_ms": {
            "mean": round(statistics.mean(r2i_times) * 1000, 3) if r2i_times else None,
            "max": round(max(r2i_times) * 1000, 3) if r2i_times else None,
            "p95": round(pct(r2i_times, 0.95) * 1000, 3) if r2i_times else None,
        },
        "rss_mb": {
            "min": min(rss) if rss else None,
            "max": max(rss) if rss else None,
            "mean": round(statistics.mean(rss), 1) if rss else None,
        },
        "temp_c": {
            "min": min(temps) if temps else None,
            "max": max(temps) if temps else None,
            "mean": round(statistics.mean(temps), 1) if temps else None,
        },
    }
    print("[%s] done: frames=%d dropped=%d stalls=%d fps=%.2f elapsed=%.1fs res=%s" % (
        tag, n, dropped, stalls, res["fps"], elapsed, cap_info), flush=True)
    return res


def main():
    quick = len(sys.argv) > 1 and sys.argv[1] == "quick"
    p1s = 8.0 if quick else PHASE1_S
    p2s = 0.0 if quick else PHASE2_S
    report = {"env": {}, "phases": {}}
    report["env"] = {
        "cv2_version": cv2.__version__,
        "display": os.environ.get("DISPLAY", ""),
        "python": sys.version.split()[0],
        "hostname": os.uname().nodename,
        "uname": os.uname().release,
    }
    try:
        p1 = run_phase(p1s, use_gui=True, tag="gui30s")
        report["phases"]["gui30s"] = p1
    except Exception as e:
        report["phases"]["gui30s"] = {"error": repr(e)}
    if p2s > 0:
        try:
            p2 = run_phase(p2s, use_gui=True, tag="gui60s")
            report["phases"]["gui60s"] = p2
        except Exception as e:
            report["phases"]["gui60s"] = {"error": repr(e)}

    with open("/tmp/cam_report.json", "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print("[report] written /tmp/cam_report.json", flush=True)


if __name__ == "__main__":
    main()
