#!/usr/bin/env python3
"""Extract production C++ geometry and run image-only; no ROS, K, TF or weights."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import shlex
import subprocess
import time

import cv2
import numpy as np
import yaml

SPECS = {
    "circle": ("CircularDetectorNode", "bool CircularDetectorNode::detectBlueCircles", "cv::Mat CircularDetectorNode::drawDebug"),
    "cross": ("CrossDetectorNode", "bool CrossDetectorNode::detectRedCross", "cv::Mat CrossDetectorNode::drawDebugImage"),
    "landing": ("LandingDetectorNode", "bool LandingDetectorNode::detectLandingPad", "cv::Mat LandingDetectorNode::drawDebug"),
}


def dump(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def extract(root, kind):
    cls, start, end = SPECS[kind]
    rel = Path("vision_ws/src/uav_vision")
    src = (root / rel / "src" / (kind + "_detector_node.cpp")).read_text(encoding="utf-8")
    config_rel = rel / "config" / (kind + "_detector.yaml")
    config = yaml.safe_load((root / config_rel).read_text(encoding="utf-8"))
    params = re.findall(
        r'nh_\.param(?:<std::string>)?\("([^"]+)",\s*(\w+),\s*("[^"]*"|true|false|-?\d+(?:\.\d+)?)\s*\);', src)
    fields, effective = [], {}
    for key, name, default in params:
        default_value = json.loads(default)
        value = config.get(key, default_value)
        ctype = "std::string" if isinstance(default_value, str) else (
            "bool" if isinstance(default_value, bool) else "double" if isinstance(default_value, float) else "int")
        fields.append(ctype + " " + name + " = " + json.dumps(value) + ";")
        effective[key] = {"value": value, "source": str(config_rel) if key in config else "C++ default",
                          "member": name}
    missed = set(config) - set(effective)
    if missed:
        raise ValueError(f"Unparsed config keys: {kind}: {sorted(missed)}")
    methods = src[src.index(start):src.index(end)]
    signatures = re.findall(r"^(?:bool|double|int|void)\s+" + cls +
                            r"::\w+\([^)]*\)\s*(?:const)?(?=\s*\{)", methods, re.M)
    declarations = "\n".join(sig.replace(cls + "::", "") + ";" for sig in signatures)
    extra = "struct CircleCandidate {cv::RotatedRect ellipse; double quality;};" if kind == "circle" else ""
    generated = f"class {cls} {{public:\n{extra}\n" + "\n".join(fields) + "\n" + declarations + "\n};\n" + methods
    return generated, src, {"source": str(rel / "src" / (kind + "_detector_node.cpp")),
                            "methods": [re.search(cls + r"::(\w+)", s)[1] for s in signatures],
                            "parameters": effective}


CPP_MAIN = r"""
void vectorJson(const std::vector<double>& v) {
 std::cout << "[";
 for(size_t j=0;j<v.size();++j){if(j)std::cout<<",";std::cout<<v[j];}
 std::cout<<"]";
}
int main(int argc,char** argv) {
 if(argc!=2)return 2;
 cv::setNumThreads(1);
 uav_vision::CircularDetectorNode c;
 uav_vision::CrossDetectorNode x;
 uav_vision::LandingDetectorNode h;
 std::ifstream input(argv[1]);std::string file;int idx=0;
 while(std::getline(input,file)){
  cv::Mat original=cv::imread(file,cv::IMREAD_COLOR);
  if(original.empty()){std::cerr<<"imread failed at "<<idx<<"\n";return 3;}
  cv::Mat image=original;
  auto begin=std::chrono::steady_clock::now();
  bool enable_resize_=c.enable_resize_, preserve_aspect_ratio_=c.preserve_aspect_ratio_;
  int resize_width_=c.resize_width_,resize_height_=c.resize_height_;
CIRCLE_PREPROCESS
  cv::Mat cmask;std::vector<std::vector<cv::Point>> cc;
  std::vector<uav_vision::CircularDetectorNode::CircleCandidate> candidates;
  c.detectBlueCircles(image,candidates,cmask,cc);
  double circle_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  std::cout<<std::setprecision(12)<<"{\"index\":"<<idx++<<",\"width\":"<<original.cols<<",\"height\":"<<original.rows
   <<",\"letterbox\":{\"width\":"<<image.cols<<",\"height\":"<<image.rows<<",\"inverse_scale_x\":"<<scale_x
   <<",\"inverse_scale_y\":"<<scale_y<<",\"offset_x\":"<<offset_x<<",\"offset_y\":"<<offset_y<<"},\"circle\":[";
  for(size_t i=0;i<candidates.size();++i){
   if(i)std::cout<<",";
   const auto &candidate=candidates[i];
   const cv::Point2f center(
       static_cast<float>((candidate.ellipse.center.x-offset_x)*scale_x),
       static_cast<float>((candidate.ellipse.center.y-offset_y)*scale_y));
   const float radius=static_cast<float>(((candidate.ellipse.size.width*scale_x)+(candidate.ellipse.size.height*scale_y))/4.0);
   float cx=std::max(0.0f,std::min(center.x,static_cast<float>(original_width-1)));
   float cy=std::max(0.0f,std::min(center.y,static_cast<float>(original_height-1)));
   int bx=std::max(0,static_cast<int>(cx-radius)),by=std::max(0,static_cast<int>(cy-radius));
   int x2=std::min(original_width,static_cast<int>(cx+radius)),y2=std::min(original_height,static_cast<int>(cy+radius));
   std::cout<<"{\"center_px\":["<<cx<<","<<cy<<"],\"radius_px\":"<<radius<<",\"quality\":"<<candidate.quality
    <<",\"roi_xywh\":["<<bx<<","<<by<<","<<std::max(0,x2-bx)<<","<<std::max(0,y2-by)
    <<"],\"processed_center_px\":["<<candidate.ellipse.center.x<<","<<candidate.ellipse.center.y
    <<"],\"processed_radius_px\":"<<(candidate.ellipse.size.width+candidate.ellipse.size.height)/4.0<<"}";
  }
  std::cout<<"],\"circle_ms\":"<<circle_ms;
  image=original;
  begin=std::chrono::steady_clock::now();
  bool enable_gaussian_blur_=x.enable_gaussian_blur_;int blur_kernel_size_=x.blur_kernel_size_;
CROSS_PREPROCESS
  cv::Point2f xc;double xa=0;cv::Mat xm;std::vector<std::vector<cv::Point>> xcont;std::vector<double> xq;
  bool xf=x.detectRedCross(processed,xc,xa,xcont,xm,xq);
  if(xf&&x.enable_black_ring_check_)xf=x.checkBlackOuterRing(original,xc);
  double cross_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  std::cout<<",\"red_cross\":{\"found\":"<<(xf?"true":"false");
  if(xf){std::cout<<",\"center_px\":["<<xc.x<<","<<xc.y<<"],\"area_px2\":"<<xa<<",\"quality\":"<<xq.at(4)<<",\"metrics\":";vectorJson(xq);}
  std::cout<<"},\"cross_ms\":"<<cross_ms;
  begin=std::chrono::steady_clock::now();
  cv::Point2f hc;float hr=0;cv::Mat hm;cv::Rect hb;std::vector<std::vector<cv::Point>> hcont;std::vector<double> hq;
  bool hf=h.detectLandingPad(original,hc,hr,hm,hcont,hq,hb);
  double landing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  std::cout<<",\"landing\":{\"found\":"<<(hf?"true":"false");
  if(hf){std::cout<<",\"center_px\":["<<hc.x<<","<<hc.y<<"],\"radius_px\":"<<hr<<",\"quality\":"<<hq.at(3)
   <<",\"roi_xywh\":["<<hb.x<<","<<hb.y<<","<<hb.width<<","<<hb.height<<"],\"metrics\":";vectorJson(hq);}
  std::cout<<"},\"landing_ms\":"<<landing_ms<<"}\n"<<std::flush;
 }
 return 0;
}
"""


def write_report(output, summary, meta):
    counts = summary["counts"]
    methods = "\n".join(f"- {key}: {', '.join(v['methods'])}" for key, v in meta.items())
    (output / "REPORT.md").write_text(f"""# Independent offline image geometry

61-frame Huawei phone-video sample; all coordinates and radii below are original-image pixels.
No camera calibration, onboard K, TF, world/map coordinates or metric accuracy are used.

## Results
- Circle-positive frames: {counts['circle_frames']}/61; accepted candidates: {counts['circle_candidates']}.
- Red-cross-positive frames: {counts['red_cross_frames']}/61.
- H-positive frames (raw landing geometry): {counts['landing_frames']}/61.
- These are detector outputs, not annotated true positives or recall. No manual instance/center ground truth.

## Production methods extracted (conditional calls retain defaults)
{methods}
- landing also calls the unchanged header helpers landingHMask and detectHStrokes.
- The method bodies are extracted verbatim at runtime from the recorded checkout. ROS-free class shells are generated.
- Circle imageCallback resize block is extracted verbatim. publishDetections pixel inverse-transform,
  float conversions, clipping and radius-derived ROI expressions are reproduced in the harness.
- Cross imageCallback Gaussian-blur block is extracted verbatim; native image dimensions are retained.
  Strict/relaxed validation, shared template IoU and fourfold symmetry all run in production C++.
  checkBlackOuterRing is extracted but not called with the default enable_black_ring_check=false.
- YAML values override parsed C++ constructor defaults. See effective_parameters.json.
  Circle resize=true, aspect-preserving 640x512, min quality=0.70; landing grayscale_otsu,
  H structure check=true, H stroke fallback=false. No threshold sweeps or forced fallback.

## Boundaries
- This bypasses CameraInfo-initialized callback guards, ROS image transport/cv_bridge, timestamps,
  source-completion publication, queues and online launch overrides.
- H is evaluated directly on every image. Production default align_mode=disabled and
  process_only_in_landing_mode=true would suppress the node until landing mode; no such mode is
  inferred from this phone recording. An H geometry positive is not an online H publication.
- Circle radius is mean fitted ellipse semiaxes, not a metric radius. Cross center is contour moments,
  not YOLO box center. Cross returns only its single largest accepted contour, H only its best candidate.
- No YOLO identities/boxes, production refiner association/conflict gates, temporal memory,
  projection, TF freshness, alignment, release permission or flight/actuator chain are exercised.
- No ROS node/master, simulator, device or board is started. C++ uses host OpenCV via pkg-config;
  Python uses the existing rl_drone environment. Times are host offline measurements, not board FPS.
- Sampled phone footage at approximately 2 Hz is not a synchronized onboard camera/pose stream.
  No complete flight-chain PASS, semantic accuracy, map precision or board acceptance is claimed.

## Artifacts and joining
- candidates.jsonl / results.json: one row per manifest row, unchanged frame, t and file metadata;
  circle list and red_cross/landing objects use original pixels. Found=false has no fabricated center.
- circle_candidates.csv: flat circle table including processed and original coordinates.
- per_frame.csv: all 61 frames including zero-candidate frames.
- overlays/, contact_sheet.jpg, index.html: visual review only; predictions are not ground truth.
- production_geometry.cpp, production_geometry executable, files.txt, build.log and
  provenance.json retain compilation inputs/environment, source HEAD/status, and extraction coverage.
- Join the main agent's YOLO outputs on frame or file, using circle ROI/center/radius in the original
  pixel domain. Production refiner association remains a separate subsequent check.

## Integration schema and focused interval
Preferred fields: circles=[{{center_x,center_y,radius,quality,roi}}],
cross={{found,x,y,area,quality,roi}}, landing={{found,x,y,radius,area,quality,roi}}.
ROI is [x,y,width,height] in original pixels; missing values are null.
Detailed circle/red_cross fields remain for diagnostics. Cross ROI reproduces publishDetection:
area-derived square, not contour bbox, with no right/bottom image clipping.
focus_11_5_to_12_5.json isolates frames 345/360/375. At 11.505/12.006s circles pass
with quality 0.9665/0.8555; at 12.506s none pass. Geometry cannot distinguish pillbox/panzer.
The circle border guard uses the padded canvas. Fitted ellipses may extend into padding:
345/360 mean-radius circles extend below the original image, and their published ROI is clipped.
A geometry positive does not certify a fully visible ring or resolve a class error.
verification.json records sequence, quality and inverse-transform checks.

## Reproduction
Activate /home/xhj/miniconda3/etc/profile.d/conda.sh and conda activate rl_drone;
run docs/verification/competition_video_20261015/geometry_offline.py with --root checkout
(optionally --frames manifest and --output destination). Defaults resolve this dataset inside checkout.
No runtime source, shared documentation or git commit is changed by this script.
""", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[3])
    ap.add_argument("--frames", type=Path)
    ap.add_argument("--output", type=Path)
    args = ap.parse_args()
    root = args.root.resolve()
    manifest = args.frames or root / "\u8bd5\u98de\u4ea7\u7269/competition_video_20261015/frames.json"
    output = args.output or manifest.parent / "output/geometry"
    output.mkdir(parents=True, exist_ok=True)
    frames = json.loads(manifest.read_text(encoding="utf-8"))
    if len(frames) != 61:
        raise ValueError(f"Expected 61 frames, got {len(frames)}")
    paths = [(Path(f["file"]) if Path(f["file"]).is_absolute() else manifest.parent / f["file"]).resolve() for f in frames]
    if any(not p.is_file() or "\n" in str(p) or "\r" in str(p) for p in paths):
        raise ValueError("Missing or newline-containing frame path")
    generated, sources, meta = {}, {}, {}
    for kind in SPECS:
        generated[kind], sources[kind], meta[kind] = extract(root, kind)
    circle_pre = sources["circle"].split("  const int original_width = image.cols;", 1)[1].split("  cv::Mat debug_mask;", 1)[0]
    circle_pre = "  const int original_width = image.cols;" + circle_pre
    cross_pre = sources["cross"].split("  cv::Mat processed;", 1)[1].split("  cv::Point2f cross_center;", 1)[0]
    cross_pre = "  cv::Mat processed;" + cross_pre
    includes = """#include <opencv2/opencv.hpp>
#include <uav_vision/h_stroke_detector.h>
#include <uav_vision/landing_h_mask.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
"""
    cpp = includes + "\nnamespace uav_vision {\n" + "\n".join(generated.values()) + "\n}\n"
    cpp += CPP_MAIN.replace("CIRCLE_PREPROCESS", circle_pre).replace("CROSS_PREPROCESS", cross_pre)
    cpp_path, executable = output / "production_geometry.cpp", output / "production_geometry"
    cpp_path.write_text(cpp, encoding="utf-8")
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "opencv4"], text=True))
    command = ["g++", "-O2", "-std=c++14", str(cpp_path), "-I" + str(root / "vision_ws/src/uav_vision/include"), *flags, "-o", str(executable)]
    with (output / "build.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    dump(output / "effective_parameters.json", {k:v["parameters"] for k,v in meta.items()})
    provenance = {
        "head": subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip(),
        "git_status_before_run": subprocess.check_output(["git", "-C", str(root), "status", "--short"], text=True),
        "compiler": subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
        "opencv_cpp": subprocess.check_output(["pkg-config", "--modversion", "opencv4"], text=True).strip(),
        "opencv_python": cv2.__version__, "compile_command": command, "methods": meta,
        "manifest": str(manifest), "frame_count": len(frames),
        "circle_callback_preprocess_extracted": True, "cross_callback_blur_extracted": True,
        "coordinates": "original image pixels only; no K/TF/world/map"}
    dump(output / "provenance.json", provenance)
    (output / "files.txt").write_text("\n".join(map(str, paths)) + "\n", encoding="utf-8")
    started = time.monotonic()
    with (output / "raw_results.jsonl").open("w") as stream:
        subprocess.run([str(executable), str(output / "files.txt")], stdout=stream, check=True)
    raw = [json.loads(line) for line in (output / "raw_results.jsonl").read_text().splitlines()]
    if len(raw) != len(frames) or [r["index"] for r in raw] != list(range(len(frames))):
        raise ValueError("Incomplete or reordered runner output")
    results = [{**frame, **result} for frame, result in zip(frames, raw)]
    # Flat integration schema; retain detailed legacy fields for diagnostics.
    for row in results:
        row["circles"] = [
            {"center_x": c["center_px"][0], "center_y": c["center_px"][1],
             "radius": c["radius_px"], "quality": c["quality"], "roi": c["roi_xywh"]}
            for c in row["circle"]]
        cross = row["red_cross"]
        if cross["found"]:
            x, y = cross["center_px"]
            # Reproduce CrossDetectorNode::publishDetection ROI (not contour bbox).
            half = max(24, int(math.sqrt(max(cross["area_px2"], 1.0)) * 0.8))
            row["cross"] = {"found": True, "x": x, "y": y, "area": cross["area_px2"],
                            "quality": cross["quality"],
                            "roi": [max(0, int(x)-half), max(0, int(y)-half), 2*half, 2*half]}
        else:
            row["cross"] = dict(found=False, x=None, y=None, area=None, quality=None, roi=None)
        h = row["landing"]
        if h["found"]:
            h.update(x=h["center_px"][0], y=h["center_px"][1], radius=h["radius_px"],
                     area=h["metrics"][0], roi=h["roi_xywh"])
        else:
            h.update(x=None, y=None, radius=None, area=None, quality=None, roi=None)
    dump(output / "results.json", results)
    focus = [r for r in results if 11.4 <= r["t"] <= 12.6]
    dump(output / "focus_11_5_to_12_5.json",
         [{"frame": r["frame"], "t": r["t"], "file": r["file"],
           "circles": r["circles"], "cross": r["cross"], "landing": r["landing"]} for r in focus])
    residuals = []
    for row in results:
        b = row["letterbox"]
        for c in row["circle"]:
            expected = (np.array(c["processed_center_px"])-[b["offset_x"], b["offset_y"]]) * [b["inverse_scale_x"], b["inverse_scale_y"]]
            expected = np.clip(expected, [0, 0], [row["width"]-1, row["height"]-1])
            residuals.append(float(np.max(np.abs(expected-np.array(c["center_px"])))))
            assert abs(c["radius_px"]-c["processed_radius_px"]*b["inverse_scale_x"]) < 1e-4
            assert c["quality"] >= meta["circle"]["parameters"]["circle_min_quality"]["value"]
    assert max(residuals, default=0) < 1e-4
    dump(output / "verification.json", {"frames": len(results), "max_inverse_transform_residual_px": max(residuals, default=0),
          "radius_inverse_transform": "PASS", "circle_min_quality": "PASS",
          "input_sequence_complete": True, "semantic_accuracy": "not evaluated"})
    (output / "candidates.jsonl").write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in results), encoding="utf-8")
    counts = {"circle_frames": sum(bool(r["circle"]) for r in results),
              "circle_candidates": sum(len(r["circle"]) for r in results),
              "red_cross_frames": sum(r["red_cross"]["found"] for r in results),
              "landing_frames": sum(r["landing"]["found"] for r in results)}
    summary = {"counts": counts, "frame_count": len(results), "geometry_wall_seconds": time.monotonic()-started,
               "dimensions": sorted({(r["width"],r["height"]) for r in results}),
               "positive_frames": {k:[r["frame"] for r in results if (r["circle"] if k=="circle" else r[k]["found"])] for k in ["circle","red_cross","landing"]},
               "timing_ms": {k:{"median":float(np.median([r[k] for r in results])),
                                 "p95":float(np.percentile([r[k] for r in results],95))}
                             for k in ["circle_ms","cross_ms","landing_ms"]}}
    dump(output / "summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    with (output / "per_frame.csv").open("w", newline="", encoding="utf-8") as f:
        writer=csv.writer(f);writer.writerow(["frame","t","file","circle_count","red_cross_found","landing_found","circle_ms","cross_ms","landing_ms"])
        for r in results: writer.writerow([r["frame"],r["t"],r["file"],len(r["circle"]),r["red_cross"]["found"],r["landing"]["found"],r["circle_ms"],r["cross_ms"],r["landing_ms"]])
    with (output / "circle_candidates.csv").open("w", newline="", encoding="utf-8") as f:
        writer=csv.writer(f);writer.writerow(["frame","t","file","candidate_index","x_px","y_px","radius_px","quality","processed_x_px","processed_y_px","processed_radius_px","roi_x","roi_y","roi_w","roi_h"])
        for r in results:
            for i,c in enumerate(r["circle"]): writer.writerow([r["frame"],r["t"],r["file"],i,*c["center_px"],c["radius_px"],c["quality"],*c["processed_center_px"],c["processed_radius_px"],*c["roi_xywh"]])
    overlay_dir = output / "overlays";overlay_dir.mkdir(exist_ok=True)
    thumbs=[]
    for r,path in zip(results,paths):
        im=cv2.imread(str(path))
        for i,c in enumerate(r["circle"]):
            center=tuple(round(v) for v in c["center_px"])
            cv2.circle(im,center,round(c["radius_px"]),(255,180,0),3)
            cv2.drawMarker(im,center,(255,180,0),cv2.MARKER_CROSS,20,2)
            cv2.putText(im,f'C{i} q={c["quality"]:.3f}',(max(0,center[0]-60),max(30,center[1]-20)),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,180,0),2)
        for key,label,color in [("red_cross","X",(0,0,255)),("landing","H",(0,220,0))]:
            det=r[key]
            if det["found"]:
                center=tuple(round(v) for v in det["center_px"])
                cv2.drawMarker(im,center,color,cv2.MARKER_CROSS,32,3)
                cv2.putText(im,f'{label} q={det["quality"]:.3f}',(max(0,center[0]-60),max(30,center[1]-30)),cv2.FONT_HERSHEY_SIMPLEX,.8,color,2)
        cv2.putText(im,f'frame {r["frame"]} t={r["t"]:.2f}s C={len(r["circle"])} X={int(r["red_cross"]["found"])} H={int(r["landing"]["found"])}',(20,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
        cv2.imwrite(str(overlay_dir/f'{r["frame"]:05d}.jpg'),im,[cv2.IMWRITE_JPEG_QUALITY,88])
        thumb=cv2.resize(im,(480,270),interpolation=cv2.INTER_AREA)
        thumbs.append(thumb)
    sheet=np.zeros((((len(thumbs)+3)//4)*270,1920,3),dtype=np.uint8)
    for i,im in enumerate(thumbs):sheet[(i//4)*270:(i//4+1)*270,(i%4)*480:(i%4+1)*480]=im
    cv2.imwrite(str(output/"contact_sheet.jpg"),sheet,[cv2.IMWRITE_JPEG_QUALITY,90])
    (output/"index.html").write_text('<meta charset="utf-8"><title>Offline geometry</title><p>Phone pixels only; geometry predictions, no ground truth or flight acceptance. C=blue circle, X=red cross, H=landing geometry.</p><a href="REPORT.md">Scope</a> | <a href="summary.json">Counts</a><br><img style="width:100%" src="contact_sheet.jpg">'+"".join(f'<p>Frame {r["frame"]}, {r["t"]:.2f}s</p><a href="overlays/{r["frame"]:05d}.jpg"><img width="640" src="overlays/{r["frame"]:05d}.jpg"></a>' for r in results),encoding="utf-8")
    write_report(output,summary,meta)
    print("Complete: "+str(output),flush=True)


if __name__ == "__main__":
    main()
