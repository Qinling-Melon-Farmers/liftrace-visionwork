#!/usr/bin/env python3
"""Compile actual circle/cross methods and compare the same files without ROS.

Baseline circle source is read from a Git revision (default HEAD); new source
is read from disk. Thresholds/resize come from the checked-in YAML. Exported
frame manifests accept file/image/source and t. Real centre deltas and temporal
steps are agreement/motion diagnostics, NEVER unlabelled ground-truth error.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import shlex
import subprocess

import cv2
import numpy as np
import yaml


PACKAGE = "vision_ws/src/uav_vision"


def fields(source, overrides):
    result = []
    for key, name, value in re.findall(
            r'nh_\.param\("([^"]+)",\s*(\w+),\s*([^;\n]+?)\);', source):
        value = value.strip()
        if key in overrides:
            value = str(overrides[key]).lower()
        kind = "bool" if value in ("true", "false") else (
            "double" if "." in value else "int")
        if re.fullmatch(r'(true|false|[-+0-9.eE]+)', value):
            result.append(f"{kind} {name} = {value};")
    return "\n".join(result)


def circle_class(source, name, config):
    methods = source[source.index("bool CircularDetectorNode::detectBlueCircles"):
                     source.index("cv::Mat CircularDetectorNode::drawDebug")]
    resize = source[source.index("  const int original_width = image.cols;"):
                    source.index("  cv::Mat debug_mask;")]
    declaration = r'''
class NAME { public:
 struct CircleCandidate { cv::RotatedRect ellipse; double quality; };
 FIELDS
 bool detectBlueCircles(const cv::Mat&,std::vector<CircleCandidate>&,
                       cv::Mat&,std::vector<std::vector<cv::Point>>&);
 void run(const cv::Mat& input,int frame,const char* variant,int repeats) {
  cv::Mat image=input;
  RESIZE
  cv::Mat mask;std::vector<std::vector<cv::Point>> contours;
  std::vector<CircleCandidate> candidates;std::vector<double> timings;
  for(int i=0;i<repeats+1;++i) {
   candidates.clear();contours.clear();
   auto start=std::chrono::steady_clock::now();
   detectBlueCircles(image,candidates,mask,contours);
   const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
   if(i>0) timings.push_back(ms);
  }
  std::sort(timings.begin(),timings.end());double ms=timings[timings.size()/2];
  if(candidates.empty()) {
   std::cout<<frame<<"\t"<<variant<<"\tcircle\t-1\t0\t0\t0\t0\t"<<ms<<"\t0\t0\t360\t0\n";
  }
  for(size_t i=0;i<candidates.size();++i) {
   const auto& candidate=candidates[i];EllipseSupport support;
   for(const auto& contour:contours) {
    if(contour.size()<5) continue;
    auto fit=cv::fitEllipse(contour);
    if(cv::norm(fit.center-candidate.ellipse.center)<1e-4 &&
       std::abs(fit.size.width-candidate.ellipse.size.width)<1e-4 &&
       std::abs(fit.size.height-candidate.ellipse.size.height)<1e-4) {
     support=ellipseSupport(contour,candidate.ellipse);break;
    }
   }
   std::cout<<frame<<"\t"<<variant<<"\tcircle\t"<<i<<"\t"<<candidates.size()<<"\t"
    <<(candidate.ellipse.center.x-offset_x)*scale_x<<"\t"
    <<(candidate.ellipse.center.y-offset_y)*scale_y<<"\t"<<candidate.quality<<"\t"<<ms<<"\t"
    <<support.normal_rms_px<<"\t"<<support.visible_arc_fraction<<"\t"
    <<support.largest_gap_degrees<<"\t"
    <<(candidate.ellipse.size.width*scale_x+candidate.ellipse.size.height*scale_y)/4<<"\n";
  }
 }
};
METHODS
'''
    return (declaration.replace("FIELDS", fields(source, config))
            .replace("RESIZE", resize).replace("METHODS", methods)
            .replace("CircularDetectorNode", name).replace("NAME", name))


def cross_class(source, header, config):
    methods = source[source.index("bool CrossDetectorNode::detectRedCross"):
                     source.index("bool CrossDetectorNode::checkBlackOuterRing")]
    prototypes = header[header.index("  bool detectRedCross"):
                        header.index("  bool checkBlackOuterRing")]
    return "class CrossDetectorNode { public:\n" + fields(source, config) + prototypes + "};\n" + methods


def build_runner(root, output, revision, repeats):
    path = PACKAGE + "/src/circle_detector_node.cpp"
    baseline = subprocess.check_output(["git", "show", f"{revision}:{path}"], cwd=root, text=True)
    current = (root/path).read_text()
    (output/"baseline_circle.cpp").write_text(baseline)
    (output/"candidate_circle.cpp").write_text(current)
    circle_config = yaml.safe_load((root/PACKAGE/"config/circle_detector.yaml").read_text())
    cross_config = yaml.safe_load((root/PACKAGE/"config/cross_detector.yaml").read_text())
    if cross_config.get("enable_black_ring_check",False):
        raise ValueError("This offline cross runner does not reproduce optional black-ring checking")
    cpp = r'''#include <opencv2/opencv.hpp>
#include <uav_vision/circle_geometry.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
namespace uav_vision {
'''
    cpp += circle_class(baseline, "OldCircle", circle_config)
    cpp += circle_class(current, "DefaultCircle", circle_config)
    cpp += circle_class(current, "NewCircle", dict(circle_config,circle_quality_ordered_nms=True))
    cpp += cross_class((root/PACKAGE/"src/cross_detector_node.cpp").read_text(),
                       (root/PACKAGE/"include/uav_vision/cross_detector_node.h").read_text(), cross_config)
    cpp += r'''
}
int main(int argc,char**argv) {
 cv::setNumThreads(1);std::cout<<std::setprecision(10);
 uav_vision::OldCircle old;uav_vision::NewCircle candidate;uav_vision::DefaultCircle defaults;
 uav_vision::CrossDetectorNode cross;
 const int repeats=std::stoi(argv[2]);std::ifstream files(argv[1]);std::string file;int frame=0;
 std::cout<<"frame\tvariant\tdetector\tindex\tcount\tx\ty\tquality\tms\tresidual_px\tarc_fraction\tgap_deg\tradius\n";
 while(std::getline(files,file)) {
  auto image=cv::imread(file);if(image.empty()) return 2;
  // Alternate order to avoid systematically favouring the second timing.
  if(frame%2) {candidate.run(image,frame,"new",repeats);old.run(image,frame,"old",repeats);}
  else {old.run(image,frame,"old",repeats);candidate.run(image,frame,"new",repeats);}
  defaults.run(image,frame,"default",repeats);
  cv::Point2f center;double area=0;cv::Mat mask;std::vector<std::vector<cv::Point>> contours;
  std::vector<double> quality,timings;bool found=false;
  for(int i=0;i<=repeats;++i) {
   auto start=std::chrono::steady_clock::now();cv::Mat filtered=image;
   if(cross.enable_gaussian_blur_) {
    const int k=cross.blur_kernel_size_%2 ? cross.blur_kernel_size_ : cross.blur_kernel_size_+1;
    cv::GaussianBlur(image,filtered,cv::Size(k,k),0);
   }
   found=cross.detectRedCross(filtered,center,area,contours,mask,quality);
   if(i>0) timings.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  }
  std::sort(timings.begin(),timings.end());
  std::cout<<frame<<"\tunchanged\tcross\t"<<(found?0:-1)<<"\t"<<found<<"\t"
   <<(found?center.x:0)<<"\t"<<(found?center.y:0)<<"\t"<<(found?quality[4]:0)<<"\t"
   <<timings[timings.size()/2]<<"\t0\t0\t0\t0\n";
  ++frame;
 }
}
'''
    generated = output/"production_geometry.cpp"
    generated.write_text(cpp)
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "opencv4"], text=True))
    exe = output/"production_geometry"
    subprocess.run(["g++", "-std=c++14", "-O2", str(generated), "-I"+str(root/PACKAGE/"include"),
                    *flags, "-o", str(exe)], check=True)
    test = output/"circle_geometry_test"
    subprocess.run(["g++", "-std=c++14", "-O2", str(root/PACKAGE/"test/circle_geometry_test.cpp"),
                    "-I"+str(root/PACKAGE/"include"), *flags, "-o", str(test)], check=True)
    subprocess.run([str(test)], check=True)
    return exe, circle_config, cross_config


def synthetic(output):
    folder = output/"synthetic"
    folder.mkdir(exist_ok=True)
    rows = []
    # Perspective maps with modest foreshortening (production aspect >= .85).
    for kind in ("ring", "ring_nick", "ring_multiple", "cross", "disk", "rectangle", "arc", "cropped_ring", "red_disk", "red_rectangle"):
        for perspective in (0.0, .0004, .0008):
            for phase in range(6):
                image = np.full((512,640,3),210,np.uint8)
                center = (320,256)
                if kind.startswith("ring"):
                    cv2.circle(image, center,75,(255,0,0),14)
                    # Off-centre interior icon must never move a standard-board centre.
                    cv2.rectangle(image,(295,232),(330,265),(20,20,20),-1)
                    if kind == "ring_nick":
                        # Small inner-edge defect leaves the outside ring intact.
                        cv2.rectangle(image,(376,247),(390,265),(210,210,210),-1)
                    elif kind == "ring_multiple":
                        cv2.circle(image,(130,130),45,(255,0,0),12)
                elif kind == "cross":
                    cv2.rectangle(image,(260,238),(380,274),(0,0,255),-1)
                    cv2.rectangle(image,(302,196),(338,316),(0,0,255),-1)
                elif kind in ("disk","red_disk"):
                    cv2.circle(image,center,75,(0,0,255) if kind.startswith("red") else (255,0,0),-1)
                elif kind in ("rectangle","red_rectangle"):
                    cv2.rectangle(image,(245,190),(395,322),(0,0,255) if kind.startswith("red") else (255,0,0),-1)
                elif kind == "arc":
                    cv2.ellipse(image,center,(75,75),0,40,285,(255,0,0),14)
                else:
                    cv2.circle(image,(8,256),75,(255,0,0),14)
                angle = phase * 7
                rotation = np.vstack([cv2.getRotationMatrix2D(center,angle,1),[0,0,1]])
                to_origin=np.array([[1,0,-320],[0,1,-256],[0,0,1]],float)
                projective = np.linalg.inv(to_origin) @ np.array(
                    [[1,0,0],[0,1,0],[perspective,0,1]],float) @ to_origin
                jitter = np.array([[1,0,.35*math.sin(phase)],[0,1,.35*math.cos(phase)],[0,0,1]],float)
                transform = jitter @ projective @ rotation
                image = cv2.warpPerspective(image,transform,(640,512),borderValue=(210,210,210))
                file = folder/f"{kind}_{perspective}_{phase}.png"
                cv2.imwrite(str(file),image)
                target = cv2.perspectiveTransform(np.array([[center]],np.float32),transform)[0,0].tolist()
                rows.append(dict(file=str(file),group="synthetic_"+kind,t=phase,
                                 sequence=f"synthetic_{kind}_{perspective}",
                                 perspective=perspective,
                                 truth=target if kind in ("ring","ring_nick","cross") else None,
                                 expected="circle" if kind.startswith("ring") else ("cross" if kind=="cross" else "negative")))
    return rows


def perspective_demonstration():
    result=[]
    for k in (0,.0004,.0008):
        # Circle x^2+y^2=R^2, map x'=x/(1+k*x), y'=y/(1+k*x).
        # Conic centre x'=-k*R^2/(1-k^2*R^2), circle centre projection=0.
        radius=75
        theta=np.linspace(0,2*np.pi,720,endpoint=False)
        points=np.column_stack((radius*np.cos(theta),radius*np.sin(theta))).astype(np.float32)
        projected=cv2.perspectiveTransform(points.reshape(-1,1,2),np.array([[1,0,0],[0,1,0],[k,0,1]],float))
        ellipse=cv2.fitEllipse(projected)
        analytic=-k*radius**2/(1-k*k*radius**2)
        result.append(dict(perspective=k,radius=radius,circle_center_projection=[0,0],
                           analytic_ellipse_center=[analytic,0],fitted_ellipse_center=list(ellipse[0]),
                           difference_px=math.dist((0,0),ellipse[0])))
    return result


def stats(values):
    if not values:
        return dict(n=0,min=None,median=None,p95=None,max=None)
    return dict(n=len(values),median=float(np.median(values)),
                min=float(np.min(values)),p95=float(np.percentile(values,95)),max=float(np.max(values)))


def summarize(rows, observations):
    by_frame = {}
    for row in observations:
        for key in ("frame","index","count"):
            row[key] = int(row[key])
        for key in ("x","y","quality","ms","residual_px","arc_fraction","gap_deg","radius"):
            row[key] = float(row[key])
        by_frame.setdefault(row["frame"],{}).setdefault((row["variant"],row["detector"]),[]).append(row)
    output = {}
    mismatches=[]
    for i,variants in by_frame.items():
        old=variants["old","circle"];defaults=variants["default","circle"]
        keys=("index","count","x","y","quality","radius")
        if [[r[k] for k in keys] for r in old]!=[[r[k] for k in keys] for r in defaults]:
            mismatches.append(i)
    if mismatches:
        raise AssertionError(f"Disabled candidate differs from baseline: {mismatches[:20]}")
    for group in sorted({r["group"] for r in rows}):
        indices = [i for i,r in enumerate(rows) if r["group"]==group]
        report = dict(frames=len(indices),variants={})
        paired_deltas, quality_gains = [],[]
        increased,decreased,unpaired = 0,0,0
        for variant,detector in (("old","circle"),("new","circle"),("default","circle"),("unchanged","cross")):
            found, timings, errors,steps,compensated = 0,[],[],[],[]
            residuals,arcs,gaps = [],[],[]
            previous = {}
            for i in indices:
                dets = by_frame[i][variant,detector]
                timings.append(dets[0]["ms"])
                candidates = [d for d in dets if d["index"]>=0]
                if detector=="circle":
                    residuals.extend(d["residual_px"] for d in candidates)
                    arcs.extend(d["arc_fraction"] for d in candidates)
                    gaps.extend(d["gap_deg"] for d in candidates)
                found += bool(candidates)
                # Only single-target sequences for temporal steps: multi-target
                # rank changes must not masquerade as centre jitter.
                if len(candidates)==1:
                    d = candidates[0];r = rows[i];seq = r.get("sequence",group)
                    if r.get("truth") is not None and r["expected"]==detector:
                        errors.append(math.dist((d["x"],d["y"]),r["truth"]))
                    last = previous.get(seq)
                    if last is not None and last[0]+1==i:
                        steps.append(math.dist((d["x"],d["y"]),last[1]))
                        if r.get("truth") is not None and last[2] is not None and r["expected"]==detector:
                            e=np.array((d["x"],d["y"]))-r["truth"]
                            compensated.append(float(np.linalg.norm(e-last[2])))
                    residual = np.array((d["x"],d["y"]))-r["truth"] if r.get("truth") is not None else None
                    previous[seq]=(i,(d["x"],d["y"]),residual)
            report["variants"][variant+"_"+detector] = dict(
                frames_detected=found,frame_detection_fraction=found/len(indices),
                median_processing_ms=stats(timings),
                synthetic_projected_center_error_px=stats(errors),
                consecutive_single_target_center_step_px=stats(steps),
                synthetic_truth_compensated_jitter_px=stats(compensated),
                ellipse_normal_residual_detector_px=stats(residuals),
                visible_arc_fraction=stats(arcs),largest_gap_degrees=stats(gaps))
        for i in indices:
            old = [d for d in by_frame[i]["old","circle"] if d["index"]>=0]
            new = [d for d in by_frame[i]["new","circle"] if d["index"]>=0]
            increased += len(new)>len(old);decreased += len(new)<len(old)
            # Greedy one-to-one matching bounded by half a ring radius.
            pairs=sorted((math.dist((a["x"],a["y"]),(b["x"],b["y"])),j,k)
                         for j,a in enumerate(old) for k,b in enumerate(new))
            used_old,used_new=set(),set()
            for delta,j,k in pairs:
                if j in used_old or k in used_new or delta>max(2,.5*min(old[j]["radius"],new[k]["radius"])):
                    continue
                used_old.add(j);used_new.add(k);paired_deltas.append(delta)
                quality_gains.append(new[k]["quality"]-old[j]["quality"])
            unpaired += len(old)+len(new)-2*len(used_old)
        report["same_frame_old_new_center_delta_px"] = stats(paired_deltas)
        report["same_frame_quality_gain"] = stats(quality_gains)
        report["candidate_count_increased_frames"] = increased
        report["candidate_count_decreased_frames"] = decreased
        report["unpaired_candidates"] = unpaired
        output[group]=report
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root",type=Path,default=Path(__file__).resolve().parents[4])
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--baseline",default="HEAD")
    parser.add_argument("--frames",type=Path,action="append",default=[])
    parser.add_argument("--limit-per-manifest",type=int,default=0)
    parser.add_argument("--repeats",type=int,default=3)
    args=parser.parse_args()
    if args.repeats<1: parser.error("--repeats must be positive")
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    rows=synthetic(args.output)
    for manifest in args.frames:
        data=json.loads(manifest.read_text())
        data=data["frames"] if isinstance(data,dict) else data
        if args.limit_per_manifest>0 and len(data)>args.limit_per_manifest:
            data=[data[i] for i in np.linspace(0,len(data)-1,args.limit_per_manifest,dtype=int)]
        for r in data:
            file=Path(r.get("file",r.get("image",r.get("source",""))))
            if not file.is_absolute(): file=manifest.parent/file
            if not file.is_file(): raise FileNotFoundError(file)
            rows.append(dict(file=str(file),group="recorded_"+manifest.parent.name,
                             sequence=str(manifest)+":"+str(r.get("group","all")),t=r.get("t"),
                             truth=None,expected="unlabelled"))
    if any("\n" in r["file"] or "\r" in r["file"] for r in rows):
        raise ValueError("Newline in image path")
    (args.output/"frames.json").write_text(json.dumps(rows,indent=2))
    exe,circle_config,cross_config=build_runner(args.root,args.output,args.baseline,args.repeats)
    files=args.output/"files.txt";files.write_text("\n".join(r["file"] for r in rows)+"\n")
    with (args.output/"geometry.tsv").open("w") as stream:
        subprocess.run([str(exe),str(files),str(args.repeats)],stdout=stream,check=True)
    with (args.output/"geometry.tsv").open() as stream:
        observations=list(csv.DictReader(stream,delimiter="\t"))
    summary=dict(baseline_revision=subprocess.check_output(["git","rev-parse",args.baseline],cwd=args.root,text=True).strip(),
                 frames=len(rows),circle_config=circle_config,cross_config=cross_config,
                 opencv_cpp=subprocess.check_output(["pkg-config","--modversion","opencv4"],text=True).strip(),
                 repeats=args.repeats,groups=summarize(rows,observations),
                 perspective_center_demonstration=perspective_demonstration(),
                 candidate_parameter="circle_quality_ordered_nms=true (offline opt-in); production default=false",
                 disabled_candidate_parity="Exact count/order/centre/quality/radius match on every frame; asserted",
                 limits=["Recorded frames have no manual target centre labels; deltas/steps are not accuracy.",
                         "Temporal steps include camera and target motion; subsampled steps are not online jitter.",
                         "Synthetic projected circle centre differs from fitted ellipse centre under perspective.",
                         "Circle timing excludes resize, diagnostics, ROS and projection; cross includes blur.",
                         "Cross production geometry is unchanged; no centre refinement candidate enabled.",
                         "Visible arc diagnostics are boundary support, not a target identity gate."])
    (args.output/"summary.json").write_text(json.dumps(summary,indent=2))
    for name,g in summary["groups"].items():
        print(json.dumps(dict(group=name,frames=g["frames"],
            old_detected=g["variants"]["old_circle"]["frames_detected"],
            new_detected=g["variants"]["new_circle"]["frames_detected"],
            cross_detected=g["variants"]["unchanged_cross"]["frames_detected"],
            center_delta=g["same_frame_old_new_center_delta_px"],quality_gain=g["same_frame_quality_gain"],
            old_ms=g["variants"]["old_circle"]["median_processing_ms"],
            new_ms=g["variants"]["new_circle"]["median_processing_ms"],
            old_error=g["variants"]["old_circle"]["synthetic_projected_center_error_px"],
            new_error=g["variants"]["new_circle"]["synthetic_projected_center_error_px"])),flush=True)
    print(args.output/"summary.json",flush=True)


if __name__ == "__main__":
    main()
