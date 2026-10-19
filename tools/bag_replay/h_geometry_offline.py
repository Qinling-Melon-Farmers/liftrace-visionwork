#!/usr/bin/env python3
"""Run extracted production H geometry on exported frames; no ROS master/flight.

This checks actual C++ image methods and defaults, not ROS mode/TF/flight behavior.
Use a frames.json list with file paths (absolute or relative to the manifest).
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--frames", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument("--segmentation", choices=["legacy_hsv", "grayscale_otsu"], required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    source = (a.root / "vision_ws/src/uav_vision/src/landing_detector_node.cpp").read_text()
    params = re.findall(r'nh_\.param(?:<std::string>)?\("(landing_[^"]+)",\s*(\w+),\s*([^\)]+)\);', source)
    fields = []
    for _, name, value in params:
        kind = "std::string" if value.startswith('"') else ("bool" if value in ("true", "false") else ("double" if "." in value else "int"))
        fields.append(kind + " " + name + " = " + value + ";")
    methods = source[source.index("bool LandingDetectorNode::detectLandingPad"):source.index("cv::Mat LandingDetectorNode::drawDebug")]
    cpp = r'''#include <opencv2/opencv.hpp>
#include <uav_vision/h_stroke_detector.h>
#include <uav_vision/landing_h_mask.h>
#include <iostream>
#include <fstream>
#include <chrono>
namespace uav_vision {
class LandingDetectorNode {public:
FIELDS
bool detectLandingPad(const cv::Mat&,cv::Point2f&,float&,cv::Mat&,std::vector<std::vector<cv::Point>>&,std::vector<double>&,cv::Rect&);
bool validateHStructure(const cv::Mat&,const cv::RotatedRect&,std::vector<double>&) const;
};
METHODS
}
int main(int argc,char**argv){
 cv::setNumThreads(1);uav_vision::LandingDetectorNode n;
 n.enable_h_stroke_fallback_=true;n.h_segmentation_=argv[2];
 std::ifstream files(argv[1]);std::string file;
 std::cout<<"file,found,x,y,radius,quality,contour_points,processing_ms\n";
 while(std::getline(files,file)) {
  auto im=cv::imread(file);if(im.empty()) return 2;
  cv::Mat mask;cv::Point2f center;float radius=0;cv::Rect box;
  std::vector<std::vector<cv::Point>> contours;std::vector<double> quality;
  auto start=std::chrono::steady_clock::now();
  bool found=n.detectLandingPad(im,center,radius,mask,contours,quality,box);
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  std::cout<<file<<","<<found<<","<<(found?center.x:0)<<","<<(found?center.y:0)<<","<<radius<<","<<(found?quality[3]:0)<<","<<(found?quality[4]:0)<<","<<ms<<"\n";
 }
}
'''.replace("FIELDS", "\n".join(fields)).replace("METHODS", methods)
    generated = a.output / "production_geometry.cpp"
    generated.write_text(cpp)
    exe = a.output / "production_geometry"
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "opencv4"], text=True))
    subprocess.run(["g++", "-O2", "-std=c++14", str(generated), "-I" + str(a.root / "vision_ws/src/uav_vision/include"), *flags, "-o", str(exe)], check=True)
    frames = json.loads(a.frames.read_text())
    paths = [Path(row["file"]) for row in frames]
    paths = [v if v.is_absolute() else a.frames.parent/v for v in paths]
    if any(any(c in str(v) for c in "\n\r,") for v in paths):
        raise ValueError("Frame paths cannot contain CSV/newline delimiters")
    file_list = a.output / "files.txt"
    file_list.write_text("\n".join(map(str, paths)) + "\n")
    with (a.output / "geometry.csv").open("w") as stream:
        subprocess.run([str(exe), str(file_list), a.segmentation], stdout=stream, check=True)
    print(a.output / "geometry.csv")


if __name__ == "__main__":
    main()
