#!/usr/bin/env python3
"""Run the current production H geometry methods offline; never creates ROS nodes."""
import argparse,json,re,shlex,subprocess
from pathlib import Path
import yaml

p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,required=True)
p.add_argument('--image',type=Path,required=True)
p.add_argument('--params',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
a=p.parse_args()
a.out.mkdir(parents=True,exist_ok=False)
src=a.root/'vision_ws/src/uav_vision/src/landing_detector_node.cpp'
header=a.root/'vision_ws/src/uav_vision/include/uav_vision/landing_detector_node.h'
text=src.read_text();decl=header.read_text()
body=text[text.index('bool LandingDetectorNode::detectLandingPad'):text.index('cv::Mat LandingDetectorNode::drawDebug')]
fields=decl[decl.index('  int blur_kernel_size_;'):decl.index('\n};')]
params=yaml.safe_load(a.params.read_text())['landing_detector']
assign=[]
effective={}
for kind,member in re.findall(r'\b(int|double|bool)\s+(\w+_);',fields):
    match=re.search(r'nh_\.param\("([^"]+)",\s*'+member+r',\s*([^;]+)\);',text)
    if not match:raise ValueError('No source default for '+member)
    key,default=match.groups()
    value=params.get(key)
    if value is None:
        raw=default.strip()
        value=(raw=='true') if kind=='bool' else float(raw)
    if kind=='bool':literal='true' if value else 'false'
    elif kind=='int':literal=str(int(value))
    else:literal=repr(float(value))
    effective[key]=value
    assign.append('d.'+member+'='+literal+';')
cpp='#include <opencv2/opencv.hpp>\n#include <uav_vision/h_stroke_detector.h>\n#include <iostream>\n#include <iomanip>\n#include <string>\nnamespace uav_vision {\nstruct LandingDetectorNode {\n'+fields+'''
bool detectLandingPad(const cv::Mat&,cv::Point2f&,float&,cv::Mat&,std::vector<std::vector<cv::Point>>&,std::vector<double>&,cv::Rect&);
bool validateHStructure(const cv::Mat&,const cv::RotatedRect&,std::vector<double>&) const;
};
'''+body+'\n}\n'+r'''
int main(int argc,char**argv) {
  cv::setNumThreads(1);
  cv::Mat image=cv::imread(argv[1]); if(image.empty()) return 2;
  for(int mode=0;mode<3;++mode) {
    uav_vision::LandingDetectorNode d;
'''+''.join(assign)+r'''
    const char* label=mode==0?"RECORDED_PARAMETERS":mode==1?"OFFLINE_RADIUS_600_ONLY":"OFFLINE_FALLBACK_TRUE_ONLY";
    if(mode==1)d.radius_max_=600;
    if(mode==2)d.enable_h_stroke_fallback_=true;
    cv::Point2f center;float radius=0;cv::Mat mask;std::vector<std::vector<cv::Point>> contours;
    std::vector<double> metrics;cv::Rect bbox;
    bool found=d.detectLandingPad(image,center,radius,mask,contours,metrics,bbox);
    std::cout<<std::setprecision(10)<<"CASE "<<label<<" found="<<found<<" center_x="<<center.x<<" center_y="<<center.y<<" radius="<<radius<<" contours="<<contours.size()<<"\n";
    if(mode!=0)continue;
    size_t index=0;
    for(const auto& contour:contours) {
      ++index;if(contour.size()<static_cast<size_t>(d.min_contour_points_))continue;
      auto e=cv::fitEllipse(contour);double w=e.size.width,h=e.size.height;
      if(w<1e-3||h<1e-3)continue;
      double ar=std::min(w,h)/std::max(w,h);
      double area=cv::contourArea(contour);
      double fill=area/std::max(CV_PI*w*h*.25,1.0),r=(w+h)/4;
      std::string reject="ACCEPT";std::vector<double> hm;
      if(ar<d.aspect_ratio_threshold_)reject="aspect";
      else if(fill<d.min_ellipse_fill_ratio_)reject="ellipse_fill";
      else if(r<d.radius_min_||r>d.radius_max_)reject="radius";
      else if(d.enable_h_structure_check_&&!d.validateHStructure(image,e,hm))reject="H_structure";
      std::cout<<"CONTOUR "<<index<<" points="<<contour.size()<<" x="<<e.center.x<<" y="<<e.center.y<<" radius="<<r<<" ar="<<ar<<" fill="<<fill<<" reject="<<reject<<"\n";
    }
  }
}
'''
source=a.out/'production_h_probe.cpp';source.write_text(cpp)
flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))
binary=a.out/'production_h_probe'
cmd=['g++','-std=c++14','-O0',str(source),'-I'+str(a.root/'vision_ws/src/uav_vision/include'),*flags,'-o',str(binary)]
subprocess.run(cmd,check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
output=subprocess.check_output([str(binary),str(a.image)],text=True)
(a.out/'probe.txt').write_text(output)
meta=dict(source=str(src),image=str(a.image),rosparams=str(a.params),effective_parameters=effective,
    method='Unmodified detectLandingPad and validateHStructure extracted from current production C++; diagnostic contour loop calls same validateHStructure.',
    runtime_effect='None: standalone OpenCV process, no ROS, no source modifications.',
    variants='Only RECORDED_PARAMETERS is the recorded configuration. Other cases are counterfactual geometry checks on one saved image, not new flight evidence.',
    opencv_cpp=subprocess.check_output(['pkg-config','--modversion','opencv4'],text=True).strip())
(a.out/'provenance.json').write_text(json.dumps(meta,indent=2))
print(output)
