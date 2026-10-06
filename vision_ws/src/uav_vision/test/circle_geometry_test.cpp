// Standalone offline regression: no ROS master or generated messages needed.
#include <uav_vision/circle_geometry.h>
#include <iostream>
#include <stdexcept>

namespace {
struct Candidate { cv::RotatedRect ellipse; double quality; };
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
Candidate circle(float x, float y, float radius, double quality) {
  return {cv::RotatedRect({x,y},{2*radius,2*radius},0),quality};
}
}

int main() {
  using namespace uav_vision;
  int tests = 0;
  {
    std::vector<Candidate> candidates{circle(104,100,40,.71),circle(100,100,40,.89)};
    qualityOrderedCircleNms(candidates,.45,12);
    require(candidates.size()==1 && candidates[0].quality==.89,
            "low-quality contour suppressed the high-quality one"); ++tests;
  }
  {
    std::vector<Candidate> input{circle(100,100,40,.89),circle(103,101,40,.71),
                                 circle(250,100,30,.85),circle(252,101,30,.73)};
    for (int i=0;i<24;++i) {
      auto candidates=input; qualityOrderedCircleNms(candidates,.45,12);
      require(candidates.size()==2 && candidates[0].quality==.89 &&
              candidates[1].quality==.85,"permutation changed suppression");
      std::next_permutation(input.begin(),input.end(),[](const Candidate& a,const Candidate& b){
        return a.quality < b.quality;
      });
    } ++tests;
  }
  {
    std::vector<Candidate> candidates{circle(101,100,40,.8),circle(100,100,40,.8)};
    qualityOrderedCircleNms(candidates,.45,12);
    require(candidates.size()==1 && candidates[0].ellipse.center.x==100,
            "equal-score tie depends on contour order"); ++tests;
  }
  {
    std::vector<Candidate> candidates{circle(100,100,30,.81),circle(250,100,30,.83)};
    qualityOrderedCircleNms(candidates,.45,1);
    require(candidates.size()==1 && candidates[0].quality==.83,"limit applied before ranking");
    qualityOrderedCircleNms(candidates,.45,0); require(candidates.empty(),"zero cap");
    qualityOrderedCircleNms(candidates,.45,-1); require(candidates.empty(),"negative cap"); ++tests;
  }
  {
    cv::Mat mask=cv::Mat::zeros(300,300,CV_8UC1);
    cv::ellipse(mask,{150,150},{80,65},25,0,360,255,2);
    std::vector<std::vector<cv::Point>> simple,dense;
    cv::findContours(mask.clone(),simple,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(mask.clone(),dense,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    const auto fit=cv::fitEllipse(dense[0]);
    const auto a=ellipseSupport(simple[0],fit),b=ellipseSupport(dense[0],fit);
    require(a.visible_arc_fraction>.95 && a.normal_rms_px<1.0,"clean ellipse diagnostics");
    require(std::abs(a.normal_rms_px-b.normal_rms_px)<1e-8 &&
            a.visible_arc_fraction==b.visible_arc_fraction,"diagnostics depend on SIMPLE vertices");
    ++tests;
  }
  {
    std::vector<cv::Point> full,partial;
    cv::ellipse2Poly({150,150},{80,65},25,0,360,2,full);
    cv::ellipse2Poly({150,150},{80,65},25,40,280,2,partial);
    const cv::RotatedRect reference({150,150},{160,130},25);
    auto a=ellipseSupport(full,reference),b=ellipseSupport(partial,reference);
    require(a.visible_arc_fraction>.95 && b.visible_arc_fraction<.8 &&
            b.largest_gap_degrees>=80 && b.normal_rms_px>a.normal_rms_px+2,
            "occlusion chord counted as visible arc"); ++tests;
  }
  {
    const auto empty=ellipseSupport({},cv::RotatedRect());
    require(empty.visible_arc_fraction==0 && empty.largest_gap_degrees==360,"empty support");
    std::vector<cv::Point> rectangle{{80,80},{220,80},{220,220},{80,220}};
    auto s=ellipseSupport(rectangle,cv::RotatedRect({150,150},{140,140},0));
    require(s.normal_rms_px>10 && s.visible_arc_fraction<.5,"rectangle resembles ellipse support");
    ++tests;
  }
  {
    std::vector<cv::Point2f> circle_points;
    for (int i=0;i<720;++i) {
      double angle=i*2*CV_PI/720;
      circle_points.emplace_back(75*std::cos(angle),75*std::sin(angle));
    }
    const cv::Mat transform=(cv::Mat_<double>(3,3)<<1,0,0,0,1,0,.0008,0,1);
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(circle_points,projected,transform);
    const auto ellipse=cv::fitEllipse(projected);
    const double expected=-.0008*75*75/(1-.0008*.0008*75*75);
    require(std::abs(ellipse.center.x-expected)<.001 &&
            cv::norm(ellipse.center)>4.5,"ellipse centre mistaken for projected circle centre");
    ++tests;
  }
  std::cout << tests << " circle geometry tests PASS\n";
}
