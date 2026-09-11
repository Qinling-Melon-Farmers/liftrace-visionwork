// Offline fixture runner for the SAME traversal used by the planner service.
// No ROS initialization, map publisher, physics, planner search or flight outputs.
#include <plan_env/local_segment_probe.h>
#include <array>
#include <iostream>
#include <set>
using namespace fast_planner;
int main() {
  LocalCloudView map;ProbeLimits limits;double now;size_t count;
  map.valid=true;map.frame="fixture";map.revision=1;
  if (!(std::cin>>map.resolution)) return 2;
  for(auto* p:{&map.origin,&map.lower,&map.upper}) for(int i=0;i<3;++i) std::cin>>(*p)[i];
  std::cin>>map.stamp>>now>>count;
  if(count>100000) return 2;
  std::set<std::array<int,3>> occupied;
  for(size_t i=0;i<count;++i) {
    std::array<int,3> p;std::cin>>p[0]>>p[1]>>p[2];occupied.insert(p);
  }
  std::cin>>count;if(!std::cin || count>32) return 2;
  std::vector<Segment> segments(count);
  for(auto& s:segments) for(auto* p:{&s.start,&s.end}) for(int i=0;i<3;++i) std::cin>>(*p)[i];
  if(!std::cin) return 2;
  const auto r=probeSegments(map,segments,limits,now,[&](const Eigen::Vector3i& p) {
    return occupied.count({p.x(),p.y(),p.z()}) ? 1 : 0;
  });
  std::cout<<r.accepted<<" "<<r.reason<<" "<<r.checked_voxels<<"\n";
  for(auto status:r.status) std::cout<<int(status)<<" ";
  std::cout<<"\n";
  return 0;
}
