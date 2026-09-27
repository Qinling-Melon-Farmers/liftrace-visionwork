
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include "plan_env/vertical_obstacle_support.h"
// Frozen source snapshot with configuration supplied by its rosparams.yaml.
// Never feed above-map returns to the selector: SDFMap::cloudCallback doesn't.
int main(int argc,char**argv){
 if(argc!=4)return 2;
 std::ifstream in(argv[1]),cfg(argv[3]);std::ofstream out(argv[2]);
 int nx,ny,min_points;
 double res,ox,oy,oz,sx,sy,sz,support_radius,min_span;
 double minx,maxx,miny,maxy,minz,lo,hi,max_hull,fill;
 if(!(cfg>>nx>>ny>>res>>ox>>oy>>oz>>sx>>sy>>sz>>support_radius>>min_points>>min_span
      >>minx>>maxx>>miny>>maxy>>minz>>lo>>hi>>max_hull>>fill))return 3;
 const auto candidate=[&](const std::array<double,3>& a){
   return a[0]>=minx&&a[0]<=maxx&&a[1]>=miny&&a[1]<=maxy&&a[2]>=minz;
 };
 std::vector<std::array<double,3>> p;
 fast_planner::VerticalObstacleSupport support(nx,ny,support_radius/res,min_points,min_span);
 float x,y,z;
 while(in>>x>>y>>z){
   if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z) ||
      x<ox+1e-4||x>ox+sx-1e-4||y<oy+1e-4||y>oy+sy-1e-4||
      z<oz+1e-4||z>oz+sz-1e-4)continue;
   p.push_back({x,y,z});
   if(candidate(p.back()))support.observe(int(floor((x-ox)/res)),int(floor((y-oy)/res)),z);
 }
#ifdef OLD_SELECTOR
 fast_planner::MiddleHeightColumnSelector middle(nx,ny,support_radius/res,lo,hi);
#else
 fast_planner::MiddleHeightColumnSelector middle(nx,ny,support_radius/res,lo,hi,max_hull/res,fill/res);
#endif
 for(auto a:p){
   int ix=int(floor((a[0]-ox)/res)),iy=int(floor((a[1]-oy)/res));
   if(candidate(a)&&support.supported(ix,iy))middle.observe(ix,iy,a[2]);
 }
 middle.build();
 for(auto a:p)if(candidate(a))
   middle.observeBand(int(floor((a[0]-ox)/res)),int(floor((a[1]-oy)/res)),a[2]);
 middle.forEachFootprint([&](int ix,int iy,double bottom){out<<ix<<" "<<iy<<" "<<bottom<<"\n";});
}
