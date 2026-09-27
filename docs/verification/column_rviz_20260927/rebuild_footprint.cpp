
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include "plan_env/vertical_obstacle_support.h"
int main(int argc,char**argv){
 if(argc!=3)return 2;
 std::ifstream in(argv[1]);std::ofstream out(argv[2]);double x,y,z;
 std::vector<std::array<double,3>> p;
 fast_planner::VerticalObstacleSupport support(240,240,3.,3,.2);
 while(in>>x>>y>>z){
   p.push_back({x,y,z});
   if(x>=-.5 && x<=7.6 && y>=-4.8 && y<=4.8 && z>=.18)
     support.observe(int(std::floor((x+1)*20)),int(std::floor((y+6)*20)),z);
 }
 fast_planner::MiddleHeightColumnSelector middle(240,240,3.,.4,.6);
 for(auto a:p) {
   int ix=int(std::floor((a[0]+1)*20)),iy=int(std::floor((a[1]+6)*20));
   if(a[0]>=-.5&&a[0]<=7.6&&a[1]>=-4.8&&a[1]<=4.8&&a[2]>=.18&&support.supported(ix,iy))
     middle.observe(ix,iy,a[2]);
 }
 middle.build();
 for(auto a:p) if(a[0]>=-.5&&a[0]<=7.6&&a[1]>=-4.8&&a[1]<=4.8&&a[2]>=.18)
   middle.observeBand(int(std::floor((a[0]+1)*20)),int(std::floor((a[1]+6)*20)),a[2]);
 middle.forEachFootprint([&](int ix,int iy,double bottom){out<<ix<<" "<<iy<<" "<<bottom<<"\n";});
}
