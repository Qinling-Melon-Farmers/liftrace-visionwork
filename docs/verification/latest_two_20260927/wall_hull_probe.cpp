#include <iostream>
#include "plan_env/vertical_obstacle_support.h"
int main(){
 fast_planner::MiddleHeightColumnSelector s(220,220,3.,.4,.6);
 // Three connected walls: open courtyard, not a solid obstacle.
 for(int x=10;x<=180;x++)for(double z: {0.3,1.8,3.3}){s.observe(x,10,z);s.observe(x,190,z);}
 for(int y=10;y<=190;y++)for(double z: {0.3,1.8,3.3})s.observe(10,y,z);
 s.build();
 for(int x=10;x<=180;x++){s.observeBand(x,10,1.8);s.observeBand(x,190,1.8);}
 for(int y=10;y<=190;y++)s.observeBand(10,y,1.8);
 bool interior=false;int count=0;
 s.forEachFootprint([&](int x,int y,double){++count;if(x==32&&y==100)interior=true;});
 std::cout<<"{\"synthetic_case\":\"connected_U_walls\",\"interior_without_points_filled\":"<<(interior?"true":"false")<<",\"footprint_cells\":"<<count<<"}\n";
}