#include <plan_manage/trajectory_progress.h>
#include <iostream>
int main(){
  auto position=[](double t)->Eigen::Vector3d{return {t,0.,0.};};
  const Eigen::Vector3d measured(0.,.25,0.);
  double progress=0.;bool hold=false;bool replan=false;
  for(int i=0;i<1000;++i){
    progress=fast_planner::projectProgress(position,measured,progress,10.);
    double next=fast_planner::boundedLookahead(position,measured,progress,10.,.15);
    hold=hold || (position(next)-measured).norm()>.15+.05;
    replan=(position(progress)-measured).norm()>.45 || progress>=10.-.03;
    if(!hold || replan || progress!=0.) return 1;
  }
  std::cout << "REPRODUCED: lead=0.15 error=0.25 hold=true replan=false progress=0 for 1000 ticks\n";
}
