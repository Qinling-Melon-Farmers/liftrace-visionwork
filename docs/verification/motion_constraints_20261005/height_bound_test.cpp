#include <plan_env/reference_height.h>
#include <cassert>
int main(){
 fast_planner::ReferenceHeight h;h.enabled=true;h.max_z=.78;
 assert(h.accepts(.68));assert(!h.accepts(.80));
 assert(!h.polynomial(.68,.8,-.8,0,1)); // endpoints below, mid-primitive peak above
 assert(h.polynomial(.68,.1,0,0,1));
 Eigen::MatrixXd controls(4,3);controls.setZero();controls.col(2).setConstant(.68);
 assert(h.controls(controls));controls(1,2)=.9;assert(!h.controls(controls));
 h.valid=false;assert(!h.accepts(.6));assert(!h.controls(controls));
}
