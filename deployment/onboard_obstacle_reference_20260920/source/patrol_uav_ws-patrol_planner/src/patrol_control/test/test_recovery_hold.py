#!/usr/bin/env python3
"""Compile the production hold branches and exercise delayed handoff/drift."""
from pathlib import Path
import subprocess,tempfile,unittest

class RecoveryHoldTest(unittest.TestCase):
    def test_old_trajectory_cannot_replace_recovery_hold(self):
        src=(Path(__file__).resolve().parents[1]/'src/patrol_control.cpp').read_text()
        start=src.index('                if (external_waiting_for_motion_) {')
        end=src.index('                ROS_INFO_THROTTLE(5, "[PatrolControl] Forwarding external planner trajectory")',start)
        branch=src[start:end]
        code='''#include <cassert>
#define ROS_WARN_THROTTLE(...)
struct P { double x,y,z; }; struct Pose { P position; }; struct Msg {Pose pose;};
bool hasValidExternalPlannerCommand(){return true;}
int main(){
bool external_waiting_for_motion_=true;
Msg patrol_cmd{{{1,2,1.2}}},mavros_point_cmd{},planner_cmd{{{9,9,.1}}},last_mavros_point_cmd{};
double external_planner_max_command_z_=2.08;
for(int i=0;i<100;i++){
'''+branch+'''
assert(mavros_point_cmd.pose.position.z==1.2);
assert(mavros_point_cmd.pose.position.x==1);
planner_cmd.pose.position.z-=.001;
}
external_waiting_for_motion_=false;
'''+branch+'''
assert(mavros_point_cmd.pose.position.x==9);
}
'''
        self.compile_run(code)

    def test_tracking_loss_latches_position_until_new_trajectory(self):
        root=Path(__file__).resolve().parents[2]
        src=(root/'Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp').read_text()
        start=src.index('  if (trajectory_interrupted_) {',src.index('void cmdCallback'))
        end=src.index('  double interpolated_yaw',start)
        branch=src[start:end]
        code='''#include <Eigen/Dense>
#include <cassert>
int main(){
bool trajectory_interrupted_=false,tracking_hold_active_=false;
Eigen::Vector3d tracking_hold_position_,stop_position_,odom_pos_(1,2,1.0),pos(9,9,9),vel,acc;
double target_dist=.4;
'''+branch+'''
assert(tracking_hold_active_);
odom_pos_.z()=.2;
'''+branch+'''
assert(pos.z()==1.0);
tracking_hold_active_=false;pos=odom_pos_;
'''+branch+'''
assert(!tracking_hold_active_ && pos.z()==.2);
}
'''
        self.compile_run(code)

    def compile_run(self,code):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'test.cpp';p.write_text(code);exe=Path(d)/'test'
            subprocess.run(['g++','-std=c++14','-fsanitize=undefined','-I/usr/include/eigen3',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__':unittest.main()
