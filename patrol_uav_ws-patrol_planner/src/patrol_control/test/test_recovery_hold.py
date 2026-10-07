#!/usr/bin/env python3
"""Compile the production hold branches and exercise delayed handoff/drift."""
from pathlib import Path
import subprocess,tempfile,unittest

class RecoveryHoldTest(unittest.TestCase):
    def test_old_trajectory_cannot_replace_recovery_hold(self):
        from test_height_hold import compile_run, production_program
        compile_run(production_program(r'''
LLController c;c.external_waiting_for_motion_=true;
c.patrol_cmd.pose.position.x=1;c.patrol_cmd.pose.position.y=2;c.patrol_cmd.pose.position.z=1.2;
at(c,1,2,1.2);
for(int i=0;i<100;++i){
    send(c,9,9,.1);c.tick();expect(c,1,2,1.2);
}
c.external_waiting_for_motion_=false;
send(c,1.1,2,1.2);c.tick();expect(c,1.1,2,1.2);
send(c,1.1,2,2.5);c.tick();expect(c,1,2,1.2);
assert(!c.have_planner_cmd && c.height_replan_pub_.calls==1);
'''))

    def test_tracking_loss_latches_position_until_new_trajectory(self):
        root=Path(__file__).resolve().parents[2]
        src=(root/'Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp').read_text()
        start=src.index('  if (trajectory_interrupted_) {',src.index('void cmdCallback'))
        end=src.index('  if (progress_enabled)',start)
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
