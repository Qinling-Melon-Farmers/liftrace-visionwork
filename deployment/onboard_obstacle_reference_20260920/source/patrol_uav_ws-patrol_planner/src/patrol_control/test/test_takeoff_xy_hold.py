#!/usr/bin/env python3
"""Exercise the actual limiter block; takeoff XY must not follow a drifting pose."""
from pathlib import Path
import subprocess,tempfile,unittest

class TakeoffXYTest(unittest.TestCase):
    def test_actual_limiter_keeps_xy_anchor_only_for_takeoff(self):
        source=(Path(__file__).resolve().parents[1]/'src/patrol_control.cpp').read_text()
        start=source.index('    Eigen::Vector3d current_pos(uav_pose.pose.position.x')
        end=source.index('    // The distance limiter interpolates',start)
        block=source[start:end]
        code='''#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#define ROS_INFO_THROTTLE(...)
struct Vec { double x,y,z; }; struct Pose { Vec position; }; struct Msg { Pose pose; };
void run(int Drone_mode) {
  const int Takeoff=0;
  Msg uav_pose{{{-.325,-.278,.094}}},mavros_point_cmd{{{0,0,1.18}}};
  double px4_max_distance=.4; double takeoff_point[3]={0,0,1.18};
'''+block+'''
  if(Drone_mode==Takeoff) {
    assert(mavros_point_cmd.pose.position.x==0 && mavros_point_cmd.pose.position.y==0);
    assert(mavros_point_cmd.pose.position.z<.495);
  } else {
    assert(mavros_point_cmd.pose.position.x<-.1 && mavros_point_cmd.pose.position.y<-.1);
    Eigen::Vector3d d(mavros_point_cmd.pose.position.x+.325,mavros_point_cmd.pose.position.y+.278,mavros_point_cmd.pose.position.z-.094);
    assert(std::abs(d.norm()-.4)<1e-10);
  }
}
int main(){run(0);run(1);}
'''
        with tempfile.TemporaryDirectory() as folder:
            cpp=Path(folder)/'test.cpp';cpp.write_text(code);exe=Path(folder)/'test'
            subprocess.run(['g++','-std=c++14','-fsanitize=undefined','-I/usr/include/eigen3',str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__':unittest.main()
