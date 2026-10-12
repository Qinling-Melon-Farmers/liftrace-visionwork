"""Production H command block regression; requires C++ compilation, no ROS/flight."""
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
PROGRAM = r'''
#include "patrol_control/drop_descent_profile.h"
#include <cassert>
#include <cmath>
using namespace patrol_control;
namespace Eigen { struct Vector3d {}; }
namespace ros {
double clock=100;
struct Time { double value; double toSec()const{return value;} static Time now(){return {clock};} };
}
struct XYZ { double x=0,y=0,z=0; };
struct Pose { XYZ position; };
struct PoseStamped { Pose pose; };
struct Odom { struct { Pose pose; } pose; };
enum { Land, Other };
struct Controller {
 bool external_mission_mode_=true,external_landing_active_=true,external_landing_cancelled_=false;
 bool external_landing_alignment_complete_=true,external_landing_auto_land_requested_=false;
 bool fresh=true,controlled=true;
 int Drone_mode=Land;
 double land_height=.35;
 DropDescentConfig landing_descent_config_;
 DropDescentProfile landing_descent_profile_;
 PoseStamped mavros_point_cmd,uav_pose,external_landing_aligned_goal_;
 Odom motion_odom_;
 bool freshMotion(Eigen::Vector3d*,Eigen::Vector3d*){return fresh;}
 bool externalLandingControlReady(ros::Time){return controlled;}
 int tick(){ BLOCK
 return vertical_landing ? 3 : independent_landing ? 2 : 1;
 }
};
void close(double a,double b){assert(std::abs(a-b)<1e-9);}
int main(){
 for(int mode=1;mode<=3;++mode){
  Controller c;c.landing_descent_config_.mode=mode;
  c.landing_descent_profile_=DropDescentProfile(c.landing_descent_config_);
  c.uav_pose.pose.position={4,5,1.2};c.motion_odom_.pose.pose.position={4,5,1.2};
  c.external_landing_aligned_goal_.pose.position={1,2,.35};
  c.mavros_point_cmd=c.external_landing_aligned_goal_;
  ros::clock=100;assert(c.tick()==mode);
  if(mode==1){close(c.mavros_point_cmd.pose.position.z,.35);continue;}
  close(c.mavros_point_cmd.pose.position.z,1.2);
  for(int i=1;i<=10;++i){
   ros::clock=100+i*.05;c.uav_pose.pose.position.x+=.1;assert(c.tick()==mode);
   close(c.mavros_point_cmd.pose.position.z,1.2-.2*i*.05);
   close(c.mavros_point_cmd.pose.position.x,mode==2 ? 1 : c.uav_pose.pose.position.x);
  }
  c.fresh=false;ros::clock=100.55;c.tick();close(c.mavros_point_cmd.pose.position.z,1.1);
  c.fresh=true;ros::clock=101;c.tick();close(c.mavros_point_cmd.pose.position.z,1.1);
  for(int i=1;i<=100;++i){ros::clock=101+i*.05;c.tick();}
  close(c.mavros_point_cmd.pose.position.z,.35);
  c.external_landing_auto_land_requested_=true;c.mavros_point_cmd.pose.position.z=.37;
  ros::clock=106.05;c.tick();close(c.mavros_point_cmd.pose.position.z,.37);
  c.external_landing_cancelled_=true;assert(c.tick()==1);
 }
 Controller raw;raw.landing_descent_config_.mode=4;
 raw.uav_pose.pose.position={4,5,.75};
 raw.external_landing_aligned_goal_.pose.position={1,2,.3};
 raw.mavros_point_cmd=raw.external_landing_aligned_goal_;
 raw.fresh=false;assert(raw.tick()==2);
 close(raw.mavros_point_cmd.pose.position.z,.75);
 close(raw.mavros_point_cmd.pose.position.x,1);close(raw.mavros_point_cmd.pose.position.y,2);
 assert(!raw.landing_descent_profile_.active()); // Mode 4 never uses ramp math.
 Controller c;c.landing_descent_config_.mode=2;c.fresh=false;c.uav_pose.pose.position.z=1.2;
 assert(c.tick()==2);close(c.mavros_point_cmd.pose.position.z,1.2);
 assert(!c.landing_descent_profile_.active());
 c.external_landing_alignment_complete_=false;assert(c.tick()==1);
}
'''


class LandingDescentModesTests(unittest.TestCase):
    def test_production_h_modes_ramp_pause_and_handoff(self):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        start = source.index('    const bool independent_landing =')
        end = source.index('    // 计算当前 UAV 位置到目标点的距离', start)
        with tempfile.TemporaryDirectory(prefix='h_descent_modes_') as folder:
            cpp, binary = Path(folder)/'test.cpp', Path(folder)/'test'
            cpp.write_text(PROGRAM.replace('BLOCK', source[start:end]), encoding='utf-8')
            subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-Werror',
                            '-I',str(PACKAGE/'include'),str(cpp),'-o',str(binary)], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
