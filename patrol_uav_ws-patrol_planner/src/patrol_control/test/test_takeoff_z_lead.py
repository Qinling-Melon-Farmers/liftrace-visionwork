"""Compile and exercise the production limiter and validator without ROS nodes."""
from pathlib import Path
import subprocess
import tempfile
import unittest

LEGACY_BLOCK = r'''    Eigen::Vector3d current_pos(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
    Eigen::Vector3d target_pos(mavros_point_cmd.pose.position.x, mavros_point_cmd.pose.position.y, mavros_point_cmd.pose.position.z);
    double distance_to_target = (target_pos - current_pos).norm();
    // 如果距离超过 px4_max_distance，则进行插值
    if (distance_to_target > px4_max_distance &&
        !(external_mission_mode_ && Drone_mode == Run_point &&
          external_planner_height_hold_active_ && !external_planner_command_accepted)) {
        Eigen::Vector3d direction = (target_pos - current_pos).normalized();
        Eigen::Vector3d new_pos = current_pos + direction * px4_max_distance;
        // 更新目标点为插值后的点位
        mavros_point_cmd.pose.position.x = new_pos.x();
        mavros_point_cmd.pose.position.y = new_pos.y();
        mavros_point_cmd.pose.position.z = new_pos.z();
        ROS_INFO_THROTTLE(2, "Target point adjusted to max distance limit.");
    }

    // Vertical takeoff must hold the launch XY independently of the Z slew.
    // A 3-D lookahead otherwise moves XY toward the drifting measured pose.
    if (Drone_mode == Takeoff) {
        mavros_point_cmd.pose.position.x = takeoff_point[0];
        mavros_point_cmd.pose.position.y = takeoff_point[1];
    }

    // The distance limiter interpolates from the measured pose, which may
    // already be above the ceiling. Reject its whole result, not just its Z.
    // A latched hold can itself be above the ceiling; this is a stop request,
    // not a claim that measured height or the FC local frame is safe/reset-free.
    if (external_mission_mode_ && mavros_point_cmd.pose.position.z >
        external_planner_max_command_z_ + external_planner_height_epsilon_ && !hasValidNavigationRecovery()) {
        if (Drone_mode==Run_point) {
            holdExternalPlannerHeight("final", mavros_point_cmd.pose.position.z);
        } else {
            // Existing non-planner ceiling safeguard, e.g. recovery/takeoff.
            mavros_point_cmd.pose.position.z=external_planner_max_command_z_;
        }
    } else if (external_mission_mode_ && Drone_mode == Run_point &&
               external_planner_command_accepted) {
        external_planner_height_hold_active_ = false;
    }

'''

class TakeoffZLead(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source=(Path(__file__).resolve().parents[1]/'src/patrol_control.cpp').read_text()
        start=source.index('    Eigen::Vector3d current_pos(uav_pose.pose.position.x')
        end=source.index('    // 提取当前 yaw',start)
        helper_start=source.index('double validatedTakeoffZLead(')
        helper_end=source.index('\n}',helper_start)+2
        cls.folder=tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.folder.cleanup)
        cls.exe=Path(cls.folder.name)/'test'
        code=r'''
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <string>
#define ROS_INFO_THROTTLE(...)
'''+source[helper_start:helper_end]+r'''
enum {Takeoff, Run_point, Aligning, Land, Hover};
struct Vec { double x,y,z; }; struct Pose { Vec position; }; struct Msg { Pose pose; };
struct Limiter {
  int Drone_mode=Takeoff;
  bool external_mission_mode_=true, external_planner_height_hold_active_=false;
  bool external_planner_command_accepted=false, recovery=false;
  double px4_max_distance=.4, takeoff_z_lead_m_=0.;
  double external_planner_max_command_z_=3.5, external_planner_height_epsilon_=1e-9;
  double takeoff_point[3]={0.,0.,1.18};
  Msg uav_pose{{{-.325,-.278,.094}}},mavros_point_cmd{{{0.,0.,1.18}}};
  bool hasValidNavigationRecovery(){return recovery;}
  void holdExternalPlannerHeight(const char*,double){mavros_point_cmd.pose.position={.1,.2,.3};external_planner_height_hold_active_=true;}
  void production(){
'''+source[start:end]+r'''
  }
  void legacy(){
'''+LEGACY_BLOCK+r'''
  }
};
void close(double a,double b){assert(std::abs(a-b)<1e-12);}
void same(const Limiter&a,const Limiter&b){
 close(a.mavros_point_cmd.pose.position.x,b.mavros_point_cmd.pose.position.x);
 close(a.mavros_point_cmd.pose.position.y,b.mavros_point_cmd.pose.position.y);
 close(a.mavros_point_cmd.pose.position.z,b.mavros_point_cmd.pose.position.z);
 assert(a.external_planner_height_hold_active_==b.external_planner_height_hold_active_);
}
int main(int argc,char**argv){
 assert(argc==2);int test=std::stoi(argv[1]),cases=0;
 double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
 if(test==0){
  for(double invalid:{0.,-.1,.39,.600001,1.,nan,inf,-inf}){close(validatedTakeoffZLead(invalid),0.);cases++;}
  for(double valid:{.4,.5,.6}){close(validatedTakeoffZLead(valid),valid);cases++;}
 }else if(test==1){
  for(double lead:{0.,-.1,.39,.600001,1.,nan,inf})for(double z:{.094,1.15,1.3,3.9}){
   Limiter a;a.takeoff_z_lead_m_=lead;a.uav_pose.pose.position.z=z;Limiter old=a;
   a.production();old.legacy();same(a,old);cases++;
  }
 }else if(test==2){
  for(double lead:{.4,.5,.6})for(double z:{-.2,.094,.9,1.15,1.3,3.9}){
   Limiter a;a.takeoff_z_lead_m_=lead;a.uav_pose.pose.position.z=z;a.production();
   close(a.mavros_point_cmd.pose.position.x,0.);close(a.mavros_point_cmd.pose.position.y,0.);
   close(a.mavros_point_cmd.pose.position.z,std::max(z-lead,std::min(1.18,z+lead)));cases++;
  }
  Limiter pure,old;pure.takeoff_z_lead_m_=.4;pure.production();old.legacy();
  assert(pure.mavros_point_cmd.pose.position.z>old.mavros_point_cmd.pose.position.z+.01);cases++;
 }else if(test==3){
  for(double lead:{.4,.6})for(bool recovery:{false,true}){
   Limiter a;a.takeoff_z_lead_m_=lead;a.recovery=recovery;
   a.external_planner_max_command_z_=.4;a.production();
   close(a.mavros_point_cmd.pose.position.z,recovery?.094+lead:.4);
   close(a.mavros_point_cmd.pose.position.x,0.);close(a.mavros_point_cmd.pose.position.y,0.);cases++;
  }
 }else if(test==4){
  for(int mode:{Run_point,Aligning,Land,Hover})for(double lead:{0.,.4,.6,-.1,nan})
   for(bool external:{false,true})for(bool held:{false,true})for(bool accepted:{false,true})
    for(double z:{.094,3.9}){
     Limiter a;a.Drone_mode=mode;a.takeoff_z_lead_m_=lead;a.external_mission_mode_=external;
     a.external_planner_height_hold_active_=held;a.external_planner_command_accepted=accepted;
     a.uav_pose.pose.position.z=z;Limiter old=a;
     a.production();old.legacy();same(a,old);cases++;
    }
 }else if(test==5){
  for(double lead:{.4,.6}){
   Limiter a;a.takeoff_z_lead_m_=lead;a.uav_pose.pose.position.z=1.15;a.production();
   close(a.mavros_point_cmd.pose.position.z,1.18);
   a.Drone_mode=Run_point;a.mavros_point_cmd.pose.position={2.,.5,1.18};Limiter old=a;
   a.production();old.legacy();same(a,old);
   Eigen::Vector3d delta(a.mavros_point_cmd.pose.position.x-a.uav_pose.pose.position.x,
     a.mavros_point_cmd.pose.position.y-a.uav_pose.pose.position.y,
     a.mavros_point_cmd.pose.position.z-a.uav_pose.pose.position.z);
   close(delta.norm(),.4);assert(a.mavros_point_cmd.pose.position.x!=0.);cases++;
  }
 }else{assert(false);}
 std::cout<<"section "<<test<<": "<<cases<<" numerical cases PASS\n";
}
'''
        cpp=Path(cls.folder.name)/'test.cpp';cpp.write_text(code)
        subprocess.run(['g++','-std=c++14','-fsanitize=undefined','-fno-sanitize-recover=undefined','-I/usr/include/eigen3',str(cpp),'-o',str(cls.exe)],check=True)

    def run_section(self,index):subprocess.run([str(self.exe),str(index)],check=True)
    def test_parameter_range(self):self.run_section(0)
    def test_disabled_and_invalid_keep_legacy_3d(self):self.run_section(1)
    def test_enabled_pure_z_near_goal_and_xy_anchor(self):self.run_section(2)
    def test_original_final_ceiling_and_recovery_exception(self):self.run_section(3)
    def test_all_other_modes_match_pre_patch_limiter(self):self.run_section(4)
    def test_takeoff_to_run_point_has_no_latched_z_or_xy(self):self.run_section(5)

if __name__=='__main__':unittest.main(verbosity=2)
