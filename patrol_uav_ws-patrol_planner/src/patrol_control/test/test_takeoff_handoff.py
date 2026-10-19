"""Exercise production takeoff, mission-wait and planner callbacks continuously."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class TakeoffHandoff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (Path(__file__).resolve().parents[1] / 'src/patrol_control.cpp').read_text()

        def between(start, end, after=0):
            begin = source.index(start, after)
            return source[begin:source.index(end, begin)]

        validator = between('double validatedTakeoffZLead(', '\nbool loadSlotOffsets(')
        position = between('void LLController::positionCallback(', '\nvoid LLController::publishControlReady(')
        valid = between('bool LLController::hasValidExternalPlannerCommand()', '\nvoid LLController::holdExternalPlannerHeight(')
        navigation = between('void LLController::missionCommandCallback(', '        case patrol_control::MissionCommand::ALIGN:')
        navigation += '        default: break;\n    }\n}\n'
        takeoff = between('        case Takeoff:  // Takeoff', '        case Run_point:')
        running = between('        case Run_point: {', '            if (current_task_type == MAIN_MISSION)')
        running += '            break;\n        }\n'
        limiter = between('    Eigen::Vector3d current_pos(uav_pose.pose.position.x', '    // 提取当前 yaw')
        yaw = between('    double current_yaw = tf::getYaw(uav_pose.pose.orientation);',
                      '    // The target center can lie closer', source.index('void LLController::cmdCallback('))
        publish = between('    mavros_point_cmd.header.stamp = ros::Time::now();', '    // 判断是否已经降落')
        cls.folder = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.folder.cleanup)
        cls.exe = Path(cls.folder.name) / 'test'
        code = r'''
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#define ROS_INFO(...)
#define ROS_WARN_THROTTLE(...)
#define ROS_INFO_THROTTLE(...)
#define ROS_ERROR(...)
namespace ros {
struct Time {double value=100.;double toSec()const{return value;}static Time now(){return {};}};
Time operator-(Time a,Time b){return {a.value-b.value};}
}
namespace geometry_msgs {
struct Point {double x=0.,y=0.,z=0.;};
struct Quaternion {double x=0.,y=0.,z=0.,w=1.;};
struct Pose {Point position;Quaternion orientation;};
struct Header {ros::Time stamp;std::string frame_id="camera_init";};
struct PoseStamped {Header header;Pose pose;};
}
namespace tf {
geometry_msgs::Quaternion createQuaternionMsgFromYaw(double y){return {0.,0.,sin(y/2.),cos(y/2.)};}
double getYaw(const geometry_msgs::Quaternion&q){return atan2(2.*q.w*q.z,1.-2.*q.z*q.z);}
}
namespace patrol_control {
struct MissionCommand {
using ConstPtr=std::shared_ptr<const MissionCommand>;
enum {SEARCH,APPROACH,RESUME,RETURN_HOME,ALIGN};
int command=SEARCH,target_id=0;std::string target_class;
};
}
enum {Takeoff,Run_point,Aligning,Land,Hover,Nothing_point,MAIN_MISSION};
struct Flag {bool data=false;};struct Waypoint {double yaw=.7;};
struct Publisher {void publish(const geometry_msgs::PoseStamped&){};};
bool isQuaternionNormalized(const geometry_msgs::Quaternion&q,double tolerance){
return std::abs(std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1.)<tolerance;}
'''+validator+r'''
struct LLController {
// Inactive H transport dependencies; this fixture exercises takeoff only.
bool externalLandingOffboardDescentReady() const { return false; }
void publishExternalLandingOffboardSetpoint() {}
int Drone_mode=Takeoff,Point_mode=Nothing_point,current_task_type=MAIN_MISSION;
bool flag_takeoff_done=false,external_mission_mode_=true,debug=false;
bool have_planner_cmd=false,external_waiting_for_motion_=false;
bool external_planner_height_hold_active_=false,navigation_recovery_active_=false;
bool external_landing_auto_land_requested_=false,align_ok=false,ready=false;
double takeoff_threshould=.3,px4_max_distance=.25,takeoff_z_lead_m_=.6;
double external_planner_max_command_z_=3.5,external_planner_height_epsilon_=1e-9;
double external_planner_cmd_timeout_=.5,external_planner_start_max_distance_=1.2,max_yaw_change=1.;
Eigen::Vector3f takeoff_point{0.f,0.f,1.18778014f},uav_newest_position;
Waypoint waypoint_list[1];double adjust_target_position[3];
geometry_msgs::PoseStamped uav_pose,patrol_cmd,mavros_point_cmd,last_mavros_point_cmd;
geometry_msgs::PoseStamped planner_cmd,external_planner_height_hold_,waypoint_mark_point;
ros::Time latest_planner_cmd_time_;
Flag detect_enable_msg_temp;int detect_control_pub_=0,landing_detect_control_pub_=0;
Publisher mavros_point_cmd_pub;
static Eigen::Vector3f toEigen(const geometry_msgs::Point&p){return {float(p.x),float(p.y),float(p.z)};}
void NextPoint(){}void patrol(){}void publishControlReady(bool v){ready=v;}
void publishLegacyVisionControl(int,Flag){}void cancelDropAction(){}
void clearExternalLandingState(bool){}void resetDetectionState(){}
bool hasValidNavigationRecovery()const{return false;}
void holdExternalPlannerHeight(const char*,double){
 if(!external_planner_height_hold_active_)external_planner_height_hold_=uav_pose;
 external_planner_height_hold_active_=true;mavros_point_cmd=external_planner_height_hold_;have_planner_cmd=false;
}
void positionCallback(const geometry_msgs::PoseStamped&);
bool hasValidExternalPlannerCommand()const;
void missionCommandCallback(const patrol_control::MissionCommand::ConstPtr&);
Publisher drop_vertical_cmd_pub_,landing_vertical_cmd_pub_;
std::string external_landing_frame_="camera_init";
void tick();
};
'''+position+valid+navigation
        code += 'void LLController::tick(){bool external_planner_command_accepted=false;\nconst bool independent_drop=false,independent_landing=false,zero_velocity_drop=false,vertical_landing=false;\nswitch(Drone_mode){\n'
        code += takeoff+running+'default:break;\n}\n'+limiter+yaw+publish+'\n}\n'
        code += r'''
void close(double a,double b){assert(std::abs(a-b)<1e-9);}
void atGoal(const LLController&a,const geometry_msgs::PoseStamped&m){
close(m.pose.position.x,a.takeoff_point[0]);close(m.pose.position.y,a.takeoff_point[1]);close(m.pose.position.z,a.takeoff_point[2]);}
void expectedHoldOutput(const LLController&a){
 Eigen::Vector3d current(a.uav_pose.pose.position.x,a.uav_pose.pose.position.y,a.uav_pose.pose.position.z);
 Eigen::Vector3d goal=a.takeoff_point.cast<double>();
 const double distance=(goal-current).norm();
 Eigen::Vector3d expected=goal;
 if(distance>a.px4_max_distance)expected=current+(goal-current)*a.px4_max_distance/distance;
 close(a.mavros_point_cmd.pose.position.x,expected.x());
 close(a.mavros_point_cmd.pose.position.y,expected.y());
 close(a.mavros_point_cmd.pose.position.z,expected.z());
}
void pose(LLController&a,double x,double y,double z){
auto p=a.uav_pose;p.pose.position={x,y,z};a.positionCallback(p);}
void authorize(LLController&a,int command=patrol_control::MissionCommand::SEARCH){
auto p=std::make_shared<patrol_control::MissionCommand>();p->command=command;a.missionCommandCallback(p);}
void complete(LLController&a){
pose(a,0.,0.,.1);a.tick();assert(!a.flag_takeoff_done);
assert(a.last_mavros_point_cmd.pose.position.z<a.takeoff_point[2]-.2);
pose(a,.02,.06,.90);assert(a.flag_takeoff_done&&a.ready&&a.Drone_mode==Run_point);
assert(a.external_waiting_for_motion_);atGoal(a,a.patrol_cmd);atGoal(a,a.last_mavros_point_cmd);
close(tf::getYaw(a.patrol_cmd.pose.orientation),.7);assert(!a.external_planner_height_hold_active_);
}
int main(int argc,char**argv){
assert(argc==2);int section=std::stoi(argv[1]);
if(section==0){
 for(double lead:{0.,.4,.6})for(double limit:{.2,.25,.4}){
  LLController a;a.takeoff_z_lead_m_=lead;a.px4_max_distance=limit;complete(a);
  a.tick();atGoal(a,a.patrol_cmd);expectedHoldOutput(a);assert(a.external_waiting_for_motion_);
  Eigen::Vector3d delta(a.mavros_point_cmd.pose.position.x-.02,a.mavros_point_cmd.pose.position.y-.06,a.mavros_point_cmd.pose.position.z-.9);
  close(delta.norm(),std::min(limit,Eigen::Vector3d(-.02,-.06,double(a.takeoff_point[2])-.9).norm()));
  for(double z:{.94,1.05,1.15,1.38,1.4}){
   pose(a,.02,.06,z);a.tick();atGoal(a,a.patrol_cmd);expectedHoldOutput(a);
   close(tf::getYaw(a.patrol_cmd.pose.orientation),.7);assert(a.external_waiting_for_motion_);
   if(std::abs(z-a.takeoff_point[2])<.18)atGoal(a,a.mavros_point_cmd);
  }
 }
}else if(section==1){
 LLController a;complete(a);a.have_planner_cmd=true;
 a.planner_cmd.pose.position={.3,.1,1.2};assert(a.hasValidExternalPlannerCommand());
 a.tick();assert(a.have_planner_cmd&&a.external_waiting_for_motion_);atGoal(a,a.patrol_cmd);
 assert(a.mavros_point_cmd.pose.position.x<.02);
 for(int command:{patrol_control::MissionCommand::SEARCH,patrol_control::MissionCommand::APPROACH,
                  patrol_control::MissionCommand::RESUME,patrol_control::MissionCommand::RETURN_HOME}){
  LLController b;complete(b);b.have_planner_cmd=true;authorize(b,command);
  assert(!b.external_waiting_for_motion_&&!b.have_planner_cmd);
  pose(b,0.,0.,1.18);b.have_planner_cmd=true;b.planner_cmd.pose.position={.1,.05,1.18};
  assert(b.hasValidExternalPlannerCommand());b.tick();
  close(b.mavros_point_cmd.pose.position.x,.1);close(b.mavros_point_cmd.pose.position.y,.05);
  assert(!b.external_planner_height_hold_active_&&!b.external_waiting_for_motion_);
 }
}else if(section==2){
 LLController a;complete(a);
 a.patrol_cmd.pose.position={.01,.02,.91};a.have_planner_cmd=true;
 a.planner_cmd.pose.position={.3,.1,1.2};a.tick();
 close(a.mavros_point_cmd.pose.position.x,.01);close(a.mavros_point_cmd.pose.position.z,.91);
 authorize(a);a.tick();auto hold=a.last_mavros_point_cmd;
 a.have_planner_cmd=true;a.latest_planner_cmd_time_.value=98.;a.tick();
 close(a.mavros_point_cmd.pose.position.z,hold.pose.position.z);
 a.latest_planner_cmd_time_.value=100.;a.planner_cmd.pose.position={5.,0.,1.2};a.tick();
 close(a.mavros_point_cmd.pose.position.z,hold.pose.position.z);
}else if(section==3){
 LLController a;authorize(a);assert(!a.flag_takeoff_done&&!a.external_waiting_for_motion_);
 a.external_mission_mode_=false;pose(a,0.,0.,1.18);
 assert(a.flag_takeoff_done&&!a.ready&&!a.external_waiting_for_motion_&&!a.external_planner_height_hold_active_);
}else if(section==4){
 LLController a;complete(a);a.max_yaw_change=.2;a.tick();
 close(tf::getYaw(a.mavros_point_cmd.pose.orientation),.2);
 close(tf::getYaw(a.patrol_cmd.pose.orientation),.7);
 a.uav_pose.pose.orientation=tf::createQuaternionMsgFromYaw(.55);a.tick();
 close(tf::getYaw(a.mavros_point_cmd.pose.orientation),.7);
}else{assert(false);}
std::cout<<"continuous handoff section "<<section<<" PASS\n";
}
'''
        cpp = Path(cls.folder.name) / 'test.cpp'
        cpp.write_text(code)
        subprocess.run(['g++', '-std=c++14', '-fsanitize=undefined',
                        '-fno-sanitize-recover=undefined', '-I/usr/include/eigen3',
                        str(cpp), '-o', str(cls.exe)], check=True)

    def run_section(self, section):
        subprocess.run([str(self.exe), str(section)], check=True)

    def test_first_frame_and_repeated_no_planner_outputs_keep_endpoint(self):
        self.run_section(0)

    def test_old_planner_ignored_until_motion_authorized_then_new_planner_takes_over(self):
        self.run_section(1)

    def test_existing_pending_hold_and_invalid_planner_fallback(self):
        self.run_section(2)

    def test_pre_takeoff_commands_and_legacy_mode_unchanged(self):
        self.run_section(3)

    def test_yaw_limiter_never_replaces_retained_takeoff_yaw(self):
        self.run_section(4)


if __name__ == '__main__':
    unittest.main(verbosity=2)
