"""Run actual FSM odom/context methods with inert transport; no ROS is started."""
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


def method(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 0
    for end in range(opening, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if depth == 0:
            return source[start:end + 1]
    raise AssertionError(signature)


PROGRAM = r'''
#include <plan_manage/navigation_recovery.h>
#include <cassert>
#include <memory>
namespace ros {
double clock=10;
struct Time { double value=0; Time(double v=0):value(v) {} bool isZero() const {return value==0;}
  double toSec() const{return value;} uint64_t toNSec() const{return value*1e9;}
  static Time now(){return Time(clock);} };
struct Duration { double value;double toSec() const{return value;} };
Duration operator-(Time a,Time b){return Duration{a.value-b.value};}
bool operator>=(Time a,Time b){return a.value>=b.value;}
bool operator==(Time a,Time b){return a.value==b.value;}
}
struct Header {ros::Time stamp;std::string frame_id;};
namespace nav_msgs {
struct Odometry {Header header;std::string child_frame_id;
  struct {struct {struct {double x=0,y=0,z=0;} position;
    struct {double w=1,x=0,y=0,z=0;} orientation;} pose;} pose;
  struct {struct {struct {double x=0,y=0,z=0;} linear;} twist;} twist;};
using OdometryConstPtr=std::shared_ptr<const Odometry>;
}
namespace fast_planner {
struct Height {bool enabled=true,valid=true;double max_z=2.9;std::string frame="camera_init";};
Height referenceHeight(){return Height();}
struct Map {ros::Time recoveryStamp()const{return ros::Time(10);}
  std::string recoveryFrame()const{return "camera_init";} uint64_t recoveryRevision()const{return 1;}};
struct Environment {std::shared_ptr<Map> sdf_map_=std::make_shared<Map>();};
struct Manager {std::shared_ptr<Environment> edt_environment_=std::make_shared<Environment>();};
struct GoalTracker {struct Goal {Header header;} goal; const Goal& effectiveGoal()const{return goal;}};
class KinoReplanFSM {public:
  recovery::Config recovery_config_;
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  struct {Header header;ros::Time deadline;bool active=true;} recovery_context_;
  struct {bool connected=true,armed=true;std::string mode="OFFBOARD";} recovery_fc_state_;
  ros::Time recovery_fc_stamp_{10},controller_stamp_{10},odom_stamp_;
  GoalTracker goal_status_tracker_;
  recovery::Vec odom_pos_=recovery::Vec::Zero(),odom_vel_=recovery::Vec::Zero(),end_pt_{1,0,1};
  Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();
  bool have_target_=true,have_odom_=false;int controller_mode_=1;
  std::string odom_frame_,recovery_odom_child_frame_,recovery_odom_twist_frame_="child";
  void odometryCallback(const nav_msgs::OdometryConstPtr&);
  recovery::Context recoveryContext()const;
};
PRODUCTION_METHODS
}
int main(int argc,char** argv) {
  using namespace fast_planner;using namespace recovery;
  assert(argc==2);const std::string name=argv[1];const double pi=std::acos(-1.);
  KinoReplanFSM f;f.recovery_config_.enabled=true;
  f.recovery_context_.header.stamp=f.goal_status_tracker_.goal.header.stamp=ros::Time(9);
  f.recovery_context_.header.frame_id="camera_init";f.recovery_context_.deadline=ros::Time(30);
  auto msg=std::make_shared<nav_msgs::Odometry>();
  msg->header.stamp=ros::Time(10);msg->header.frame_id="camera_init";msg->child_frame_id="base_link";
  msg->pose.pose.position.z=1.;msg->twist.twist.linear.x=.1;
  Eigen::Quaterniond q(Eigen::AngleAxisd(pi/2.,Vec::UnitZ()));Vec expected(0,.1,0);
  if(name=="tilted") {q=Eigen::Quaterniond(Eigen::AngleAxisd(pi/6.,Vec::UnitX()));
    msg->twist.twist.linear.x=0.;msg->twist.twist.linear.z=.1;expected=Vec(0,-.05,.1*std::cos(pi/6.));}
  if(name=="world") {f.recovery_odom_twist_frame_="world";expected=Vec(.1,0,0);}
  if(name=="missing_child") msg->child_frame_id="";
  if(name=="invalid_orientation" || name=="invalid_orientation_world") q=Eigen::Quaterniond(0,0,0,0);
  if(name=="invalid_orientation_world") f.recovery_odom_twist_frame_="world";
  if(name=="nonfinite_orientation") q=Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(),0,0,0);
  if(name=="nonfinite_velocity") msg->twist.twist.linear.y=std::numeric_limits<double>::quiet_NaN();
  if(name=="stale") msg->header.stamp=ros::Time(9.7);
  if(name=="future") msg->header.stamp=ros::Time(10.005);
  msg->pose.pose.orientation.w=q.w();msg->pose.pose.orientation.x=q.x();
  msg->pose.pose.orientation.y=q.y();msg->pose.pose.orientation.z=q.z();
  f.odometryCallback(msg);auto x=f.recoveryContext();
  assert(f.recovery_odom_child_frame_==msg->child_frame_id);
  assert(x.evidence.stamp==msg->header.stamp.toSec());
  assert(x.action_deadline==30 && x.mission_deadline==30 && x.soft_max_z==2.9);
  if(name=="missing_child") assert(admission(f.recovery_config_,x)=="recovery_child_frame_missing");
  else if(name=="invalid_orientation" || name=="invalid_orientation_world" || name=="nonfinite_orientation")
    assert(admission(f.recovery_config_,x)=="recovery_orientation_invalid");
  else if(name=="nonfinite_velocity") assert(admission(f.recovery_config_,x)=="recovery_velocity_nonfinite");
  else {
    assert((x.velocity-expected).norm()<1e-12);
    // Recovery conversion never writes back to the ordinary planning velocity.
    assert(f.odom_vel_==Vec(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z));
    if(name=="stale" || name=="future") assert(admission(f.recovery_config_,x)=="stale_input");
    else assert(admission(f.recovery_config_,x).empty());
  }
}
'''


class ProductionRecoveryVelocityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        odom = (PACKAGE / 'src/kino_replan_fsm.cpp').read_text(encoding='utf-8')
        recovery = (PACKAGE / 'src/navigation_recovery.cpp').read_text(encoding='utf-8')
        methods = method(odom, 'void KinoReplanFSM::odometryCallback(') + '\n' + method(
            recovery, 'recovery::Context KinoReplanFSM::recoveryContext(')
        cls.temp = tempfile.TemporaryDirectory(prefix='production_recovery_velocity_')
        cls.addClassCleanup(cls.temp.cleanup)
        cpp = Path(cls.temp.name) / 'velocity.cpp'
        cls.binary = Path(cls.temp.name) / 'velocity'
        cpp.write_text(PROGRAM.replace('PRODUCTION_METHODS', methods), encoding='utf-8')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I/usr/include/eigen3', '-I', str(PACKAGE / 'include'),
                        str(cpp), '-o', str(cls.binary)], check=True, timeout=30)

    def check_case(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=10)


def case(name):
    def test(self):
        self.check_case(name)
    test.__name__ = 'test_' + name
    return test


for name in ('yaw_90', 'tilted', 'world', 'missing_child', 'invalid_orientation',
             'invalid_orientation_world', 'nonfinite_orientation', 'nonfinite_velocity', 'stale', 'future'):
    setattr(ProductionRecoveryVelocityTests, 'test_' + name, case(name))


if __name__ == '__main__':
    unittest.main()