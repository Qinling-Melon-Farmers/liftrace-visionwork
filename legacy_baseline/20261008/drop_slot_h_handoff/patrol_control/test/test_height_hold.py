"""Compile production height checks and feed continuous commands/pose drift.

Transport and clock are doubles; no ROS master, aircraft or simulator is run.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


def production_method(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 0
    for end in range(opening, len(source)):
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
            if depth == 0:
                return source[start:end + 1]
    raise AssertionError(signature)


def production_program(main):
    source = (PACKAGE/'src/patrol_control.cpp').read_text()
    header = (PACKAGE/'include/patrol_control/patrol_control.h').read_text()
    epsilon = next(line.strip() for line in header.splitlines()
                   if 'static constexpr double external_planner_height_epsilon_' in line)
    methods = '\n'.join(production_method(source, sig) for sig in (
        'bool isQuaternionNormalized(',
        'void LLController::plannercmdCallback(',
        'bool LLController::hasValidExternalPlannerCommand(',
        'void LLController::holdExternalPlannerHeight(',
    ))
    start = source.index('                if (external_waiting_for_motion_) {')
    end = source.index('                ROS_INFO_THROTTLE(5, "[PatrolControl] Forwarding external planner trajectory")', start)
    branch = source[start:end]
    start = source.index('    Eigen::Vector3d current_pos(uav_pose.pose.position.x')
    end = source.index('    // ', source.index('external_planner_height_hold_active_ = false;', start))
    limiter = source[start:end]
    return r'''
#include <cassert>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#define ROS_WARN_THROTTLE(...)
#define ROS_INFO_THROTTLE(...)
namespace std_msgs { struct Empty {}; }
namespace ros {
double clock=100;
struct Duration { double value; double toSec() const { return value; } };
struct Time { double value=0; static Time now(){return Time{clock};} };
Duration operator-(Time a, Time b){return Duration{a.value-b.value};}
struct Publisher { int calls=0; template<class T> void publish(const T&){++calls;} };
}
namespace geometry_msgs {
struct Point { double x=0,y=0,z=0; };
struct Quaternion { double x=0,y=0,z=0,w=1; };
struct Pose { Point position; Quaternion orientation; };
struct PoseStamped { Pose pose; };
}
enum Mode { Takeoff, Run_point, Aligning, Land };
class LLController {
public:
    bool external_mission_mode_=true, external_waiting_for_motion_=false;
    bool have_planner_cmd=false, external_planner_height_hold_active_=false;
    bool external_planner_command_accepted=false;
    double external_planner_max_command_z_=2.0;
    double external_planner_cmd_timeout_=.5, external_planner_start_max_distance_=.6;
    double px4_max_distance=.4, takeoff_point[3]={0,0,1.2};
    EPSILON
    Mode Drone_mode=Run_point;
    ros::Publisher height_replan_pub_;
    ros::Time height_replan_stamp_, latest_planner_cmd_time_;
    geometry_msgs::PoseStamped planner_cmd, uav_pose, mavros_point_cmd;
    geometry_msgs::PoseStamped patrol_cmd, last_mavros_point_cmd, external_planner_height_hold_;
    bool hasValidExternalPlannerCommand() const;
    void holdExternalPlannerHeight(const char* source, double rejected_z);
    void plannercmdCallback(const geometry_msgs::PoseStamped&);
    void tick();
};
METHODS
void LLController::tick(){
    external_planner_command_accepted=false;
    BRANCH
    LIMITER
    last_mavros_point_cmd=mavros_point_cmd;
}
void send(LLController& c,double x,double y,double z){
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x=x;msg.pose.position.y=y;msg.pose.position.z=z;
    c.plannercmdCallback(msg);
}
void at(LLController& c,double x,double y,double z){
    c.uav_pose.pose.position.x=x;c.uav_pose.pose.position.y=y;c.uav_pose.pose.position.z=z;
}
void expect(const LLController& c,double x,double y,double z){
    assert(c.mavros_point_cmd.pose.position.x==x);
    assert(c.mavros_point_cmd.pose.position.y==y);
    assert(c.mavros_point_cmd.pose.position.z==z);
}
int main(){ MAIN }
'''.replace('EPSILON', epsilon).replace('METHODS', methods).replace('BRANCH', branch).replace('LIMITER', limiter).replace('MAIN', main)


def compile_run(code, includes=()):
    with tempfile.TemporaryDirectory() as folder:
        cpp = Path(folder)/'test.cpp'
        cpp.write_text(code)
        exe = Path(folder)/'test'
        subprocess.run(['g++', '-std=c++14', '-fsanitize=undefined',
                        '-I/usr/include/eigen3', *includes, str(cpp), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


class HeightHoldTest(unittest.TestCase):
    def test_rejection_latches_xyz_across_drift_and_new_invalid_commands(self):
        compile_run(production_program(r'''
LLController c;at(c,1,2,1.9);
send(c,1.2,2.1,2.01);c.tick();
expect(c,1,2,1.9);assert(!c.have_planner_cmd && c.external_planner_height_hold_active_);
assert(c.height_replan_pub_.calls==1);
for(int i=0;i<400;++i){
    ros::clock+=.05;at(c,1+i*.004,2-i*.003,1.9+i*.002);
    if(i%4==0) send(c,5,6,2.01);  // continuous rejected curve
    if(i%4==2) send(c,9,9,1.8);   // too far, not recovery
    if(i%4==3) send(c,std::numeric_limits<double>::quiet_NaN(),2,1.8);
    c.tick();expect(c,1,2,1.9);
}
assert(c.height_replan_pub_.calls>1 && c.height_replan_pub_.calls<=101);
'''))

    def test_first_rejection_above_ceiling_does_not_force_descent(self):
        compile_run(production_program(r'''
LLController c;at(c,1,2,2.1);send(c,1.1,2.1,2.2);c.tick();expect(c,1,2,2.1);
for(int i=0;i<200;++i){
    ros::clock+=.05;at(c,1+i*.01,2,2.1+i*.005);c.tick();expect(c,1,2,2.1);
}
assert(c.external_planner_height_hold_active_);
'''))

    def test_final_interpolation_rejects_whole_result_until_safe_replacement(self):
        compile_run(production_program(r'''
LLController c;c.px4_max_distance=.1;at(c,1,2,2.3);
send(c,1.1,2.1,1.9);c.tick();expect(c,1,2,2.3);
assert(c.external_planner_height_hold_active_ && !c.have_planner_cmd);
for(int i=0;i<200;++i){
    ros::clock+=.05;at(c,1,2,2.3+i*.0001);
    send(c,1.1,2.1,1.9);c.tick();expect(c,1,2,2.3);
}
at(c,1,2,1.9);send(c,1.05,2.05,1.95);c.tick();expect(c,1.05,2.05,1.95);
assert(!c.external_planner_height_hold_active_ && c.have_planner_cmd);
for(int i=0;i<100;++i){
    ros::clock+=.05;at(c,1+i*.001,2,1.9);send(c,1.05+i*.001,2,1.95);c.tick();
    expect(c,1.05+i*.001,2,1.95);
}
'''))

    def test_numeric_boundary_and_invalid_samples_cannot_unlock(self):
        compile_run(production_program(r'''
LLController c;at(c,1,2,2.0);
for(double extra : {0.0,5e-10,1e-9}){
    send(c,1.1,2,2.0+extra);c.tick();expect(c,1.1,2,2.0+extra);
    assert(!c.external_planner_height_hold_active_);
}
send(c,1.1,2,2.0+2e-9);c.tick();expect(c,1,2,2.0);
send(c,1.1,2,2.0);c.planner_cmd.pose.orientation.w=0;c.tick();expect(c,1,2,2.0);
assert(c.external_planner_height_hold_active_);
send(c,1.1,2,2.0);ros::clock+=1;c.tick();expect(c,1,2,2.0);
assert(c.external_planner_height_hold_active_);
c.have_planner_cmd=false;c.tick();expect(c,1,2,2.0);
send(c,1.1,2,2.0+1e-9);c.tick();expect(c,1.1,2,2.0+1e-9);
assert(!c.external_planner_height_hold_active_);
'''))

    def test_actual_planner_uses_same_numerical_boundary(self):
        include = PACKAGE.parent/'Fast-Planner/fast_planner/plan_env/include'
        compile_run(r'''
#include <cassert>
#include <plan_env/reference_height.h>
int main(){
fast_planner::ReferenceHeight h;h.enabled=true;h.max_z=2;
for(double extra : {0.0,5e-10,1e-9,2e-9}){
    const bool accepted=extra<=1e-9;
    assert(h.accepts(2+extra)==accepted);
    Eigen::MatrixXd p=Eigen::MatrixXd::Zero(4,3);p.col(2).setConstant(2+extra);
    assert(h.controls(p)==accepted);
    assert(h.polynomial(2+extra,0,0,0,1)==accepted);
}
}
''', ['-I'+str(include), '-I/opt/ros/noetic/include'])


if __name__ == '__main__':
    unittest.main()