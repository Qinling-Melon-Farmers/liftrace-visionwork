"""Exercise production LAND callbacks/ticks with a fake clock and mode service.

No ROS master, node, flight controller, or hardware is started. As in the
existing pixel-scale test, compile the actual controller method bodies with
transport doubles, then assert service calls and transaction state changes.
"""
from pathlib import Path
import io
import json
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
    raise AssertionError('Unclosed production method: ' + signature)


PROGRAM = r'''
#include <array>
#include <cassert>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <iostream>
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_ERROR_THROTTLE(...) ((void)0)
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_ERROR(...) ((void)0)
namespace std_msgs { struct String; }
namespace ros {
double clock = 100;
struct Duration { double value; double toSec() const { return value; } };
struct Time {
    double value;
    explicit Time(double v = 0) : value(v) {}
    bool isZero() const { return value == 0; }
    static Time now() { return Time(clock); }
    unsigned long long toNSec() const { return static_cast<unsigned long long>(value*1e9); }
};
bool operator==(Time a, Time b) { return a.value == b.value; }
Duration operator-(Time a, Time b) { return Duration{a.value - b.value}; }
struct Publisher {
    int calls=0;
    std::vector<std::string> statuses;
    template<typename T> void publish(const T&) {++calls;}
    void publish(const std_msgs::String&);
};
}
struct Header { unsigned int seq=0; ros::Time stamp; std::string frame_id; };
struct Position { double x=0, y=0, z=0; };
namespace geometry_msgs {
using Point = Position;
struct PoseStamped {
    Header header;
    struct { Position position; double orientation=0; } pose;
};
}
namespace std_msgs { struct Empty {}; struct Bool { bool data=false; }; struct String { std::string data; }; }
void ros::Publisher::publish(const std_msgs::String& message) {
    ++calls; statuses.push_back(message.data);
}
namespace mavros_msgs {
struct State {
    using ConstPtr = std::shared_ptr<const State>;
    Header header; bool connected=false, armed=false; std::string mode;
};
struct SetMode {
    struct { std::string custom_mode; } request;
    struct { bool mode_sent=false; } response;
};
}
namespace patrol_control {
struct MissionCommand {
    using ConstPtr = std::shared_ptr<const MissionCommand>;
    enum { SEARCH, APPROACH, ALIGN, RESUME, RETURN_HOME, LAND };
    Header header;
    int command=LAND, target_id=1;
    std::string target_class;
    geometry_msgs::PoseStamped goal;
};
}
namespace tf {
double getYaw(double yaw) { return yaw; }
double createQuaternionMsgFromYaw(double yaw) { return yaw; }
}
bool isQuaternionNormalized(double yaw, double tolerance=1e-6) { return std::isfinite(yaw); }
struct ModeService {
    int calls=0;
    bool transport_ok=true, mode_sent=true;
    std::vector<std::string> requests;
    bool call(mavros_msgs::SetMode& msg) {
        assert(msg.request.custom_mode == "AUTO.LAND" || msg.request.custom_mode == "POSCTL");
        requests.push_back(msg.request.custom_mode);
        ++calls;
        msg.response.mode_sent=mode_sent;
        return transport_ok;
    }
};
enum Dronemode { Takeoff, Run_point, Aligning, Land, Hover };
enum Pointmode { Takeoff_point, Detect_point, Nothing_point, Land_point };
enum TaskType { MAIN_MISSION, CROSS_MISSION };
class LLController {
public:
    bool external_mission_mode_=true, external_waiting_for_motion_=false;
    bool drop_exact_projection_enabled_=false;
    void clearUavVisionAlignmentState() {}
    bool external_landing_active_=false, external_landing_new_mark_=false;
    bool external_landing_alignment_complete_=false;
    bool external_landing_auto_land_requested_=false, external_landing_cancelled_=false;
    bool auto_land=true, simulation_auto_land=false, flag_landing_detect=true;
    bool flag_land=false, flag_takeoff_done=true, have_land_mark=false;
    bool have_planner_cmd=false, have_waypoint_mark=false, have_cross_mark=false;
    bool align_ok=false;
    int external_landing_stable_count_=0, external_landing_stable_frames_=10;
    int waypoint_next=0, detection_resets=0;
    double external_landing_state_max_age_sec_=2.5;
    double external_landing_mark_max_age_sec_=.5;
    double external_landing_watchdog_timeout_sec_=120;
    double external_landing_alignment_tolerance_=.08;
    double external_landing_capture_height_=.75;
    double external_landing_auto_land_height_=.40;
    double external_landing_auto_land_retry_sec_=1;
    std::string external_landing_handoff_mode_="AUTO.LAND";
    bool external_landing_handoff_observed_=false;
    double external_landing_handoff_hold_height_=0;
    std::string external_landing_mission_id_;
    unsigned int external_landing_decision_seq_=0;
    ros::Time external_landing_wire_command_stamp_, external_landing_handoff_requested_at_;
    double external_landing_mode_transition_timeout_sec_=2.5;
    ros::Publisher external_landing_handoff_pub_;
    double external_planner_start_max_distance_=.6;
    double external_planner_cmd_timeout_=.5, external_planner_max_command_z_=3.5;
    double external_alignment_capture_height_=1.2;
    double land_height=.3, align_height=.75;
    std::string external_landing_frame_="camera_init";
    mavros_msgs::State external_landing_mavros_state_;
    ros::Time external_landing_state_receipt_;
    ros::Time external_landing_started_at_, external_landing_command_stamp_;
    ros::Time external_landing_last_mark_stamp_, external_landing_last_mark_receipt_;
    ros::Time external_landing_last_auto_land_attempt_;
    ros::Time latest_planner_cmd_time_, height_replan_stamp_;
    ros::Publisher height_replan_pub_;
    geometry_msgs::PoseStamped external_landing_goal_, external_landing_aligned_goal_;
    geometry_msgs::PoseStamped patrol_cmd, mavros_point_cmd, last_mavros_point_cmd;
    geometry_msgs::PoseStamped planner_cmd;
    geometry_msgs::PoseStamped uav_pose, land_mark_point, waypoint_temp;
    geometry_msgs::PoseStamped cross_mark_point, waypoint_mark_point;
    std::array<double,4> adjust_target_position{{0,0,0,0}};
    std::vector<std::string> goal;
    struct Waypoint { double yaw=0; };
    std::vector<Waypoint> waypoint_list{Waypoint{}};
    Dronemode Drone_mode=Run_point;
    Pointmode Point_mode=Nothing_point;
    TaskType current_task_type=MAIN_MISSION;
    ros::Publisher landing_detect_control_pub_;
    ModeService set_mode_client;
    unsigned int servo_alignment_decision_seq_=0, servo_alignment_target_id_=0;
    std::string servo_alignment_target_class_;
    ros::Time servo_alignment_stamp_;
    void cancelDropAction() {}
    void resetDetectionState() { ++detection_resets; }
    void publishLegacyVisionControl(ros::Publisher&, const std_msgs::Bool&) {}
    bool externalLandingMarkFresh(const ros::Time&) const;
    bool externalLandingControlReady(const ros::Time&) const;
    void externalLandingStateCallback(const mavros_msgs::State::ConstPtr&);
    void clearExternalLandingState(bool);
    void failExternalLanding(const std::string&);
    void externalLandingTick();
    void publishExternalLandingHandoff(const std::string&);
    void CallLand();
    void missionCommandCallback(const patrol_control::MissionCommand::ConstPtr&);
    void plannercmdCallback(const geometry_msgs::PoseStamped&);
    bool hasValidExternalPlannerCommand() const;
    void runPointTimerBranch();
};
PRODUCTION_METHODS
void LLController::runPointTimerBranch() {
    assert(external_mission_mode_ && Drone_mode==Run_point);
    PRODUCTION_RUN_POINT_BRANCH
}
void state(LLController& c, const std::string& mode="OFFBOARD", bool connected=true, bool armed=true) {
    auto msg=std::make_shared<mavros_msgs::State>();
    msg->header.stamp=ros::Time::now();
    msg->mode=mode; msg->connected=connected; msg->armed=armed;
    c.externalLandingStateCallback(msg);
}
void command(LLController& c, int kind=patrol_control::MissionCommand::LAND) {
    auto msg=std::make_shared<patrol_control::MissionCommand>();
    msg->command=kind;
    msg->goal=c.uav_pose; msg->goal.header.frame_id="camera_init";
    msg->target_class="test-mission"; msg->goal.header.seq=42;
    msg->goal.header.stamp=ros::Time::now();
    c.missionCommandCallback(msg);
}
LLController landing(bool aligned=true) {
    LLController c;
    c.uav_pose.pose.position={1,2,.35};
    state(c); command(c);
    assert(c.external_landing_active_ && c.Drone_mode==Land);
    if (aligned) {
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_=c.uav_pose;
    }
    return c;
}
void cancelled(const LLController& c) {
    assert(c.external_landing_cancelled_ && !c.external_landing_active_);
    assert(!c.external_landing_auto_land_requested_ && !c.flag_land);
    assert(c.Drone_mode==Run_point && c.Point_mode==Nothing_point);
    assert(c.external_waiting_for_motion_);
    assert(c.patrol_cmd.pose.position.z==c.uav_pose.pose.position.z);
}
int main(int argc, char** argv) {
    assert(argc==2 || argc==7);
    const std::string test=argv[1];
    if (test=="permission") {
        for (int bad=0; bad<9; ++bad) {
            auto c=landing();
            auto& s=c.external_landing_mavros_state_;
            if (bad==0) s=mavros_msgs::State();
            if (bad==1) s.connected=false;
            if (bad==2) s.armed=false;
            if (bad==3) s.mode="POSCTL";
            if (bad==4) s.header.stamp=ros::Time(97.49);
            if (bad==5) c.external_landing_state_receipt_=ros::Time(97.49);
            if (bad==6) s.header.stamp=ros::Time(100.01);
            if (bad==7) c.external_landing_state_receipt_=ros::Time(100.01);
            if (bad==8) s.header.stamp=ros::Time(0);
            assert(!c.externalLandingControlReady(ros::Time::now()));
            c.CallLand(); assert(c.set_mode_client.calls==0 && !c.flag_land);
            c.externalLandingTick(); cancelled(c);
            assert(c.set_mode_client.calls==0);
        }
    } else if (test=="fresh_boundary") {
        // The board supervisor uses 2.5 seconds; 2.0 seconds is not a second gate.
        for (double age : {2.25,2.5}) {
            auto c=landing();
            c.external_landing_mavros_state_.header.stamp=ros::Time(100-age);
            c.external_landing_state_receipt_=ros::Time(100-age);
            assert(c.externalLandingControlReady(ros::Time::now()));
            c.externalLandingTick();
            assert(c.set_mode_client.calls==1 && c.external_landing_auto_land_requested_);
            c.CallLand(); c.externalLandingTick();
            assert(c.set_mode_client.calls==1);
        }
    } else if (test=="geometry") {
        auto c=landing();
        c.uav_pose.pose.position.z=.401; c.externalLandingTick();
        assert(c.set_mode_client.calls==0);
        c.uav_pose.pose.position.z=.35;
        c.uav_pose.pose.position.x+=.081; c.externalLandingTick();
        assert(c.set_mode_client.calls==0);
        c.uav_pose.pose.position.x-=.081; c.externalLandingTick();
        assert(c.set_mode_client.calls==1);
    } else if (test=="alignment") {
        auto c=landing(false);
        c.have_land_mark=true; c.land_mark_point=c.uav_pose;
        for (int i=1; i<=10; ++i) {
            c.external_landing_new_mark_=true;
            c.external_landing_last_mark_stamp_=ros::Time::now();
            c.external_landing_last_mark_receipt_=ros::Time::now();
            c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==10 ? 1 : 0));
        }
        assert(c.external_landing_stable_count_==10);
    } else if (test=="retry") {
        for (bool transport_failure : {false,true}) {
            ros::clock=100;
            auto c=landing();
            c.set_mode_client.transport_ok=!transport_failure;
            c.set_mode_client.mode_sent=false;
            c.externalLandingTick();
            assert(c.set_mode_client.calls==1 && !c.flag_land);
            assert(!c.external_landing_auto_land_requested_);
            ros::clock=100.9; c.externalLandingTick();
            assert(c.set_mode_client.calls==1);
            ros::clock=101; c.externalLandingTick();
            assert(c.set_mode_client.calls==2);
            state(c,"POSCTL"); cancelled(c);
            ros::clock=102; state(c); c.externalLandingTick(); c.CallLand();
            assert(c.set_mode_client.calls==2); cancelled(c);
        }
    } else if (test=="retry_success") {
        for (bool transport_failure : {false,true}) {
            ros::clock=100;
            auto c=landing();
            c.set_mode_client.transport_ok=!transport_failure;
            c.set_mode_client.mode_sent=false;
            c.externalLandingTick();
            assert(c.set_mode_client.calls==1 && !c.external_landing_auto_land_requested_);
            c.set_mode_client.transport_ok=true;
            c.set_mode_client.mode_sent=true;
            ros::clock=100.9; c.externalLandingTick();
            assert(c.set_mode_client.calls==1);
            ros::clock=101; c.externalLandingTick();
            assert(c.set_mode_client.calls==2 && c.external_landing_auto_land_requested_);
            for (int i=0; i<20; ++i) {
                ros::clock+=.1;
                state(c); c.externalLandingTick(); c.CallLand(); command(c);
                assert(c.set_mode_client.calls==2 && c.flag_land);
            }
        }
    } else if (test=="takeover") {
        for (const auto* mode : {"POSCTL","MANUAL","ALTCTL","AUTO.LOITER","AUTO.LAND",""}) {
            auto c=landing(false); // cancellation applies before H acquisition too
            state(c,mode); cancelled(c);
            state(c); // mode returns before another timer tick
            command(c); c.externalLandingTick(); c.CallLand();
            cancelled(c); assert(c.set_mode_client.calls==0);
        }
    } else if (test=="connection") {
        for (bool disconnected : {false,true}) {
            auto c=landing();
            state(c,"OFFBOARD",!disconnected,disconnected);
            cancelled(c);
            state(c); command(c); c.externalLandingTick(); c.CallLand();
            cancelled(c); assert(c.set_mode_client.calls==0);
        }
    } else if (test=="state_timeout") {
        auto c=landing();
        ros::clock=102.51; c.externalLandingTick(); cancelled(c);
        state(c); command(c); c.externalLandingTick(); c.CallLand();
        cancelled(c); assert(c.set_mode_client.calls==0);
    } else if (test=="handoff") {
        auto c=landing(); c.externalLandingTick();
        assert(c.flag_land && c.external_landing_auto_land_requested_);
        state(c,"AUTO.LAND"); c.externalLandingTick(); c.CallLand();
        assert(c.external_landing_active_ && !c.external_landing_cancelled_);
        ros::clock=230; c.externalLandingTick(); // no post-handoff watchdog retry
        // Only verify no repeated mode request under these telemetry values.
        // Neither armed=false nor connected=false proves a landed terminal state.
        for (bool connected : {true,false}) {
            for (bool armed : {true,false}) {
                state(c,"AUTO.LAND",connected,armed);
                c.externalLandingTick(); c.CallLand(); command(c);
                assert(c.set_mode_client.calls==1 && !c.external_landing_cancelled_);
            }
        }
    } else if (test=="takeover_after_handoff") {
        for (bool auto_land_observed : {false,true}) {
            auto c=landing(); c.externalLandingTick();
            if (auto_land_observed) state(c,"AUTO.LAND");
            state(c,"POSCTL"); cancelled(c);
            state(c); command(c); c.externalLandingTick(); c.CallLand();
            assert(c.set_mode_client.calls==1); cancelled(c);
        }
    } else if (test=="posctl_handoff") {
        auto c=landing(); c.external_landing_handoff_mode_="POSCTL";
        c.externalLandingTick();
        assert(c.set_mode_client.requests==std::vector<std::string>{"POSCTL"});
        assert(c.flag_land && c.external_landing_auto_land_requested_);
        assert(!c.external_landing_handoff_observed_);
        assert(c.external_landing_handoff_pub_.calls==1);
        assert(c.patrol_cmd.pose.position.z==.35 && c.align_height==.35);
        // A service ACK does not prove POSCTL or landing. Until its heartbeat,
        // hold the handoff height, block old navigation, and never request again.
        state(c); c.uav_pose.pose.position.z=.34;
        c.externalLandingTick(); c.CallLand(); command(c);
        command(c,patrol_control::MissionCommand::RETURN_HOME);
        command(c,patrol_control::MissionCommand::ALIGN);
        assert(c.set_mode_client.calls==1 && c.Drone_mode==Land);
        assert(c.patrol_cmd.pose.position.z==.35);
        assert(!c.external_landing_handoff_observed_);
        state(c,"POSCTL");
        assert(c.external_landing_handoff_observed_ && !c.external_landing_cancelled_);
        assert(c.external_landing_handoff_pub_.calls==2);
        auto trajectory=c.uav_pose; trajectory.pose.position.x+=.10;
        c.plannercmdCallback(trajectory);
        ros::clock=230; state(c,"POSCTL"); c.uav_pose.pose.position.z=.1; c.externalLandingTick();
        assert(c.Drone_mode==Land && c.patrol_cmd.pose.position.z==.35);
        assert(c.set_mode_client.calls==1);
        // Telemetry is the completion authority; disarm does not send another
        // mode request or create a new LAND transaction in this controller.
        state(c,"POSCTL",true,false); c.externalLandingTick(); c.CallLand();
        assert(c.external_landing_active_ && !c.external_landing_cancelled_);
        assert(c.set_mode_client.calls==1);
    } else if (test=="posctl_takeover") {
        for (const auto* mode : {"AUTO.LAND", "MANUAL", "ALTCTL"}) {
            auto c=landing(); c.external_landing_handoff_mode_="POSCTL";
            c.externalLandingTick(); state(c,mode); cancelled(c);
            state(c); command(c); c.externalLandingTick(); c.CallLand();
            assert(c.set_mode_client.calls==1); cancelled(c);
        }
        auto c=landing(); c.external_landing_handoff_mode_="POSCTL";
        c.externalLandingTick(); state(c,"POSCTL");
        state(c); cancelled(c); // Returning OFFBOARD cannot resume the old descent.
        command(c); c.externalLandingTick(); c.CallLand();
        assert(c.set_mode_client.calls==1); cancelled(c);
        c=landing(false); c.external_landing_handoff_mode_="POSCTL";
        state(c,"POSCTL"); cancelled(c); // Early pilot POSCTL still cancels.
        assert(c.set_mode_client.calls==0);
    } else if (test=="posctl_gates_retry") {
        auto c=landing(false); c.external_landing_handoff_mode_="POSCTL";
        c.externalLandingTick(); assert(c.set_mode_client.calls==0);
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_=c.uav_pose;
        c.uav_pose.pose.position.z=.401; c.externalLandingTick();
        assert(c.set_mode_client.calls==0);
        c.uav_pose.pose.position.z=.35; c.uav_pose.pose.position.x+=.081;
        c.externalLandingTick(); assert(c.set_mode_client.calls==0);
        c.uav_pose.pose.position.x-=.081; c.set_mode_client.mode_sent=false;
        c.externalLandingTick(); assert(c.set_mode_client.calls==1 && !c.flag_land);
        ros::clock=100.9; c.externalLandingTick(); assert(c.set_mode_client.calls==1);
        c.set_mode_client.mode_sent=true; ros::clock=101; state(c);
        c.externalLandingTick(); assert(c.set_mode_client.calls==2 && c.flag_land);
        assert(c.set_mode_client.requests==std::vector<std::string>({"POSCTL","POSCTL"}));
    } else if (test=="status_json") {
        auto c=landing(); c.external_landing_handoff_mode_="POSCTL";
        c.external_landing_mission_id_="test-\"mission";
        c.externalLandingTick(); state(c,"POSCTL"); state(c,"MANUAL");
        for (const auto& status : c.external_landing_handoff_pub_.statuses)
            std::cout << status << '\n';
    } else if (test=="posctl_timeout_identity") {
        auto c=landing(); c.external_landing_handoff_mode_="POSCTL";
        c.externalLandingTick(); assert(c.external_landing_handoff_pub_.calls==1);
        ros::clock=102.51; state(c); c.externalLandingTick(); cancelled(c);
        assert(c.external_landing_handoff_pub_.calls==2); // CANCELLED after request.
        c=landing(); c.external_landing_handoff_mode_="POSCTL";
        c.externalLandingTick(); state(c,"POSCTL");
        ros::clock=105.02; c.externalLandingTick(); cancelled(c); // Stale expected-mode state.
        LLController missing;
        missing.external_landing_handoff_mode_="POSCTL"; state(missing);
        auto msg=std::make_shared<patrol_control::MissionCommand>();
        msg->goal=missing.uav_pose; msg->goal.header.frame_id="camera_init";
        missing.missionCommandCallback(msg);
        assert(!missing.external_landing_active_ && missing.set_mode_client.calls==0);
    } else if (test=="planner_hold") {
        auto c=landing();
        auto old_trajectory=c.uav_pose;
        old_trajectory.pose.position.x+=.10;
        c.plannercmdCallback(old_trajectory);
        assert(c.hasValidExternalPlannerCommand());
        state(c,"POSCTL"); cancelled(c);
        const auto hold=c.patrol_cmd;
        for (int i=0; i<8; ++i) {
            ros::clock+=.05;
            old_trajectory.pose.position.x+=.01;
            c.plannercmdCallback(old_trajectory);
            // These are fresh, geometrically admissible samples of the old path.
            assert(c.have_planner_cmd && c.hasValidExternalPlannerCommand());
            if (i==2) state(c); // pilot returns to OFFBOARD
            command(c); // the same LAND must not release either cancellation or hold
            c.runPointTimerBranch();
            assert(c.mavros_point_cmd.pose.position.x==hold.pose.position.x);
            assert(c.mavros_point_cmd.pose.position.y==hold.pose.position.y);
            assert(c.mavros_point_cmd.pose.position.z==hold.pose.position.z);
            cancelled(c); assert(c.set_mode_client.calls==0);
        }
        assert(c.external_waiting_for_motion_);
        command(c,patrol_control::MissionCommand::RETURN_HOME);
        assert(!c.external_waiting_for_motion_ && !c.have_planner_cmd);
        c.runPointTimerBranch();
        assert(c.mavros_point_cmd.pose.position.x==hold.pose.position.x);
        c.plannercmdCallback(old_trajectory); c.runPointTimerBranch();
        assert(c.mavros_point_cmd.pose.position.x==old_trajectory.pose.position.x);
    } else if (test=="new_transaction") {
        auto c=landing(); state(c,"POSCTL"); cancelled(c);
        state(c); command(c); cancelled(c);
        // Only an explicit new non-LAND mission transaction clears cancellation.
        command(c,patrol_control::MissionCommand::RETURN_HOME);
        assert(!c.external_landing_cancelled_ && !c.external_landing_active_);
        command(c); assert(c.external_landing_active_);
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_=c.uav_pose;
        c.externalLandingTick(); assert(c.set_mode_client.calls==1);
    } else if (test=="disabled_inactive") {
        auto c=landing(); c.auto_land=false;
        c.externalLandingTick(); c.CallLand();
        assert(c.set_mode_client.calls==0 && !c.flag_land);
        c=landing(); c.external_landing_active_=false; c.CallLand();
        assert(c.set_mode_client.calls==0 && !c.flag_land);
    } else if (test=="align_identity") {
        LLController c;
        auto msg=std::make_shared<patrol_control::MissionCommand>();
        msg->command=patrol_control::MissionCommand::ALIGN;
        msg->header.seq=1; msg->goal.header.seq=42;
        msg->header.stamp=ros::Time::now();
        msg->target_id=7; msg->target_class="panzer";
        c.missionCommandCallback(msg);
        assert(c.detection_resets==1 && c.Drone_mode==Aligning);
        assert(c.servo_alignment_decision_seq_==42);
        // Receipt time may change, but the same decision is still one action.
        msg->header.seq=2; msg->header.stamp=ros::Time(101);
        c.missionCommandCallback(msg);
        assert(c.detection_resets==1);
        msg->goal.header.seq=43;
        c.missionCommandCallback(msg);
        assert(c.detection_resets==2);
        // Compatibility fallback for old publishers with nested seq=0.
        msg->goal.header.seq=0; msg->header.stamp=ros::Time(102);
        c.missionCommandCallback(msg); c.missionCommandCallback(msg);
        assert(c.detection_resets==3);
    } else if (test=="align_transport") {
        assert(argc==7);
        LLController c;
        auto msg=std::make_shared<patrol_control::MissionCommand>();
        msg->command=patrol_control::MissionCommand::ALIGN;
        msg->target_id=7; msg->target_class="panzer";
        msg->header.stamp=ros::Time::now();
        // Values decoded from actual rospy.serialize_message packets in Python.
        msg->header.seq=std::stoul(argv[2]);
        msg->goal.header.seq=std::stoul(argv[3]);
        const auto context_decision_seq=std::stoul(argv[6]);
        assert(msg->header.seq!=msg->goal.header.seq);
        c.missionCommandCallback(msg);
        assert(c.Drone_mode==Aligning && c.detection_resets==1);
        assert(c.servo_alignment_decision_seq_==context_decision_seq);
        assert(c.servo_alignment_target_id_==7 && c.servo_alignment_target_class_=="panzer");
        // Republishing the same decision changes transport seq, not action identity.
        msg->header.seq=std::stoul(argv[4]);
        msg->header.stamp=ros::Time(101);
        c.missionCommandCallback(msg);
        assert(c.detection_resets==1 && c.servo_alignment_decision_seq_==context_decision_seq);
        // A new nested decision must replace the old identity even if transport seq repeats.
        msg->goal.header.seq=std::stoul(argv[5]);
        c.missionCommandCallback(msg);
        assert(c.detection_resets==2 && c.servo_alignment_decision_seq_==msg->goal.header.seq);
        assert(c.servo_alignment_decision_seq_!=context_decision_seq);
    } else if (test=="legacy") {
        for (bool simulation : {false,true}) {
            for (bool accepted : {false,true}) {
                LLController c; c.external_mission_mode_=false;
                c.simulation_auto_land=simulation;
                c.external_landing_handoff_mode_="POSCTL"; // External-only parameter.
                c.set_mode_client.mode_sent=accepted;
                c.external_landing_cancelled_=true;
                c.CallLand();
                assert(c.flag_land);
                assert(c.set_mode_client.calls==(simulation ? 1 : 0));
                if (simulation) assert(c.set_mode_client.requests==std::vector<std::string>{"AUTO.LAND"});
                assert(c.align_height==(simulation ? c.land_height : -1.));
                state(c,"POSCTL"); assert(c.flag_land && c.Drone_mode==Run_point);
            }
        }
    } else { return 2; }
}
'''


class ExternalLandingHandoffTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        signatures = (
            'bool LLController::externalLandingMarkFresh(',
            'bool LLController::externalLandingControlReady(',
            'void LLController::externalLandingStateCallback(',
            'void LLController::publishExternalLandingHandoff(',
            'void LLController::clearExternalLandingState(',
            'void LLController::failExternalLanding(',
            'void LLController::externalLandingTick(',
            'void LLController::CallLand(',
            'void LLController::missionCommandCallback(',
            'void LLController::plannercmdCallback(',
            'bool LLController::hasValidExternalPlannerCommand(',
        )
        methods = '\n'.join(production_method(source, item) for item in signatures)
        timer = production_method(source, 'void LLController::cmdCallback(')
        run_point = timer.index('case Run_point:')
        start = timer.index('if (external_waiting_for_motion_) {', run_point)
        end = timer.index(
            'ROS_INFO_THROTTLE(5, "[PatrolControl] Forwarding external planner trajectory")',
            start)
        timer_branch = timer[start:end]
        cls.temp = tempfile.TemporaryDirectory(prefix='h_rc_handoff_test_')
        cls.addClassCleanup(cls.temp.cleanup)
        cpp = Path(cls.temp.name) / 'handoff.cpp'
        cls.binary = Path(cls.temp.name) / 'handoff'
        cpp.write_text(PROGRAM.replace('PRODUCTION_METHODS', methods).replace(
            'PRODUCTION_RUN_POINT_BRANCH', timer_branch), encoding='utf-8')
        subprocess.run([
            'g++', '-std=c++14', '-O0', '-Wall', '-Wextra',
            '-Wno-unused-parameter', '-fsanitize=undefined',
            '-fno-sanitize-recover=all', str(cpp), '-o', str(cls.binary),
        ], check=True)

    def test_repeated_align_cannot_reset_one_servo_action(self):
        self.run_case('align_identity')

    def test_real_rospy_transport_preserves_nested_align_decision_identity(self):
        from rospy.msg import serialize_message
        from patrol_control.msg import MissionCommand
        from uav_vision.msg import AlignmentTargetContext

        decision_seq = 10
        context = AlignmentTargetContext()
        context.decision_seq = decision_seq

        def received(transport_seq, stable_seq):
            message = MissionCommand()
            message.command = MissionCommand.ALIGN
            message.target_id = 7
            message.target_class = 'panzer'
            message.header.seq = stable_seq
            message.goal.header.seq = stable_seq
            buffer = io.BytesIO()
            serialize_message(buffer, transport_seq, message)
            decoded = MissionCommand().deserialize(buffer.getvalue()[4:])
            self.assertEqual(message.header.seq, transport_seq)
            self.assertEqual(decoded.header.seq, transport_seq)
            self.assertEqual(decoded.goal.header.seq, stable_seq)
            self.assertEqual(decoded.target_id, 7)
            self.assertEqual(decoded.target_class, 'panzer')
            return decoded

        first = received(3, decision_seq)
        duplicate = received(4, decision_seq)
        successor = received(4, decision_seq + 1)
        subprocess.run([
            str(self.binary), 'align_transport', str(first.header.seq),
            str(first.goal.header.seq), str(duplicate.header.seq),
            str(successor.goal.header.seq), str(context.decision_seq),
        ], check=True)

    def run_case(self, name):
        subprocess.run([str(self.binary), name], check=True)

    def test_fresh_connected_armed_offboard_is_required_at_service_boundary(self):
        self.run_case('permission')

    def test_freshness_boundary_allows_exactly_one_successful_request(self):
        self.run_case('fresh_boundary')

    def test_existing_xy_and_height_gates_remain_required(self):
        self.run_case('geometry')

    def test_existing_ten_fresh_h_frames_remain_required(self):
        self.run_case('alignment')

    def test_failed_requests_retry_only_before_takeover(self):
        self.run_case('retry')

    def test_rejected_request_can_succeed_on_retry_and_never_repeat_after_success(self):
        self.run_case('retry_success')

    def test_mode_takeover_between_ticks_cancels_and_blocks_duplicate_land(self):
        self.run_case('takeover')

    def test_disconnection_and_disarming_cancel_pending_land(self):
        self.run_case('connection')

    def test_state_timeout_cannot_resume_on_a_later_offboard_heartbeat(self):
        self.run_case('state_timeout')

    def test_auto_land_handoff_does_not_reissue_on_disarmed_or_disconnected_state(self):
        self.run_case('handoff')

    def test_manual_takeover_after_request_cannot_trigger_another_request(self):
        self.run_case('takeover_after_handoff')

    def test_posctl_request_holds_z_until_observed_and_waits_for_pilot(self):
        self.run_case('posctl_handoff')

    def test_posctl_handoff_preserves_early_and_post_handoff_takeover_cancellation(self):
        self.run_case('posctl_takeover')

    def test_posctl_preserves_geometry_alignment_and_retry_gates(self):
        self.run_case('posctl_gates_retry')

    def test_posctl_requires_transaction_identity_and_mode_transition_freshness(self):
        self.run_case('posctl_timeout_identity')

    def test_actual_cpp_status_json_keeps_identity_and_integer_nanoseconds(self):
        output = subprocess.check_output([str(self.binary), 'status_json'], text=True)
        events = [json.loads(line) for line in output.splitlines()]
        self.assertEqual([event['stage'] for event in events], ['REQUESTED', 'OBSERVED', 'CANCELLED'])
        for event in events:
            self.assertEqual(event['mission_id'], 'test-"mission')
            self.assertEqual(event['decision_seq'], 42)
            self.assertEqual(event['mode'], 'POSCTL')
            self.assertIsInstance(event['command_stamp_ns'], str)
            self.assertIsInstance(event['event_stamp_ns'], str)
            self.assertEqual(int(event['command_stamp_ns']), 100_000_000_000)

    def test_planner_callback_and_timer_hold_after_takeover_until_new_mission(self):
        self.run_case('planner_hold')

    def test_only_a_new_mission_transaction_can_clear_cancellation(self):
        self.run_case('new_transaction')

    def test_disabled_or_inactive_external_landing_cannot_request_mode(self):
        self.run_case('disabled_inactive')

    def test_nonexternal_landing_preserves_legacy_success_and_failure_behavior(self):
        self.run_case('legacy')


if __name__ == '__main__':
    unittest.main()
