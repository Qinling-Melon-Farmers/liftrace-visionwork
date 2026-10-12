"""Production controller mode-4 regression with transport doubles; no ROS/flight."""
from pathlib import Path
import tempfile
import subprocess
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_external_landing_handoff as harness

CASES = r"""
void extended(LLController& c, int kind, double stamp=-1) {
    auto msg=std::make_shared<mavros_msgs::ExtendedState>();
    msg->header.stamp=ros::Time(stamp < 0 ? ros::clock : stamp);
    msg->landed_state=kind;
    c.externalLandingExtendedStateCallback(msg);
}
LLController offboard(bool aligned=true) {
    LLController c;
    c.offboard_landing_config_.ground_z=-.22;
    c.landing_descent_config_.mode=4; c.external_landing_handoff_mode_="OFFBOARD";
    c.uav_pose.pose.position={1,2,.75}; c.motion_odom_.pose.pose.position={1,2,.75};
    state(c); command(c);
    c.external_landing_alignment_complete_=aligned;
    c.external_landing_aligned_goal_=c.uav_pose;
    ros::clock=100.1; state(c);
    extended(c, mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR);
    c.externalLandingTick();
    return c;
}
void ground(LLController& c, double t) {
    ros::clock=t; state(c);
    extended(c, mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND);
    c.externalLandingTick();
}
void confirm(LLController& c) { ground(c,101); ground(c,102); }
"""

BRANCHES = r"""
    if(test=="stale_motion_first_freeze_holds_z") {
        auto c=offboard(false);
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_=c.uav_pose;
        c.motion_fresh_=false;
        c.externalLandingTick();
        assert(!c.external_landing_auto_land_requested_);
        assert(c.patrol_cmd.pose.position.z==.75);
        assert(!c.externalLandingOffboardDescentReady());
        assert(c.set_mode_client.calls==0 && *c.external_landing_disarm_client_.calls==0);
    } else if(test=="raw_frozen_xy_yaw_mask") {
        auto c=offboard();
        assert(c.external_landing_auto_land_requested_ && c.set_mode_client.calls==0);
        assert(c.handoff_settle_calls_==0 && !c.flag_land);
        c.uav_pose.pose.position.x=8; c.uav_pose.pose.position.y=9;
        c.mavros_point_cmd.pose.orientation=.2;
        assert(c.externalLandingOffboardDescentReady());
        c.publishExternalLandingOffboardSetpoint();
        const auto& raw=c.external_landing_offboard_setpoint_pub_.raw;
        assert(raw[0]==2524 && raw[1]==1 && raw[2]==2 && raw[3]==.75);
        assert(raw[4]==-.3 && raw[5]==.2);
        c.externalLandingTick();
        assert(c.external_landing_handoff_pub_.calls==1); // REQUESTED only once
        c.CallLand(); assert(c.set_mode_client.calls==0);
    } else if(test=="raw_and_position_output_are_exclusive") {
        auto c=offboard();
        c.runOutputLane(); assert(c.external_landing_offboard_setpoint_pub_.calls==1);
        assert(c.mavros_point_cmd_pub.calls==0);
        c.motion_fresh_=false; c.runOutputLane();
        assert(c.external_landing_offboard_setpoint_pub_.calls==1 && c.mavros_point_cmd_pub.calls==1);
        c.motion_fresh_=true; confirm(c); c.runOutputLane();
        assert(c.external_landing_offboard_setpoint_pub_.calls==1 && c.mavros_point_cmd_pub.calls==2);
        ros::clock=102.05; extended(c,2); c.runOutputLane();
        assert(c.external_landing_offboard_setpoint_pub_.calls==1 && c.mavros_point_cmd_pub.calls==3);
        ros::clock=102.08; extended(c,1); ros::clock=102.09; extended(c,1);
        ros::clock=102.1; state(c,"POSCTL",true,false); c.runOutputLane();
        assert(c.external_landing_offboard_setpoint_pub_.calls==1 && c.mavros_point_cmd_pub.calls==4);
    } else if(test=="measured_floor_latches") {
        auto c=offboard();
        assert(std::abs(c.offboard_landing_config_.endpoint()+.04)<1e-9);
        c.uav_pose.pose.position.z=-100; // Floor authority is fresh odom, not pose.
        assert(c.externalLandingOffboardDescentReady());
        c.motion_odom_.pose.pose.position.z=-.041;
        c.motion_fresh_=false; assert(!c.externalLandingOffboardDescentReady());
        assert(!c.external_landing_offboard_floor_reached_);
        c.motion_fresh_=true; assert(!c.externalLandingOffboardDescentReady());
        assert(c.external_landing_offboard_floor_reached_);
        c.motion_odom_.pose.pose.position.z=.5;
        assert(!c.externalLandingOffboardDescentReady()); // No bounce-driven resume.
        assert(*c.external_landing_disarm_client_.calls==0);
    } else if(test=="ground_requires_inair_and_distinct_stamps") {
        auto c=offboard();
        c.offboard_ground_confirmation_.reset(100);
        extended(c,1,99.9); c.externalLandingTick();
        ground(c,101); ground(c,102);
        assert(*c.external_landing_disarm_client_.calls==0);
        ros::clock=102.1; extended(c,2);
        ground(c,103);
        for(int i=0;i<10;++i) { ros::clock=103.1+i*.1; extended(c,1,103); c.externalLandingTick(); }
        assert(*c.external_landing_disarm_client_.calls==0); // repeated same source
        ground(c,104); assert(*c.external_landing_disarm_client_.calls==1);
    } else if(test=="stale_ground_and_motion_block_disarm") {
        auto c=offboard(); ground(c,101);
        ros::clock=104; state(c); extended(c,1,101);
        assert(!c.externalLandingOffboardDescentReady());
        c.externalLandingTick(); assert(*c.external_landing_disarm_client_.calls==0);
        ground(c,105);
        c.motion_fresh_=false; ground(c,106);
        assert(*c.external_landing_disarm_client_.calls==0);
        assert(!c.externalLandingOffboardDescentReady());
        c.motion_fresh_=true; c.externalLandingTick();
        assert(*c.external_landing_disarm_client_.calls==1);
        assert(!c.externalLandingOffboardDescentReady()); // grounded/pending RPC
    } else if(test=="ordinary_disarm_retry_and_confirmed_completion") {
        auto c=offboard(); confirm(c);
        assert(*c.external_landing_disarm_client_.calls==1);
        assert(!c.external_landing_handoff_observed_); // RPC return never completion.
        ros::clock=102.5; c.externalLandingTick();
        assert(*c.external_landing_disarm_client_.calls==1);
        ros::clock=103; state(c); extended(c,1); c.externalLandingTick();
        assert(*c.external_landing_disarm_client_.calls==2);
        ros::clock=103.1; state(c,"POSCTL",true,false);
        assert(c.external_landing_handoff_observed_ && c.external_landing_active_ && !c.external_landing_cancelled_);
        assert(c.Drone_mode==Land && c.external_landing_handoff_pub_.calls==2);
        assert(!c.externalLandingOffboardDescentReady());
        c.clearExternalLandingState(true);
        assert(c.external_landing_handoff_pub_.calls==2); // STOP cannot emit CANCELLED.
    } else if(test=="disarm_before_ground_cross_topic_order") {
        auto c=offboard();
        ros::clock=101; state(c,"POSCTL",true,false);
        assert(c.external_landing_active_ && !c.external_landing_handoff_observed_);
        assert(*c.external_landing_disarm_client_.calls==0);
        extended(c,1); c.externalLandingTick();
        assert(!c.external_landing_handoff_observed_);
        ros::clock=102; state(c,"MANUAL",true,false); extended(c,1); c.externalLandingTick();
        assert(c.external_landing_handoff_observed_ && c.external_landing_active_ && !c.external_landing_cancelled_);
        assert(*c.external_landing_disarm_client_.calls==0);
    } else if(test=="disarm_without_ground_is_bounded") {
        auto c=offboard();
        ros::clock=101; state(c,"POSCTL",true,false);
        ros::clock=103.6; state(c,"POSCTL",true,false);
        cancelled(c); assert(*c.external_landing_disarm_client_.calls==0);
    } else if(test=="cancel_takeover_disconnect_state_stale_timeout") {
        for(int bad=0;bad<5;++bad) {
            ros::clock=100; auto c=offboard();
            if(bad==0) state(c,"POSCTL");
            if(bad==1) state(c,"OFFBOARD",false,true);
            if(bad==2) { ros::clock=102.7; c.externalLandingTick(); }
            if(bad==3) { c.external_landing_watchdog_timeout_sec_=.5; ros::clock=101; state(c); c.externalLandingTick(); }
            if(bad==4) { c.clearExternalLandingState(true); assert(!c.external_landing_active_); }
            if(bad!=4) cancelled(c);
            assert(!c.externalLandingOffboardDescentReady());
            assert(*c.external_landing_disarm_client_.calls==0);
        }
    } else if(test=="config_pair_and_drop_modes") {
        DropDescentConfig d; OffboardLandingConfig o; o.ground_z=-.22;
        assert(o.valid() && std::abs(o.endpoint()+.04)<1e-9);
        for(int mode=1;mode<=3;++mode) { d.mode=mode; assert(d.valid()); assert(validLandingDescent(d,"POSCTL")); assert(!validLandingDescent(d,"OFFBOARD")); }
        d.mode=4; assert(!d.valid()); assert(validLandingDescent(d,"OFFBOARD"));
        assert(!validLandingDescent(d,"POSCTL") && !validLandingDescent(d,"AUTO.LAND"));
        o.ground_z=0; assert(std::abs(o.endpoint()-.18)<1e-9);
        o.touchdown_overtravel=o.fc_clearance; assert(!o.valid());
    } else
"""

class OffboardLandingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        original = harness.PROGRAM
        source = (harness.PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        start = source.index('    if (externalLandingOffboardDescentReady()) {')
        end = source.index('    // std::cout', start)
        output = 'void LLController::runOutputLane() { bool vertical_landing=false, zero_velocity_drop=false;\n' + source[start:end] + '}\n'
        augmented = original.replace('    ros::Publisher external_landing_offboard_setpoint_pub_;',
            '    ros::Publisher external_landing_offboard_setpoint_pub_, mavros_point_cmd_pub, landing_vertical_cmd_pub_, drop_vertical_cmd_pub_;\n    void runOutputLane();')
        harness.PROGRAM = augmented.replace('int main(int argc, char** argv)', output + CASES + '\nint main(int argc, char** argv)').replace(
            '    if (test=="settlement_stub") {', BRANCHES + '    if (test=="settlement_stub") {')
        try:
            harness.ExternalLandingHandoffTest.setUpClass.__func__(cls)
        finally:
            harness.PROGRAM = original

    def check_case(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=10)

def _make_case_test(name):
    def test_case(self):
        self.check_case(name)
    test_case.__name__ = 'test_' + name
    return test_case

for case in ("stale_motion_first_freeze_holds_z", "raw_frozen_xy_yaw_mask", "raw_and_position_output_are_exclusive", "measured_floor_latches",
             "ground_requires_inair_and_distinct_stamps", "stale_ground_and_motion_block_disarm",
             "ordinary_disarm_retry_and_confirmed_completion", "disarm_before_ground_cross_topic_order",
             "disarm_without_ground_is_bounded", "cancel_takeover_disconnect_state_stale_timeout",
             "config_pair_and_drop_modes"):
    setattr(OffboardLandingTests, 'test_'+case, _make_case_test(case))


ASYNC_PROGRAM = r"""
#include "patrol_control/async_landing_disarm.h"
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <chrono>
using namespace patrol_control;
struct Gate {
    std::mutex mutex; std::condition_variable changed;
    bool started=false, release=false, done=false;
};
int main() {
    auto gate=std::make_shared<Gate>();
    AsyncLandingDisarm worker;
    assert(worker.submit([gate]() {
        std::unique_lock<std::mutex> lock(gate->mutex);
        gate->started=true; gate->changed.notify_all();
        gate->changed.wait(lock,[gate](){ return gate->release; });
        gate->done=true; gate->changed.notify_all();
    }));
    {
        std::unique_lock<std::mutex> lock(gate->mutex);
        assert(gate->changed.wait_for(lock,std::chrono::seconds(2),[gate](){return gate->started;}));
    }
    const auto before=std::chrono::steady_clock::now();
    for(int i=0;i<1000;++i) { worker.reap(); assert(!worker.submit([](){})); }
    worker.cancel();
    assert(std::chrono::steady_clock::now()-before < std::chrono::milliseconds(100));
    {
        std::unique_lock<std::mutex> lock(gate->mutex);
        gate->release=true;gate->changed.notify_all();
        assert(gate->changed.wait_for(lock,std::chrono::seconds(2),[gate](){return gate->done;}));
    }
    // Reap is non-blocking while RPC is in flight; only an actually finished
    // job allows a retry. No cancellation/reply mutates controller LAND state.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    worker.reap(); assert(worker.submit([](){}));
}
"""

class AsyncLandingDisarmTests(unittest.TestCase):
    def test_slow_rpc_does_not_block_or_queue_setpoint_thread(self):
        with tempfile.TemporaryDirectory(prefix='h_disarm_async_') as folder:
            cpp, binary = Path(folder)/'test.cpp', Path(folder)/'test'
            cpp.write_text(ASYNC_PROGRAM, encoding='utf-8')
            subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                '-pthread', '-I', str(harness.PACKAGE/'include'), str(cpp), '-o', str(binary)],
                check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=10)

if __name__ == '__main__':
    unittest.main()
