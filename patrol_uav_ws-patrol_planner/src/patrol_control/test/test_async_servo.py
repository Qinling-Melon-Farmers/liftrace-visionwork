"""Execute the real worker and controller submit/poll/cancel bodies without ROS.

Transport sleeps/blocks are doubles only: no master, nodes or PWM are started.
"""
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
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
            if not depth:
                return source[start:end + 1]
    raise AssertionError(signature)


PROGRAM = r"""
#include "patrol_control/async_servo.h"
#include "patrol_control/servo_action_result.h"
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <iostream>
#include <string>
#include <vector>
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_ERROR(...) ((void)0)
namespace patrol_control {
struct Servo { struct { int req=0; } request; struct { bool res=false; } response; };
struct ServoAction {
    struct Request {
        std::uint64_t request_id=0;
        unsigned int payload_slot=0, decision_seq=0, attempt=0, target_id=0;
        std::string mission_id, target_class, align_mode, permission_epoch;
        std::uint64_t permission_revision=0;
        double target_first_seen=0;
    };
    struct Response {
        enum { NOT_STARTED=1, RAW_CALL_STARTED=2, COMPLETED=3 };
        std::uint64_t request_id=0;
        unsigned int payload_slot=0;
        bool res=false, terminal=true;
        int execution_state=0;
    };
    Request request; Response response;
};
}
struct Transport {
    std::atomic<int> calls{0};
    bool success=true, transport_ok=true, throw_error=false;
    int delay_ms=1000;
    int fact=-1, echo_request_offset=0, echo_slot_override=0;
    bool terminal=true;
    std::uint64_t last_request_id=0;
    unsigned int last_decision_seq=0;
    std::string permission_epoch; std::uint64_t permission_revision=0;
    bool gated=false, released=false;
    std::mutex mutex; std::condition_variable cv;
};
namespace ros {
struct Time {
    double value;
    Time(double v=0):value(v) {}
    bool isZero() const { return value==0; }
    double toSec()const{return value;}
    static Time now() { return Time(100); }
    operator double() const { return value; }
};
bool operator<=(Time a, Time b) { return a.value<=b.value; }
bool operator==(Time a, Time b) { return a.value==b.value; }
struct Duration { double toSec() const { return 0; } };
Duration operator-(Time, Time) { return Duration(); }
struct ServiceClient {
    std::shared_ptr<Transport> impl;
    bool call(patrol_control::ServoAction& srv) {
        impl->permission_epoch=srv.request.permission_epoch;
        impl->permission_revision=srv.request.permission_revision;
        impl->last_request_id=srv.request.request_id;
        impl->last_decision_seq=srv.request.decision_seq;
        patrol_control::Servo legacy;
        const bool ok=call(legacy);
        srv.response.request_id=srv.request.request_id+impl->echo_request_offset;
        srv.response.payload_slot=impl->echo_slot_override ? impl->echo_slot_override : srv.request.payload_slot;
        srv.response.res=impl->success; srv.response.terminal=impl->terminal;
        srv.response.execution_state=impl->fact<0 ? (impl->success ? 3 : 2) : impl->fact;
        return ok;
    }
    bool call(patrol_control::Servo& srv) {
        ++impl->calls;
        if (impl->gated) {
            std::unique_lock<std::mutex> lock(impl->mutex);
            impl->cv.wait(lock, [&] { return impl->released; });
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(impl->delay_ms));
        if (impl->throw_error) throw 1;
        srv.response.res=impl->success;
        return impl->transport_ok;
    }
};
}
namespace mavros_msgs {
struct State {
    struct { ros::Time stamp; } header;
    using ConstPtr=std::shared_ptr<const State>;
    bool connected=true, armed=true;
    std::string mode="OFFBOARD";
};
}
namespace uav_vision {
struct AlignmentTargetContext {
    using ConstPtr=std::shared_ptr<const AlignmentTargetContext>;
    enum { SCHEMA_VERSION=1, ALIGN=2 };
    int schema_version=1, command=2;
    bool active=true, has_target=true;
    std::string mission_id="mission-A", semantic_target_class="panzer", align_mode="drop_circle";
    unsigned int decision_seq=900, semantic_target_id=7, payload_slot=1, attempt=0;
    ros::Time semantic_target_first_seen=ros::Time(1), deadline=ros::Time(200);
};
}
struct Bool { bool data=false; };
struct Header{ros::Time stamp;std::string frame_id;};
namespace mavros_msgs {
struct PositionTarget {
 enum {FRAME_LOCAL_NED=1,IGNORE_PZ=4,IGNORE_VX=8,IGNORE_VY=16,IGNORE_AFX=64,IGNORE_AFY=128,IGNORE_AFZ=256,IGNORE_YAW_RATE=2048};
 Header header;
 struct XYZ{double x=0,y=0,z=0;}position,velocity;
 int coordinate_frame=0,type_mask=0;double yaw=0;
};
}
namespace tf {double getYaw(int){return 0;}}
struct Pose {
    Header header;
    struct {mavros_msgs::PositionTarget::XYZ position;int orientation=0;}pose;
};
struct Publisher { int calls=0; template<typename T>void publish(const T&) { ++calls; } };
namespace patrol_control {
class LLController {
public:
    bool compensated_alignment_enabled_=true, geometry_ready=true;
    // Async-only fixture uses explicit simultaneous compatibility mode.
    bool drop_compensate_before_descent_=false, drop_descent_started_=false;
    bool compensatedDropSettled(bool) { return geometry_ready; }
    void publishCompensatedAlignment() {}
    std::string external_landing_handoff_mode_="AUTO.LAND";
    bool external_landing_handoff_observed_=false;
    ros::Time external_landing_handoff_requested_at_;
    double external_landing_mode_transition_timeout_sec_=2.5, external_landing_state_max_age_sec_=2.5;
    void publishExternalLandingHandoff(const std::string&) {}


    ros::ServiceClient servo_client, servo_action_client_;
    uav_vision::AlignmentTargetContext servo_alignment_context_;
    bool have_servo_alignment_context_=true;
    struct { std::string permission_epoch="test"; std::uint64_t permission_revision=18; } release_authorization_;
    bool permission_ready=true;
    bool hasFreshMissionReleasePermission() const { return permission_ready; }
    unsigned int servo_alignment_decision_seq_=900, servo_alignment_target_id_=7;
    std::string servo_alignment_target_class_="panzer";
    void servoAlignmentContextCallback(const uav_vision::AlignmentTargetContext::ConstPtr&);
    enum { Run_point, Aligning };
    int Drone_mode=Aligning;
    ros::Time detection_start_time, drop_alignment_started_, drop_descent_started_at_;
    double waypoint_adjust_max_second_threshould=15;
    Pose mavros_point_cmd;
    Publisher mavros_point_cmd_pub;
    void timerPublicationPath();
    // Transport-only fixture: existing H/raw branches stay inactive.
    struct { int mode=1; } landing_descent_config_;
    bool vertical_landing=false, zero_velocity_drop=false, vertical_speed_drop=false;
    double drop_vz_command=0;
    std::string external_landing_frame_="camera_init";
    Publisher landing_vertical_cmd_pub_, drop_vertical_cmd_pub_,drop_velocity_cmd_pub_;
    bool externalLandingOffboardStateReady(ros::Time) const { return true; }
    void externalLandingOffboardTick() {}
    bool externalLandingOffboardDescentReady() const { return false; }
    void publishExternalLandingOffboardSetpoint() {}
    bool control_ready=true;
    bool external_mission_mode_=true, external_landing_active_=false;
    bool external_landing_auto_land_requested_=false, external_landing_cancelled_=false;
    mavros_msgs::State external_landing_mavros_state_;
    ros::Time external_landing_state_receipt_;
    bool externalLandingControlReady(const ros::Time&) const { return control_ready; }
    void externalLandingStateCallback(const mavros_msgs::State::ConstPtr&);
    void failExternalLanding(const std::string&) {}
    struct {void pause(double){}}drop_descent_profile_;
    AsyncServo async_servo_;
    std::array<AsyncServo,3> no_ack_servo_workers_;
    bool drop_recover_without_ack_=false, drop_recovery_started_=false;
    double drop_recovery_delay_sec_=0.0; // legacy fixtures test the zero-delay policy
    uav_vision::AlignmentTargetContext servo_invocation_context_;
    int invocation_events=0;
    void publishDropInvocation(){++invocation_events;}
    AsyncServo& servoWorkerForSlot(int);
    void beginDropRecoveryWithoutAck();
    std::uint64_t servo_action_id_=1;
    int servo_action_slot_=0;
    bool servo_action_attempted_=false, servo_action_pending_=false;
    DropActionResult servo_action_result_=DropActionResult::kPending;
    double servo_call_timeout_sec_=10;
    Bool servo_complete;
    bool drop_complete=false;
    int detect_point_counter=0;
    std::vector<bool> drop_completed{false,false,false};
    DropActionResult executeDropAction(int);
    void pollDropAction();
    void cancelDropAction();
};
PRODUCTION_METHODS
}
using namespace patrol_control;
using Clock=std::chrono::steady_clock;
void waitStarted(const std::shared_ptr<Transport>& transport) {
    const auto end=Clock::now()+std::chrono::seconds(2);
    while (!transport->calls && Clock::now()<end) std::this_thread::yield();
    assert(transport->calls==1);
}
void release(const std::shared_ptr<Transport>& transport) {
    { std::lock_guard<std::mutex> lock(transport->mutex); transport->released=true; }
    transport->cv.notify_all();
}
void finish(LLController& c) {
    const auto end=Clock::now()+std::chrono::seconds(3);
    while (c.servo_action_pending_ && Clock::now()<end) {
        c.pollDropAction();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(!c.servo_action_pending_);
}
void resetContext(LLController& c) {
    c.cancelDropAction(); ++c.servo_action_id_;
    c.servo_action_attempted_=false; c.servo_action_slot_=0;
    c.servo_action_result_=DropActionResult::kPending;
    c.drop_complete=false; c.servo_complete.data=false;
    c.drop_recovery_started_=false;
}
int main(int argc, char** argv) {
    assert(argc==2); const std::string name=argv[1];
    auto t=std::make_shared<Transport>(); LLController c; c.servo_client.impl=t; c.servo_action_client_.impl=t;
    if (name=="delay_fast_ack" || name=="delay_failure" || name=="delay_hung") {
        c.drop_recover_without_ack_=true;c.drop_recovery_delay_sec_=0.5;
        t->delay_ms=0;t->success=(name!="delay_failure");t->gated=(name=="delay_hung");
        assert(c.executeDropAction(1)==DropActionResult::kPending);
        waitStarted(t);c.pollDropAction();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));c.pollDropAction();
        assert(!c.drop_recovery_started_&&!c.drop_complete&&!c.servo_complete.data);
        assert(c.invocation_events==0);
        if(!t->gated)assert(!c.servo_action_pending_);
        const auto until=Clock::now()+std::chrono::seconds(2);
        while(!c.drop_recovery_started_&&Clock::now()<until){
            c.pollDropAction();std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(c.drop_recovery_started_&&c.drop_complete&&!c.servo_complete.data);
        assert(c.servoWorkerForSlot(1).invocationElapsedSec(c.servo_action_id_,1)>=0.5);
        assert(c.invocation_events==1&&t->calls==1);
        c.pollDropAction();assert(c.invocation_events==1);
        if(t->gated)release(t);finish(c);
    } else if (name=="no_ack_recovery") {
        c.drop_recover_without_ack_=true;t->gated=true;
        assert(c.executeDropAction(1)==DropActionResult::kPending);
        waitStarted(t);c.pollDropAction();
        assert(c.drop_recovery_started_&&c.drop_complete&&!c.servo_complete.data);
        assert(c.invocation_events==1);
        for(int i=0;i<10;++i){c.pollDropAction();c.executeDropAction(1);}
        assert(t->calls==1&&c.invocation_events==1);
        // A hung first RPC cannot prevent sending the second slot.
        resetContext(c);c.servo_alignment_context_.payload_slot=2;
        auto second=std::make_shared<Transport>();second->gated=true;
        c.servo_action_client_.impl=second;
        assert(c.executeDropAction(2)==DropActionResult::kPending);
        waitStarted(second);c.pollDropAction();
        assert(c.drop_recovery_started_&&c.drop_complete&&!c.servo_complete.data);
        assert(c.invocation_events==2&&t->calls==1&&second->calls==1);
        release(t);release(second);finish(c);
    } else if (name=="no_ack_failure") {
        c.drop_recover_without_ack_=true;t->success=false;t->delay_ms=10;
        assert(c.executeDropAction(1)==DropActionResult::kPending);
        waitStarted(t);finish(c);
        assert(c.drop_recovery_started_&&c.drop_complete&&!c.servo_complete.data);
        assert(c.invocation_events==1&&t->calls==1);
    } else if (name=="no_ack_guards") {
        c.drop_recover_without_ack_=true;c.geometry_ready=false;
        assert(c.executeDropAction(1)==DropActionResult::kPending);
        assert(!c.drop_recovery_started_&&c.invocation_events==0&&t->calls==0);
        c.geometry_ready=true;c.permission_ready=false;
        assert(c.executeDropAction(1)==DropActionResult::kRejected);
        assert(!c.drop_recovery_started_&&c.invocation_events==0&&t->calls==0);
    } else if (name=="continuity") {
        const auto start=Clock::now();
        assert(c.executeDropAction(1)==DropActionResult::kPending);
        assert(!c.drop_complete && !c.servo_complete.data);
        waitStarted(t);
        assert(t->permission_epoch=="test" && t->permission_revision==18);
        int ticks=0; double largest_gap=0;
        auto previous=Clock::now(), next=previous;
        while (Clock::now()-start < std::chrono::milliseconds(1150)) {
            // Real submit/poll bodies run on the control clock; a transport
            // double replaces ROS. Each successful loop publishes a mock
            // setpoint. The RPC worker continues sleeping independently.
            c.timerPublicationPath();
            const auto now=Clock::now();
            largest_gap=std::max(largest_gap,std::chrono::duration<double>(now-previous).count());
            previous=now; ++ticks;
            const auto elapsed=now-start;
            if (elapsed<std::chrono::milliseconds(950)) {
                assert(!c.drop_complete && !c.servo_complete.data);
                assert(c.executeDropAction(1)==DropActionResult::kPending);
            }
            next+=std::chrono::milliseconds(50); std::this_thread::sleep_until(next);
        }
        finish(c);
        assert(t->calls==1 && ticks>=20 && c.mavros_point_cmd_pub.calls==ticks && largest_gap<.30);
        assert(c.drop_complete && c.servo_complete.data && c.drop_completed[0]);
        assert(c.executeDropAction(1)==DropActionResult::kSuccess && t->calls==1);
        std::cout << "mock_setpoints=" << ticks << " max_gap_s=" << largest_gap << " calls=" << t->calls << '\n';
    } else if (name=="failed" || name=="transport" || name=="exception") {
        t->delay_ms=20; t->success=false;
        t->transport_ok=(name!="transport"); t->throw_error=(name=="exception");
        assert(c.executeDropAction(1)==DropActionResult::kPending); finish(c);
        assert(!c.drop_complete && !c.servo_complete.data);
        for(int i=0;i<100;++i) assert(!dropActionSucceeded(c.executeDropAction(1)));
        resetContext(c); assert(c.executeDropAction(1)==DropActionResult::kRejected);
        assert(t->calls==1);
    } else if (name=="cancel" || name=="late" || name=="timeout") {
        t->gated=true; t->delay_ms=0;
        if(name=="timeout") c.servo_call_timeout_sec_=.03;
        assert(c.executeDropAction(1)==DropActionResult::kPending); waitStarted(t);
        if(name=="timeout") { std::this_thread::sleep_for(std::chrono::milliseconds(45)); c.pollDropAction(); }
        else if(name=="cancel") {
            auto state=std::make_shared<mavros_msgs::State>(); state->mode="POSCTL";
            c.externalLandingStateCallback(state);
        } else c.cancelDropAction();
        assert(!c.servo_action_pending_ && !c.drop_complete && !c.servo_complete.data);
        for(int i=0;i<20;++i) assert(!dropActionSucceeded(c.executeDropAction(1)));
        resetContext(c); assert(c.executeDropAction(1)==DropActionResult::kRejected);
        assert(c.executeDropAction(2)==DropActionResult::kRejected); // worker still occupied
        release(t); std::this_thread::sleep_for(std::chrono::milliseconds(25)); c.pollDropAction();
        assert(!c.drop_complete && !c.servo_complete.data && t->calls==1);
        assert(c.executeDropAction(1)==DropActionResult::kRejected);
        if(name=="late") {
            c.servo_alignment_context_.payload_slot=2;
            assert(c.executeDropAction(2)==DropActionResult::kPending); finish(c);
            assert(c.drop_complete && c.servo_complete.data && t->calls==2);
        }
    } else if (name=="identity") {
        t->gated=true; t->delay_ms=0;
        assert(c.executeDropAction(1)==DropActionResult::kPending); waitStarted(t);
        AsyncServo::Completion completion;
        assert(!c.async_servo_.poll(2,1,&completion));
        assert(!c.async_servo_.poll(1,2,&completion));
        assert(c.executeDropAction(2)==DropActionResult::kRejected);
        release(t); finish(c);
        assert(c.drop_complete && t->calls==1);
        assert(!c.async_servo_.poll(1,1,&completion)); // exactly one completion
    } else if (name=="shutdown") {
        t->gated=true; t->delay_ms=0;
        {
            LLController other; other.servo_client.impl=t; other.servo_action_client_.impl=t;
            assert(other.executeDropAction(1)==DropActionResult::kPending); waitStarted(t);
            const auto start=Clock::now(); other.async_servo_.shutdown();
            assert(std::chrono::duration<double>(Clock::now()-start).count()<.1);
            assert(!other.async_servo_.submit(2,2,10,[](int){return DropActionResult::kSuccess;}));
        } // job must not reference destroyed controller/worker
        release(t); std::this_thread::sleep_for(std::chrono::milliseconds(25)); assert(t->calls==1);
    } else if (name=="three") {
        t->delay_ms=10;
        for(int slot=1;slot<=3;++slot) {
            c.detect_point_counter=slot-1; c.servo_alignment_context_.payload_slot=slot;
            assert(c.executeDropAction(slot)==DropActionResult::kPending); finish(c);
            assert(c.drop_complete && c.drop_completed[slot-1]); resetContext(c);
        }
        for(int slot=1;slot<=3;++slot) assert(c.executeDropAction(slot)==DropActionResult::kRejected);
        assert(t->calls==3);
    } else if (name=="not_started" || name=="wrong_proof_action" || name=="wrong_proof_slot" || name=="nonterminal") {
        t->delay_ms=10; t->success=false; t->fact=1;
        if(name=="wrong_proof_action") t->echo_request_offset=1;
        if(name=="wrong_proof_slot") t->echo_slot_override=2;
        if(name=="nonterminal") t->terminal=false;
        assert(c.executeDropAction(1)==DropActionResult::kPending); finish(c);
        assert(t->last_request_id==1 && t->last_decision_seq==900); // not the same namespace
        assert(!c.drop_complete && !c.servo_complete.data && t->calls==1);
        assert(!dropActionSucceeded(c.executeDropAction(1)) && t->calls==1);
        if(name=="not_started") {
            assert(c.servo_action_result_==DropActionResult::kNotStarted);
            assert(!c.async_servo_.submit(1,1,10,[](int){return DropActionResult::kSuccess;}));
            resetContext(c); ++c.servo_alignment_decision_seq_; ++c.servo_alignment_context_.decision_seq;
            t->success=true; t->fact=3;
            assert(c.executeDropAction(1)==DropActionResult::kPending); finish(c);
            assert(c.drop_complete && c.servo_complete.data && t->calls==2);
            assert(!c.async_servo_.releaseNotStarted(1,1)); // old proof cannot unlock new owner
            resetContext(c); assert(c.executeDropAction(1)==DropActionResult::kRejected);
        } else {
            resetContext(c); assert(c.executeDropAction(1)==DropActionResult::kRejected && t->calls==1);
        }
    } else if (name=="inactive_success" || name=="inactive_not_started") {
        t->gated=true; t->delay_ms=0;
        t->success=(name=="inactive_success"); t->fact=t->success ? 3 : 1;
        assert(c.executeDropAction(1)==DropActionResult::kPending); waitStarted(t);
        auto inactive=std::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
        inactive->active=false;
        c.servoAlignmentContextCallback(inactive); // Bridge closes context before RPC reply
        assert(!c.have_servo_alignment_context_ && c.servo_action_pending_);
        release(t); finish(c);
        assert(c.drop_complete==t->success && c.servo_complete.data==t->success);
        if(!t->success) {
            resetContext(c); ++c.servo_alignment_decision_seq_; ++c.servo_alignment_context_.decision_seq;
            c.have_servo_alignment_context_=true; t->success=true; t->fact=3;
            assert(c.executeDropAction(1)==DropActionResult::kPending); finish(c);
            assert(c.drop_complete && t->calls==2);
        }
    } else if (name=="new_align_before_proof") {
        t->gated=true; t->delay_ms=0; t->success=false; t->fact=1;
        assert(c.executeDropAction(1)==DropActionResult::kPending); waitStarted(t);
        auto inactive=std::make_shared<uav_vision::AlignmentTargetContext>(c.servo_alignment_context_);
        inactive->active=false; c.servoAlignmentContextCallback(inactive);
        resetContext(c); ++c.servo_alignment_decision_seq_; ++c.servo_alignment_context_.decision_seq;
        c.have_servo_alignment_context_=true; release(t);
        // The next submission captures its separate transport, preserving the
        // old call's NOT_STARTED response until it returns.
        auto fresh=std::make_shared<Transport>(); fresh->delay_ms=0;
        c.servo_action_client_.impl=fresh;
        const auto end=Clock::now()+std::chrono::seconds(1);
        while(c.executeDropAction(1)==DropActionResult::kRejected && Clock::now()<end)
            std::this_thread::yield();
        assert(c.servo_action_pending_); finish(c);
        assert(c.drop_complete && t->calls==1 && fresh->calls==1);
    } else if (name=="cancel_proof" || name=="timeout_proof" || name=="cancel_unknown") {
        // New ALIGN cancels the old job before its fenced RPC finishes. No poll
        // will run for that generation; submit must safely reap the proof.
        auto gate=std::make_shared<Transport>(); gate->gated=true; gate->delay_ms=0;
        gate->success=false; gate->fact=(name=="cancel_unknown" ? 2 : 1);
        AsyncServo worker;
        assert(worker.submit(1,1,name=="timeout_proof" ? .01 : 10,[gate](int){
            ros::ServiceClient client; client.impl=gate; ServoAction srv;
            srv.request.request_id=1; srv.request.payload_slot=1;
            bool ok=client.call(srv); return classifyServoAction(srv.request,ok,srv.response);
        }));
        waitStarted(gate); worker.cancel();
        if(name=="timeout_proof") std::this_thread::sleep_for(std::chrono::milliseconds(20));
        release(gate);
        const auto end=Clock::now()+std::chrono::seconds(1);
        // No poll for the old generation: only submit can reap its proof.
        if(name=="cancel_unknown") {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            assert(!worker.submit(2,1,10,[](int){return DropActionResult::kSuccess;}));
        } else {
            bool submitted=false;
            while(!submitted && Clock::now()<end) {
                submitted=worker.submit(2,1,10,[](int){return DropActionResult::kSuccess;});
                std::this_thread::yield();
            }
            assert(submitted && gate->calls==1);
            assert(!worker.releaseNotStarted(1,1));
            assert(!worker.submit(1,1,10,[](int){return DropActionResult::kSuccess;}));
        }
    } else if (name=="context_mismatch") {
        c.servo_alignment_context_.decision_seq=1;
        assert(c.executeDropAction(1)==DropActionResult::kRejected && t->calls==0);
        c.servo_alignment_context_.decision_seq=900;
        c.servo_alignment_context_.semantic_target_class="bridge";
        assert(c.executeDropAction(1)==DropActionResult::kRejected && t->calls==0);
    } else if (name=="legacy_false") {
        c.external_mission_mode_=false; t->success=false; t->delay_ms=10; t->fact=1;
        assert(c.executeDropAction(1)==DropActionResult::kPending); finish(c);
        resetContext(c); assert(c.executeDropAction(1)==DropActionResult::kRejected && t->calls==1);
    } else if (name=="geometry") {
        c.geometry_ready=false;
        assert(c.executeDropAction(1)==DropActionResult::kPending && t->calls==0);
        assert(!c.servo_action_attempted_);
        c.geometry_ready=true; t->delay_ms=10;
        assert(c.executeDropAction(1)==DropActionResult::kPending);
        waitStarted(t); c.geometry_ready=false; finish(c);
        assert(c.executeDropAction(1)==DropActionResult::kSuccess && t->calls==1);
    } else if (name=="invalid") {
        c.control_ready=false;
        assert(c.executeDropAction(1)==DropActionResult::kRejected && t->calls==0);
        c.control_ready=true;
        assert(c.executeDropAction(0)==DropActionResult::kInvalidServoId);
        assert(c.executeDropAction(4)==DropActionResult::kInvalidServoId);
        c.servo_call_timeout_sec_=0;
        assert(c.executeDropAction(1)==DropActionResult::kRejected && t->calls==0);
    } else assert(false);
}
"""


class AsyncServoTest(unittest.TestCase):
    def test_delay_even_when_ack_returns_early(self): self.run_case("delay_fast_ack")
    def test_delay_failure_still_recovers(self): self.run_case("delay_failure")
    def test_delay_hung_call_does_not_wait_ack(self): self.run_case("delay_hung")
    def test_no_ack_recovers_once_and_old_hung_rpc_cannot_block_next_slot(self): self.run_case("no_ack_recovery")
    def test_no_ack_failure_still_keeps_recovery_without_synthesizing_ack(self): self.run_case("no_ack_failure")
    def test_no_ack_preserves_geometry_and_permission_before_invocation(self): self.run_case("no_ack_guards")
    @classmethod
    def setUpClass(cls):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text()
        methods = '\n'.join(method(source, signature) for signature in (
            'AsyncServo& LLController::servoWorkerForSlot(',
            'void LLController::beginDropRecoveryWithoutAck(',
            'DropActionResult LLController::executeDropAction(',
            'void LLController::pollDropAction(',
            'void LLController::cancelDropAction(',
            'void LLController::externalLandingStateCallback(',
            'void LLController::servoAlignmentContextCallback('))
        # Execute the actual timer's asynchronous prefix and publication tail;
        # geometry/navigation branches are covered by the other controller tests.
        timer = method(source, 'void LLController::cmdCallback(')
        prefix = timer[timer.index('{') + 1:timer.index('    double phase_lead')]
        publish_start = timer.index('    mavros_point_cmd.header.stamp = ros::Time::now();')
        publish_end = timer.index('    // std::cout', publish_start)
        methods += '\nvoid LLController::timerPublicationPath() {\n' + prefix + timer[publish_start:publish_end] + '\n}'
        cls.folder = tempfile.TemporaryDirectory(prefix='async-servo-')
        root = Path(cls.folder.name)
        (root / 'test.cpp').write_text(PROGRAM.replace('PRODUCTION_METHODS', methods))
        cls.binary = root / 'test'
        subprocess.run(['g++', '-std=c++14', '-O1', '-pthread', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-I', str(PACKAGE / 'include'),
                        str(root / 'test.cpp'), '-o', str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def test_geometry_wait_never_calls_servo_and_cannot_cancel_started_rpc(self): self.run_case("geometry")

    def test_mock_1s_servo_preserves_control_loop(self): self.run_case('continuity')
    def test_failed_ack_never_repeats_slot(self): self.run_case('failed')
    def test_transport_failure_never_repeats_slot(self): self.run_case('transport')
    def test_throwing_transport_is_failure(self): self.run_case('exception')
    def test_takeover_invalidates_late_ack(self): self.run_case('cancel')
    def test_old_action_result_cannot_complete_new_slot(self): self.run_case('late')
    def test_timeout_locks_slot_and_does_not_wait(self): self.run_case('timeout')
    def test_completion_matches_action_and_slot_and_is_consumed_once(self): self.run_case('identity')
    def test_shutdown_with_blocked_call_is_nonblocking_and_owns_lifetime(self): self.run_case('shutdown')
    def test_all_three_slots_work_once(self): self.run_case('three')
    def test_invalid_admission_never_calls_transport(self): self.run_case('invalid')

    def test_not_started_unlocks_only_for_new_align(self): self.run_case('not_started')
    def test_wrong_request_id_cannot_unlock(self): self.run_case('wrong_proof_action')
    def test_wrong_slot_cannot_unlock(self): self.run_case('wrong_proof_slot')
    def test_nonterminal_proof_cannot_unlock(self): self.run_case('nonterminal')
    def test_inactive_context_before_success_rpc_preserves_completion(self): self.run_case('inactive_success')
    def test_inactive_context_before_not_started_rpc_allows_new_align(self): self.run_case('inactive_not_started')
    def test_controller_new_align_before_rpc_proof_can_retry(self): self.run_case('new_align_before_proof')
    def test_new_align_before_old_proof_reaps_without_control_poll(self): self.run_case('cancel_proof')
    def test_late_terminal_not_started_can_prove_zero_raw_calls(self): self.run_case('timeout_proof')
    def test_cancelled_unknown_cannot_unlock_on_new_submit(self): self.run_case('cancel_unknown')
    def test_wrong_mission_context_cannot_start_rpc(self): self.run_case('context_mismatch')
    def test_legacy_bool_false_is_not_not_started(self): self.run_case('legacy_false')

    def run_case(self, name):
        result = subprocess.run([str(self.binary), name], check=True, capture_output=True,
                                text=True, timeout=5)
        if result.stdout: print(result.stdout.strip())


if __name__ == '__main__': unittest.main(verbosity=2)
