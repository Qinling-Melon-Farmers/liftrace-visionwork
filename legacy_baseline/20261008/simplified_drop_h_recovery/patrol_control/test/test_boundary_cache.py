"""Compile production boundary methods/branches; fake only ROS transport.

No master, ROS node, simulation, or hardware is started. The extracted C++
functions and NearWallAlignFence header are the production implementation.
"""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


def production_block(source, signature):
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
    raise AssertionError('Unclosed production block: ' + signature)


PROGRAM = r'''
#include "patrol_control/near_wall_align.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <cassert>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_ERROR_THROTTLE(...) ((void)0)
namespace geometry_msgs {
struct PoseStamped {
    struct { struct { double x=0, y=0, z=0; } position;
             double orientation=0; } pose;
};
}
namespace tf { double getYaw(double yaw) { return yaw; } }
namespace ros {
struct WallTimerEvent {};
struct WallDuration { double value; explicit WallDuration(double v):value(v){} };
struct TimerState { bool active=false; std::function<void()> callback; };
struct WallTimer {
    std::shared_ptr<TimerState> state = std::make_shared<TimerState>();
    void stop() { state->active=false; }
    void fire() { if (state->active) state->callback(); }
};
using Timer = WallTimer;
struct NodeHandle {
    std::map<std::string, XmlRpc::XmlRpcValue> values;
    // Transport double models subscribed per-key cache, including negative
    // entries; values edits simulate incoming paramUpdate notifications.
    std::set<std::string> subscribed;
    int reads=0, cache_misses=0, timers=0;
    double period=0;
    bool getParamCached(const std::string& name, XmlRpc::XmlRpcValue& out) {
        ++reads;
        if (subscribed.insert(name).second) ++cache_misses;
        auto it=values.find(name);
        if (it==values.end()) return false;
        out=it->second; return true;
    }
    bool getParamCached(const std::string& name, bool& out) {
        XmlRpc::XmlRpcValue value;
        if (!getParamCached(name,value) || value.getType()!=XmlRpc::XmlRpcValue::TypeBoolean)
            return false;
        out=static_cast<bool>(value);return true;
    }
    bool getParamCached(const std::string& name, double& out) {
        XmlRpc::XmlRpcValue value;
        if (!getParamCached(name,value)) return false;
        if (value.getType()==XmlRpc::XmlRpcValue::TypeDouble) out=static_cast<double>(value);
        else if (value.getType()==XmlRpc::XmlRpcValue::TypeInt) out=static_cast<int>(value);
        else return false;
        return true;
    }
    template<typename T> WallTimer createWallTimer(
        WallDuration duration, void(T::*callback)(const WallTimerEvent&), T* owner) {
        ++timers; period=duration.value;
        WallTimer timer; timer.state->active=true;
        timer.state->callback=[owner,callback]{(owner->*callback)(WallTimerEvent());};
        return timer;
    }
};
}
namespace patrol_control {
PRODUCTION_HELPERS
struct LLController {
    ros::NodeHandle nh_;
    ros::Timer cmd_timer;
    ros::WallTimer near_wall_align_refresh_timer_;
    NearWallAlignFence near_wall_align_fence_;
    bool external_mission_mode_=true;
    enum { Aligning, Run_point } Drone_mode=Aligning;
    geometry_msgs::PoseStamped mavros_point_cmd, uav_pose;
    double current_yaw=0, interpolated_yaw=0;
    bool params_loaded=false;
    struct { void shutdown(){} } async_servo_;
    ~LLController();
    void load_params(){params_loaded=true;}
    void refreshNearWallAlignFence(const ros::WallTimerEvent&);
    void prime() { PRODUCTION_PRIME }
    void align() { PRODUCTION_ALIGN }
    bool waypointRelease() {
        bool should_drop=true;
PRODUCTION_WAYPOINT_RELEASE
        return should_drop;
    }
    bool crossRelease() {
        bool should_drop=true;
PRODUCTION_CROSS_RELEASE
        return should_drop;
    }
};
PRODUCTION_REFRESH
PRODUCTION_DESTRUCTOR
}
using patrol_control::LLController;
const std::string root="/navigation/mission_manager/high_view_full/boundary_policy/";
XmlRpc::XmlRpcValue bounds(double a=-4.8,double b=4.8,double c=-.5,double d=7.4) {
    XmlRpc::XmlRpcValue value;value.setSize(4);
    value[0]=a;value[1]=b;value[2]=c;value[3]=d;return value;
}
void configure(LLController& c) {
    c.nh_.values[root+"enabled"]=true;
    c.nh_.values[root+"bounds"]=bounds();
    c.uav_pose.pose.position.x=0;c.uav_pose.pose.position.y=2;c.uav_pose.pose.position.z=1;
    c.mavros_point_cmd=c.uav_pose;
}
bool close(double a,double b){return std::abs(a-b)<1e-9;}
void assertHeld(LLController& c) {
    c.mavros_point_cmd.pose.position.x=-20;
    c.align();
    assert(close(c.mavros_point_cmd.pose.position.x,c.uav_pose.pose.position.x));
    assert(close(c.mavros_point_cmd.pose.position.y,c.uav_pose.pose.position.y));
    assert(!c.waypointRelease() && !c.crossRelease());
}
int main(int argc,char** argv) {
    assert(argc==2);const std::string test=argv[1];
    LLController c;configure(c);
    if(test=="startup") {
        c.prime();assert(c.params_loaded && c.nh_.timers==1 && c.nh_.period==1.0);
        assert(c.nh_.reads==5 && c.nh_.cache_misses==5 && c.near_wall_align_fence_.wellFormed());
        c.mavros_point_cmd.pose.position.x=-20;c.align();
        assert(close(c.mavros_point_cmd.pose.position.x,-4.8+c.near_wall_align_fence_.margin(0)));
        assert(c.waypointRelease() && c.crossRelease());
    } else if(test=="hot_path") {
        c.prime();const int reads=c.nh_.reads;
        for(int i=0;i<10000;++i){
            c.mavros_point_cmd.pose.position.x=-20;c.align();
            assert(c.waypointRelease() && c.crossRelease());
        }
        assert(c.nh_.reads==reads);
        c.nh_.values[root+"bounds"]=bounds(1,8,-.5,7.4);
        // Parameter edits take effect at the refresh, never via control reads.
        assert(c.waypointRelease());c.near_wall_align_refresh_timer_.fire();
        assert(c.nh_.reads==reads+5 && !c.waypointRelease() && !c.crossRelease());
    } else if(test=="dynamic_geometry") {
        c.prime();c.nh_.values[root+"bounds"]=bounds(-3,3,0,6);
        c.nh_.values[root+"guard_side_m"]=.8;
        c.nh_.values[root+"tracking_reserve_m"]=.08;
        c.nh_.values[root+"yaw_budget_deg"]=20.;
        c.near_wall_align_refresh_timer_.fire();
        const auto& f=c.near_wall_align_fence_;
        assert(f.enabled && f.wellFormed() && f.bounds[0]==-3 && f.side_m==.8);
        assert(f.tracking_reserve_m==.08 && f.yaw_budget_deg==20);
        c.current_yaw=.5;c.interpolated_yaw=.7;
        c.mavros_point_cmd.pose.position.x=9;c.mavros_point_cmd.pose.position.y=-9;
        const auto expected=f.clamp(9,-9,.5,.7);c.align();
        assert(close(c.mavros_point_cmd.pose.position.x,expected.first));
        assert(close(c.mavros_point_cmd.pose.position.y,expected.second));
        c.uav_pose=c.mavros_point_cmd;c.uav_pose.pose.orientation=.7;
        assert(c.waypointRelease() && c.crossRelease());
    } else if(test=="enable_disable") {
        c.nh_.values[root+"enabled"]=false;c.prime();assert(c.nh_.reads==5 && c.nh_.cache_misses==5);
        c.uav_pose.pose.position.x=20;c.mavros_point_cmd=c.uav_pose;
        c.align();assert(c.mavros_point_cmd.pose.position.x==20 && c.waypointRelease());
        c.nh_.values[root+"enabled"]=true;c.near_wall_align_refresh_timer_.fire();
        assert(!c.waypointRelease() && !c.crossRelease());c.align();
        assert(c.mavros_point_cmd.pose.position.x<5);
        c.nh_.values[root+"enabled"]=false;c.near_wall_align_refresh_timer_.fire();
        c.mavros_point_cmd=c.uav_pose;c.align();assert(c.mavros_point_cmd.pose.position.x==20);
        assert(c.waypointRelease() && c.crossRelease());
    } else if(test=="missing_config") {
        c.nh_.values.clear();c.prime();assert(!c.near_wall_align_fence_.enabled);
        const int reads=c.nh_.reads;
        for(int i=0;i<1000;++i){c.align();assert(c.waypointRelease());}
        assert(c.nh_.reads==reads && reads==5 && c.nh_.cache_misses==5);
        configure(c);c.near_wall_align_refresh_timer_.fire();assert(c.near_wall_align_fence_.enabled);
        c.nh_.values.erase(root+"bounds");c.near_wall_align_refresh_timer_.fire();assertHeld(c);
        c.nh_.values[root+"bounds"]=bounds();c.near_wall_align_refresh_timer_.fire();
        assert(c.waypointRelease() && c.near_wall_align_fence_.wellFormed());
        c.nh_.values.erase(root+"enabled");c.near_wall_align_refresh_timer_.fire();
        assert(!c.near_wall_align_fence_.enabled && c.waypointRelease());
    } else if(test=="invalid_bounds") {
        c.prime();
        for(auto bad:{bounds(5,-5),bounds(0,.5),bounds(-4,4,0,.5),
                      bounds(std::numeric_limits<double>::quiet_NaN()),
                      bounds(-4,4,0,std::numeric_limits<double>::infinity())}) {
            c.nh_.values[root+"bounds"]=bad;c.near_wall_align_refresh_timer_.fire();assertHeld(c);
        }
        XmlRpc::XmlRpcValue bad;bad.setSize(3);
        c.nh_.values[root+"bounds"]=bad;c.near_wall_align_refresh_timer_.fire();assertHeld(c);
        bad=bounds();bad[0]=std::string("bad");
        c.nh_.values[root+"bounds"]=bad;c.near_wall_align_refresh_timer_.fire();assertHeld(c);
        c.nh_.values[root+"bounds"]=42;c.near_wall_align_refresh_timer_.fire();assertHeld(c);
        c.nh_.values[root+"bounds"]=bounds();c.near_wall_align_refresh_timer_.fire();
        assert(c.near_wall_align_fence_.wellFormed() && c.waypointRelease());
    } else if(test=="invalid_geometry") {
        c.prime();
        const std::pair<std::string,double> invalid[]={
            {"guard_side_m",0},{"guard_side_m",1.1},
            {"tracking_reserve_m",-.01},{"tracking_reserve_m",.11},
            {"yaw_budget_deg",-1},{"yaw_budget_deg",46},
            {"yaw_budget_deg",std::numeric_limits<double>::quiet_NaN()}};
        for(const auto& item:invalid) {
            c.nh_.values[root+item.first]=item.second;
            c.near_wall_align_refresh_timer_.fire();assertHeld(c);
            c.nh_.values.erase(root+item.first);
            c.near_wall_align_refresh_timer_.fire();assert(c.near_wall_align_fence_.wellFormed());
        }
    } else if(test=="defaults_and_ints") {
        XmlRpc::XmlRpcValue ints;ints.setSize(4);ints[0]=-4;ints[1]=4;ints[2]=0;ints[3]=7;
        c.nh_.values[root+"bounds"]=ints;c.prime();assert(c.near_wall_align_fence_.wellFormed());
        c.nh_.values[root+"guard_side_m"]=.8;c.near_wall_align_refresh_timer_.fire();
        assert(c.near_wall_align_fence_.side_m==.8);
        c.nh_.values.erase(root+"guard_side_m");c.near_wall_align_refresh_timer_.fire();
        assert(c.near_wall_align_fence_.side_m==.55);
        assert(c.near_wall_align_fence_.tracking_reserve_m==.03 && c.near_wall_align_fence_.yaw_budget_deg==10);
    } else if(test=="startup_invalid_legacy") {
        c.nh_.values.erase(root+"bounds");c.prime();assertHeld(c);
        LLController legacy;configure(legacy);legacy.external_mission_mode_=false;
        legacy.prime();assert(legacy.nh_.reads==0 && legacy.nh_.timers==0);
        legacy.uav_pose.pose.position.x=20;legacy.mavros_point_cmd=legacy.uav_pose;
        legacy.align();assert(legacy.mavros_point_cmd.pose.position.x==20 && legacy.waypointRelease());
        c.Drone_mode=LLController::Run_point;c.mavros_point_cmd.pose.position.x=30;
        c.align();assert(c.mavros_point_cmd.pose.position.x==30);
    } else if(test=="warm_all_states") {
        for(int state=0;state<3;++state) {
            LLController warm;
            if(state==1) {configure(warm);warm.nh_.values[root+"enabled"]=false;}
            if(state==2) {configure(warm);warm.nh_.values.erase(root+"bounds");}
            warm.prime();assert(warm.nh_.reads==5 && warm.nh_.cache_misses==5);
            for(int tick=0;tick<1000;++tick) warm.near_wall_align_refresh_timer_.fire();
            assert(warm.nh_.reads==5005 && warm.nh_.cache_misses==5);
            configure(warm);warm.near_wall_align_refresh_timer_.fire();
            assert(warm.near_wall_align_fence_.wellFormed() && warm.waypointRelease());
            assert(warm.nh_.cache_misses==5);
        }
    } else if(test=="wrong_optional_types") {
        c.prime();
        c.nh_.values[root+"guard_side_m"]=.8;
        c.nh_.values[root+"tracking_reserve_m"]=.09;
        c.nh_.values[root+"yaw_budget_deg"]=30.;
        c.near_wall_align_refresh_timer_.fire();
        c.nh_.values[root+"guard_side_m"]=std::string("bad");
        c.nh_.values[root+"tracking_reserve_m"]=true;
        XmlRpc::XmlRpcValue invalid;c.nh_.values[root+"yaw_budget_deg"]=invalid;
        c.near_wall_align_refresh_timer_.fire();
        assert(c.near_wall_align_fence_.side_m==.55);
        assert(c.near_wall_align_fence_.tracking_reserve_m==.03);
        assert(c.near_wall_align_fence_.yaw_budget_deg==10. && c.near_wall_align_fence_.wellFormed());
        c.nh_.values[root+"enabled"]=std::string("true");
        c.near_wall_align_refresh_timer_.fire();
        assert(!c.near_wall_align_fence_.enabled && c.waypointRelease());
        assert(c.nh_.cache_misses==5);
    } else if(test=="instances_teardown") {
        ros::WallTimer observer;
        {
            LLController other;configure(other);other.prime();observer=other.near_wall_align_refresh_timer_;
            c.prime();c.nh_.values[root+"bounds"]=bounds(1,8,0,6);
            c.near_wall_align_refresh_timer_.fire();
            assert(c.near_wall_align_fence_.bounds[0]==1 && other.near_wall_align_fence_.bounds[0]==-4.8);
            assert(observer.state->active);
        }
        assert(!observer.state->active);observer.fire();
    } else return 2;
}
'''


class BoundaryCacheTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        helpers = '\n'.join(production_block(source, signature) for signature in (
            'bool readNumber(', 'NearWallAlignFence loadNearWallAlignFence(',
            'bool nearWallAlignReleaseAllowed('))
        init = production_block(source, 'void LLController::initializeNode()')
        prime = init[init.index('    load_params();'):init.index('    std::string height_replan_topic;')]
        cmd = production_block(source, 'void LLController::cmdCallback(')
        align = production_block(cmd, 'if (external_mission_mode_ && Drone_mode == Aligning)')
        release_branches = []
        for name in ('bool LLController::WayPointDetectDone()', 'bool LLController::CrossDetectionDone()'):
            method = production_block(source, name)
            start = method.index('            if (external_mission_mode_ &&\n                !nearWallAlignReleaseAllowed(')
            end = method.index('should_drop = false;', start) + len('should_drop = false;')
            release_branches.append(method[start:end])
        for token, replacement in (
            ('PRODUCTION_HELPERS', helpers), ('PRODUCTION_PRIME', prime),
            ('PRODUCTION_ALIGN', align), ('PRODUCTION_WAYPOINT_RELEASE', release_branches[0]),
            ('PRODUCTION_CROSS_RELEASE', release_branches[1]),
            ('PRODUCTION_REFRESH', production_block(source, 'void LLController::refreshNearWallAlignFence(')),
            ('PRODUCTION_DESTRUCTOR', production_block(source, 'LLController::~LLController()'))):
            if token == 'PRODUCTION_HELPERS':
                cls.program = PROGRAM
            cls.program = cls.program.replace(token, replacement)
        cls.folder = tempfile.TemporaryDirectory(prefix='boundary-cache-', dir=os.environ.get('TEST_ARTIFACT_DIR'))
        folder = Path(cls.folder.name)
        cpp = folder / 'production_boundary_test.cpp'
        cpp.write_text(cls.program, encoding='utf-8')
        cls.binary = folder / 'production_boundary_test'
        xmlrpc_flags = shlex.split(subprocess.check_output(
            ['pkg-config', '--cflags', '--libs', 'xmlrpcpp'], text=True))
        subprocess.run(['g++', '-std=c++14', '-O0', '-Wall', '-Wextra',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(PACKAGE / 'include'), str(cpp),
                        '-o', str(cls.binary)] + xmlrpc_flags, check=True)
        artifact = os.environ.get('BOUNDARY_CPP_ARTIFACT')
        if artifact:
            Path(artifact).write_text(cls.program, encoding='utf-8')

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def run_case(self, case):
        subprocess.run([str(self.binary), case], check=True)

    def test_startup_loads_validated_snapshot(self): self.run_case('startup')
    def test_10000_align_and_release_checks_do_not_read_parameters(self): self.run_case('hot_path')
    def test_runtime_bounds_margin_and_yaw_updates(self): self.run_case('dynamic_geometry')
    def test_runtime_enable_disable(self): self.run_case('enable_disable')
    def test_missing_config_and_runtime_creation_deletion(self): self.run_case('missing_config')
    def test_invalid_bounds_hold_and_reject_until_repaired(self): self.run_case('invalid_bounds')
    def test_invalid_geometry_replaces_previous_valid_cache(self): self.run_case('invalid_geometry')
    def test_numeric_integer_bounds_and_deleted_optional_defaults(self): self.run_case('defaults_and_ints')
    def test_invalid_startup_legacy_and_non_align_behavior(self): self.run_case('startup_invalid_legacy')
    def test_all_keys_primed_for_absent_disabled_and_invalid_startup(self): self.run_case('warm_all_states')
    def test_cached_wrong_optional_types_use_fresh_defaults(self): self.run_case('wrong_optional_types')
    def test_instances_are_independent_and_timer_stops(self): self.run_case('instances_teardown')


if __name__ == '__main__':
    unittest.main(verbosity=2)
