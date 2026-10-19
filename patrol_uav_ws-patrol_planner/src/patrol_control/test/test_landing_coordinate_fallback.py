"""Execute production LAND ticks and coordinate fallback with continuous feedback.

Only transport/clock inputs are doubled. The coordinate, visual-alignment and
handoff gates come from current production C++ and its real helper headers.
No ROS master, simulator, board, or shared base-test modification is involved.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_external_landing_handoff import PROGRAM, PACKAGE, production_method
from test_landing_motion_settlement import FEEDBACK


MAIN = r'''
bool close(double a, double b) { return std::abs(a-b)<1e-9; }
void pose(LLController& c, double t, double x=.15, double y=0,
          double z=1.2, double speed=0, bool mark=false,
          double visual_x=4, double visual_y=5) {
    ros::clock=t; state(c);
    c.uav_pose.pose.position={x,y,z};
    c.motion_odom_.pose.pose.position=c.uav_pose.pose.position;
    c.motion_odom_.header.stamp=ros::Time(t);
    c.motion_odom_receipt_=ros::Time(t);
    c.feedback_velocity_=Eigen::Vector3d(speed,0,0);
    c.feedback_valid_=true; c.motion_time_pending_=false;
    c.have_land_mark=mark; c.external_landing_new_mark_=mark;
    if (mark) {
        c.land_mark_point.pose.position={visual_x,visual_y,0};
        c.external_landing_last_mark_stamp_=ros::Time(t);
        c.external_landing_last_mark_receipt_=ros::Time(t);
    }
}
void landAtOrigin(LLController& c, double t, unsigned int seq=42) {
    ros::clock=t; state(c);
    auto msg=std::make_shared<patrol_control::MissionCommand>();
    msg->command=patrol_control::MissionCommand::LAND;
    msg->target_class="coordinate-test"; msg->goal.header.seq=seq;
    msg->goal.header.stamp=ros::Time(t); msg->header.stamp=ros::Time(t);
    msg->goal.header.frame_id="camera_init";
    msg->goal.pose.position={0,0,1.2};
    c.missionCommandCallback(msg);
    assert(c.external_landing_active_ && c.Drone_mode==Land);
}
LLController setup(bool enabled=true, const std::string& mode="POSCTL") {
    ros::clock=100;
    LLController c;
    c.external_landing_handoff_mode_=mode;
    c.external_landing_alignment_mode_="vh";
    c.external_landing_capture_height_=1.2;
    c.land_height=.35; c.external_landing_auto_land_height_=.50;
    c.landing_coordinate_fallback_config_.enabled=enabled;
    c.landing_settle_config_.max_sample_gap_sec=.20;
    c.landing_settle_config_.height_tolerance_m=.05;
    c.landing_settle_config_.stable_duration_sec=.15;
    c.landing_handoff_window_=LandingHandoffStabilityWindow(c.landing_settle_config_);
    c.landing_capture_config_=c.landing_settle_config_;
    c.landing_capture_config_.height_tolerance_m=.10;
    c.landing_capture_window_=LandingHandoffStabilityWindow(c.landing_capture_config_);
    c.uav_pose.pose.position={.15,0,1.2};
    landAtOrigin(c,100);
    return c;
}
void tick(LLController& c, double t, double x=.15, double y=0,
          double z=1.2, double speed=0, bool mark=false,
          double visual_x=4, double visual_y=5) {
    pose(c,t,x,y,z,speed,mark,visual_x,visual_y);
    c.externalLandingTick();
}
void holding(const LLController& c) {
    assert(c.external_landing_active_);
    assert(!c.external_landing_alignment_complete_);
    assert(close(c.patrol_cmd.pose.position.z,1.2));
    assert(c.set_mode_client.calls==0);
}
void origin(const LLController& c) {
    assert(close(c.patrol_cmd.pose.position.x,0));
    assert(close(c.patrol_cmd.pose.position.y,0));
}
void waitForFallback(LLController& c, bool visual=false) {
    for(int i=0;i<=300;++i) {
        tick(c,100+i*.05,.15,0,1.2,0,visual);
        holding(c);
        assert(c.landing_coordinate_fallback_.active()==(i==300));
    }
    origin(c);
}
void settle(LLController& c, double start, double x=.079, double speed=.10) {
    for(int i=0;i<=6;++i) {
        // 0.301 s avoids binary rounding hiding the exact 0.30 s dwell.
        const double elapsed=(i==6 ? .301 : i*.05);
        tick(c,start+elapsed,x,0,1.2,speed);
        assert(c.external_landing_alignment_complete_==(i==6));
        if(i<6) holding(c);
    }
    origin(c); assert(close(c.patrol_cmd.pose.position.z,.35));
    assert(c.set_mode_client.calls==0);
}
int main(int argc,char** argv) {
    assert(argc==2); const std::string name=argv[1];
    auto c=setup();
    if(name=="no_timer_before_xy_or_height_arrival") {
        for(int i=0;i<=400;++i) {
            const bool outside_xy=i<200;
            tick(c,100+i*.05,outside_xy ? .201 : .15,0,outside_xy ? 1.2 : 1.351);
            holding(c);
            assert(!c.landing_coordinate_fallback_.active());
            assert(close(c.landing_coordinate_fallback_.waited(ros::clock),0));
        }
        tick(c,121); holding(c);
        tick(c,135.999); holding(c); assert(!c.landing_coordinate_fallback_.active());
        tick(c,136); holding(c); assert(c.landing_coordinate_fallback_.active());
    } else if(name=="full_fifteen_seconds_and_return_to_preset_not_visual") {
        for(int i=0;i<300;++i) {
            tick(c,100+i*.05,.15,0,1.2,0,true,4,5);
            holding(c); assert(!c.landing_coordinate_fallback_.active());
            assert(close(c.patrol_cmd.pose.position.x,4));
            assert(close(c.patrol_cmd.pose.position.y,5));
        }
        tick(c,114.999,.15,0,1.2,0,true,4,5); holding(c);
        assert(!c.landing_coordinate_fallback_.active());
        tick(c,115,.15,0,1.2,0,true,4,5); holding(c); origin(c);
        assert(c.landing_coordinate_fallback_.active());
    } else if(name=="xy_speed_height_and_point_three_seconds_required") {
        waitForFallback(c);
        // Each independent violation must prevent the stable window accruing.
        for(int mode=0;mode<3;++mode) {
            for(int i=0;i<12;++i) {
                tick(c,115.05+(mode*12+i)*.05,
                     mode==0 ? .080001 : .079,0,
                     mode==2 ? 1.350001 : 1.2,mode==1 ? .100001 : .10);
                holding(c); origin(c);
            }
        }
        settle(c,117,.08,.10); // Exactly the configured XY/speed boundary.
    } else if(name=="stability_dwell_not_shortened_to_point_two_nine_nine") {
        waitForFallback(c);
        for(int i=0;i<=5;++i) { tick(c,115.05+i*.05,.01); holding(c); }
        tick(c,115.349,.01); holding(c);
        tick(c,115.351,.01);
        assert(c.external_landing_alignment_complete_); origin(c);
    } else if(name=="worse_pose_or_odom_blocks_coordinate_readiness") {
        waitForFallback(c);
        for(int i=0;i<16;++i) {
            pose(c,115.05+i*.05,i<8 ? .09 : .01);
            c.motion_odom_.pose.pose.position.x=i<8 ? .01 : .09;
            c.externalLandingTick(); holding(c);
        }
        settle(c,116);
    } else if(name=="visual_success_before_timeout_has_priority") {
        tick(c,100); holding(c);
        for(int i=0;i<=21;++i) {
            tick(c,101+i*.05,.12,0,1.2,.02,true,.12,0);
        }
        assert(c.external_landing_alignment_complete_);
        assert(!c.landing_coordinate_fallback_.active());
        assert(close(c.external_landing_aligned_goal_.pose.position.x,.12));
        tick(c,116,.12,0,.8,0,false);
        assert(close(c.patrol_cmd.pose.position.x,.12));
        assert(close(c.patrol_cmd.pose.position.z,.35));
    } else if(name=="visual_success_at_deadline_precedes_fallback") {
        c.external_landing_alignment_mode_="non_vh";
        tick(c,100); tick(c,114.5);
        for(int i=0;i<10;++i)
            tick(c,114.55+i*.05,.12,0,1.2,0,true,.12,0);
        assert(c.external_landing_alignment_complete_);
        assert(!c.landing_coordinate_fallback_.active());
        assert(close(c.patrol_cmd.pose.position.x,.12));
    } else if(name=="active_ignores_new_visual_then_freezes_without_reclimb") {
        waitForFallback(c,true);
        // Fresh perfectly aligned visual marks cannot steal an active fallback.
        for(int i=1;i<=30;++i) {
            tick(c,115+i*.05,.15,0,1.2,0,true,.15,0);
            holding(c); origin(c);
        }
        settle(c,116.6);
        for(int i=0;i<12;++i) {
            tick(c,117+i*.05,0,0,1.0-i*.04,0,i%2==0,8,9);
            assert(c.external_landing_alignment_complete_); origin(c);
            assert(close(c.patrol_cmd.pose.position.z,.35));
            assert(c.set_mode_client.calls==0);
        }
    } else if(name=="stale_or_future_odometry_cannot_release") {
        waitForFallback(c);
        for(int mode=0;mode<2;++mode) {
            for(int i=0;i<12;++i) {
                const double t=115.05+(mode*12+i)*.05;
                pose(c,t,.01);
                c.motion_odom_.header.stamp=ros::Time(t+(mode==0 ? -.201 : .003));
                c.externalLandingTick(); holding(c); origin(c);
            }
        }
        settle(c,116.5);
    } else if(name=="duplicate_odom_cannot_accumulate_dwell") {
        waitForFallback(c);
        for(int i=0;i<=8;++i) {
            pose(c,115.05+i*.05,.01);
            c.motion_odom_.header.stamp=ros::Time(115.05);
            c.externalLandingTick(); holding(c);
        }
        settle(c,115.5);
    } else if(name=="invalid_motion_and_feedback_gap_restart_window") {
        waitForFallback(c);
        tick(c,115.05,.01);tick(c,115.15,.01); holding(c);
        pose(c,115.20,.01);c.feedback_valid_=false;
        c.externalLandingTick(); holding(c);
        tick(c,115.25,.01);tick(c,115.40,.01); holding(c);
        // A gap larger than max_sample_gap cannot count as stable dwell.
        tick(c,115.65,.01); holding(c);
        tick(c,115.8,.01); holding(c);
        tick(c,115.949,.01); holding(c);
        tick(c,115.951,.01);
        assert(c.external_landing_alignment_complete_);
    } else if(name=="cancel_resets_timer_and_active_state") {
        waitForFallback(c);
        ros::clock=115.05;state(c,"POSCTL");cancelled(c);
        assert(!c.landing_coordinate_fallback_.active());
        assert(close(c.landing_coordinate_fallback_.waited(ros::clock),0));
        ros::clock=116;state(c);
        command(c); cancelled(c); // Repeated LAND cannot resurrect cancellation.
        command(c,patrol_control::MissionCommand::RETURN_HOME);
        assert(!c.external_landing_cancelled_ && !c.external_landing_active_);
        c.uav_pose.pose.position={.15,0,1.2};
        landAtOrigin(c,116,43);
        tick(c,116);tick(c,130.999);holding(c);
        assert(!c.landing_coordinate_fallback_.active());
        tick(c,131);holding(c);assert(c.landing_coordinate_fallback_.active());
    } else if(name=="new_land_clears_frozen_descent_and_restarts_wait") {
        waitForFallback(c);settle(c,115.05);
        assert(c.external_landing_alignment_complete_);
        command(c); // Repeated LAND must preserve the already frozen transaction.
        assert(c.external_landing_alignment_complete_ && c.landing_coordinate_fallback_.active());
        command(c,patrol_control::MissionCommand::RETURN_HOME);
        assert(!c.external_landing_active_ && !c.landing_coordinate_fallback_.active());
        c.uav_pose.pose.position={.15,0,1.2};
        landAtOrigin(c,116,43);holding(c);
        assert(!c.landing_coordinate_fallback_.active());
        assert(close(c.landing_coordinate_fallback_.waited(116),0));
        tick(c,116);tick(c,130.999);holding(c);
        assert(!c.landing_coordinate_fallback_.active());
        tick(c,131);holding(c);assert(c.landing_coordinate_fallback_.active());
    } else if(name=="explicit_clear_resets_active_and_partial_dwell") {
        waitForFallback(c);
        tick(c,115.05,.01); tick(c,115.20,.01);
        c.clearExternalLandingState(true);
        assert(!c.landing_coordinate_fallback_.active());
        assert(close(c.landing_coordinate_fallback_.waited(ros::clock),0));
        c.uav_pose.pose.position={.15,0,1.2};
        landAtOrigin(c,116,43);tick(c,116);holding(c);
        tick(c,131);holding(c);
        settle(c,131.05);
    } else if(name=="default_false_preserves_visual_behavior") {
        assert(!LandingCoordinateFallbackConfig{}.enabled);
        c=setup(false);
        for(int i=0;i<=620;++i) {
            tick(c,100+i*.05,.15,0,1.2,0,true,4,5);
            holding(c);assert(!c.landing_coordinate_fallback_.active());
            assert(close(c.patrol_cmd.pose.position.x,4));
        }
        for(int i=0;i<=21;++i)tick(c,132+i*.05,.12,0,1.2,0,true,.12,0);
        assert(c.external_landing_alignment_complete_);
        assert(close(c.patrol_cmd.pose.position.x,.12));
    } else if(name=="only_posctl_can_activate_coordinate_fallback") {
        for(const auto mode: {"AUTO.LAND","OFFBOARD"}) {
            c=setup(true,mode);
            for(int i=0;i<=620;++i) {
                tick(c,100+i*.05,.01);
                holding(c);assert(!c.landing_coordinate_fallback_.active());
                assert(close(c.landing_coordinate_fallback_.waited(ros::clock),0));
            }
        }
    } else { return 2; }
    assert(*c.external_landing_disarm_client_.calls==0);
    std::cout<<"PASS "<<name<<"\n";
}
'''


SIGNATURES = (
    'bool LLController::externalLandingMarkFresh(',
    'bool LLController::externalLandingControlReady(',
    'bool LLController::externalLandingOffboardStateReady(',
    'bool LLController::externalLandingOffboardDescentReady(',
    'void LLController::externalLandingOffboardTick(',
    'void LLController::externalLandingExtendedStateCallback(',
    'void LLController::publishExternalLandingOffboardSetpoint(',
    'void LLController::externalLandingStateCallback(',
    'void LLController::publishExternalLandingHandoff(',
    'void LLController::clearExternalLandingState(',
    'void LLController::failExternalLanding(',
    'void LLController::externalLandingTick(',
    'void LLController::CallLand(',
    'void LLController::missionCommandCallback(',
    'bool LLController::landingMotionSettled(',
    'bool LLController::highLandingAlignmentReady(',
    'bool LLController::coordinateLandingFallbackReady(',
)

CASES = (
    'no_timer_before_xy_or_height_arrival',
    'full_fifteen_seconds_and_return_to_preset_not_visual',
    'xy_speed_height_and_point_three_seconds_required',
    'stability_dwell_not_shortened_to_point_two_nine_nine',
    'worse_pose_or_odom_blocks_coordinate_readiness',
    'visual_success_before_timeout_has_priority',
    'visual_success_at_deadline_precedes_fallback',
    'active_ignores_new_visual_then_freezes_without_reclimb',
    'stale_or_future_odometry_cannot_release',
    'duplicate_odom_cannot_accumulate_dwell',
    'invalid_motion_and_feedback_gap_restart_window',
    'cancel_resets_timer_and_active_state',
    'new_land_clears_frozen_descent_and_restarts_wait',
    'explicit_clear_resets_active_and_partial_dwell',
    'default_false_preserves_visual_behavior',
    'only_posctl_can_activate_coordinate_fallback',
)


class LandingCoordinateFallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        prefix = PROGRAM[:PROGRAM.index('int main(int argc, char** argv)')]
        start = prefix.index('    SettlementWindowStub landing_capture_window_')
        end = prefix.index('    int external_landing_stable_count_', start)
        prefix = prefix[:start] + FEEDBACK + prefix[end:]
        replacements = {
            '    SettlementWindowStub landing_high_alignment_window_;\n': '',
            '    bool highLandingAlignmentReady() { return false; }\n': '',
            '    bool coordinateLandingFallbackReady() { return false; }\n':
                '    bool coordinateLandingFallbackReady();\n',
            'namespace Eigen { struct Vector3d {}; }\n': '',
            '    struct { struct { struct { Position position; } pose; } pose; } motion_odom_;\n': '',
            '    bool freshMotion(Eigen::Vector3d*, Eigen::Vector3d*) { return motion_fresh_; }\n': '',
        }
        for old, new in replacements.items():
            if prefix.count(old) != 1:
                raise AssertionError('Shared fixture changed: ' + old)
            prefix = prefix.replace(old, new)
        prefix = prefix.replace(
            '#include <array>',
            '#include <array>\n#include <Eigen/Core>\n'
            '#include "patrol_control/landing_handoff_stability.h"\n'
            '#include "patrol_control/landing_high_alignment.h"\n'
            'using patrol_control::LandingHandoffSample;')
        methods = '\n'.join(production_method(source, s) for s in SIGNATURES)
        program = prefix.replace('PRODUCTION_METHODS', methods).replace(
            'PRODUCTION_RUN_POINT_BRANCH', '') + MAIN
        cls.temp = tempfile.TemporaryDirectory(prefix='landing_coordinate_fallback_')
        cls.addClassCleanup(cls.temp.cleanup)
        cpp = Path(cls.temp.name) / 'coordinate_fallback.cpp'
        cls.binary = Path(cls.temp.name) / 'coordinate_fallback'
        cpp.write_text(program, encoding='utf-8')
        subprocess.run([
            'g++', '-std=c++14', '-O0', '-Wall', '-Wextra',
            '-Wno-unused-parameter', '-Wno-unused-variable', '-Wno-sign-compare',
            '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-I/usr/include/eigen3', '-I', str(PACKAGE / 'include'),
            str(cpp), '-o', str(cls.binary),
        ], check=True, timeout=30)

    def run_case(self, case):
        completed = subprocess.run(
            [str(self.binary), case], capture_output=True, text=True, timeout=10)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn('PASS ' + case, completed.stdout)


def _case_test(case):
    def test(self):
        self.run_case(case)
    test.__name__ = 'test_' + case
    test.__qualname__ = 'LandingCoordinateFallbackTests.' + test.__name__
    return test


for _case in CASES:
    setattr(LandingCoordinateFallbackTests, 'test_' + _case, _case_test(_case))


if __name__ == '__main__':
    unittest.main()
