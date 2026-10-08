"""离线执行生产 H tick 与真实停稳窗口，不启动 ROS 进程。

仅替代反馈传输；直接提取生产 landingMotionSettled，并使用真实 C++ helper。
交易测试中的停稳 stub 不能代替本测试。
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
# catkin/nose 按包名导入，unittest 按目录导入；两种入口都显式定位同目录测试工具。
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_external_landing_handoff import PROGRAM, PACKAGE, production_method


FEEDBACK = r'''
    patrol_control::LandingHandoffStabilityConfig landing_settle_config_, landing_capture_config_;
    patrol_control::LandingHandoffStabilityWindow landing_capture_window_, landing_handoff_window_;
    struct {
        Header header;
        struct { struct { Position position; } pose; } pose;
    } motion_odom_;
    ros::Time motion_odom_receipt_;
    Eigen::Vector3d feedback_velocity_{0,0,0};
    bool feedback_valid_=true;
    bool motion_time_pending_=false;
    bool motionTimePending() const { return motion_time_pending_; }
    bool freshMotion(Eigen::Vector3d* v, Eigen::Vector3d* w,
                     const char** rejection=nullptr) const {
        if (rejection) *rejection=motion_time_pending_ ? "motion_time_pending" : nullptr;
        *v=feedback_velocity_; *w=Eigen::Vector3d::Zero(); return feedback_valid_;
    }
    bool landingMotionSettled(bool handoff, double xy_error);
'''

MAIN = r'''
void feedback(LLController& c, double t, double z, double error=0., double speed=0., bool mark=true, double vz=0.) {
    ros::clock=t; state(c);
    c.uav_pose.pose.position={1+error,2,z};
    c.motion_odom_.pose.pose.position=c.uav_pose.pose.position;
    c.motion_odom_.header.stamp=ros::Time(t);
    c.motion_odom_receipt_=ros::Time(t);
    c.feedback_velocity_=Eigen::Vector3d(speed,0,vz);
    c.have_land_mark=true;
    c.external_landing_new_mark_=mark;
    if (mark) {
        c.land_mark_point.pose.position={1,2,0};
        c.external_landing_last_mark_stamp_=ros::Time(t);
        c.external_landing_last_mark_receipt_=ros::Time(t);
    }
}
LLController hlanding() {
    ros::clock=100;
    auto c=landing(false);
    c.external_landing_handoff_mode_="POSCTL";
    c.external_landing_capture_height_=1.2;
    c.land_height=.35;
    c.external_landing_auto_land_height_=.37;
    c.landing_settle_config_.max_horizontal_speed_mps=.08;
    c.landing_settle_config_.max_vertical_speed_mps=.10;
    c.landing_settle_config_.stable_duration_sec=.15;
    c.landing_handoff_window_=patrol_control::LandingHandoffStabilityWindow(c.landing_settle_config_);
    c.landing_capture_config_=c.landing_settle_config_;
    c.landing_capture_config_.height_tolerance_m=.10;
    c.landing_capture_window_=patrol_control::LandingHandoffStabilityWindow(c.landing_capture_config_);
    return c;
}
void frozen(LLController& c) {
    c.external_landing_alignment_complete_=true;
    c.external_landing_aligned_goal_.pose.position={1,2,.35};
}
int main(int argc, char** argv) {
    assert(argc==2); const std::string name=argv[1];
    auto c=hlanding();
    if (name=="capture_original_ten_fresh_h_and_xy_without_motion_window") {
        // High acquisition preserves original visual gate, independent of motion precision.
        feedback(c,100,1.2,.10); c.externalLandingTick();
        assert(c.external_landing_stable_count_==0);
        for (int i=0;i<10;++i) {
            feedback(c,100.125+i*.125,1.26,0.,.10,true,-.12);
            c.externalLandingTick();
            assert(c.external_landing_stable_count_==i+1);
            assert(c.external_landing_alignment_complete_==(i==9));
            assert(c.patrol_cmd.pose.position.z==(i==9 ? .35 : 1.2));
            assert(c.set_mode_client.calls==0);
        }
    } else if (name=="capture_20hz_feedback_four_hz_images_parallel") {
        for (int i=0;i<=45;++i) {
            feedback(c,100+i*.05,1.26,0.,.12,i%5==0);
            c.externalLandingTick();
            assert(c.external_landing_stable_count_==i/5+1);
            assert(c.external_landing_alignment_complete_==(i==45));
        }
        assert(c.set_mode_client.calls==0);
    } else if (name=="capture_stale_h_and_bad_xy_reset_visual_count") {
        for (int i=0;i<4;++i) {
            feedback(c,100+i*.05,1.2); c.externalLandingTick();
        }
        assert(c.external_landing_stable_count_==4);
        feedback(c,100.25,1.2,.081); c.externalLandingTick();
        assert(c.external_landing_stable_count_==0);
        feedback(c,100.30,1.2); c.externalLandingTick();
        ros::clock=101; state(c); c.externalLandingTick();
        assert(c.external_landing_stable_count_==0 && !c.external_landing_alignment_complete_);
    } else if (name=="high_capture_independent_height_tolerance_knowledge") {
        assert(c.landing_capture_config_.height_tolerance_m==.10);
        assert(c.landing_settle_config_.height_tolerance_m==.02);
        for (int i=0;i<5;++i) {
            feedback(c,100+i*.04,1.26);
            assert(c.landingMotionSettled(false,0)==(i==4));
        }
        feedback(c,100.25,1.31);
        assert(!c.landingMotionSettled(false,0));
    } else if (name=="low_accepts_controlled_descent_after_point_fifteen_seconds") {
        frozen(c);
        for (int i=0;i<5;++i) {
            feedback(c,100+i*.04,.366-i*.004,0.,.075,false,-.10);
            c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==4 ? 1 : 0));
        }
        assert(c.external_landing_auto_land_requested_);
    } else if (name=="low_ceiling_lower_bound_xy_and_fast_motion_reject") {
        frozen(c);
        for (int kind=0;kind<6;++kind) {
            for (int i=0;i<6;++i) {
                feedback(c,ros::clock+.04,kind==0 ? .371 : (kind==1 ? .329 : .35),
                    kind==2 ? .051 : 0.,kind==3 ? .081 : 0.,false,
                    kind==4 ? -.101 : (kind==5 ? .101 : 0.));
                c.externalLandingTick();
                assert(c.set_mode_client.calls==0);
            }
        }
        const double start=ros::clock+.04;
        for (int i=0;i<5;++i) {
            feedback(c,start+i*.04,.35,0.,.079,false,-.099);
            c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==4 ? 1 : 0));
        }
    } else if (name=="low_duplicate_source_does_not_accumulate_dwell") {
        frozen(c);
        feedback(c,100,.35,0.,0.,false); c.externalLandingTick();
        for (int i=1;i<=4;++i) {
            ros::clock=100+i*.04; state(c); c.externalLandingTick();
            assert(c.set_mode_client.calls==0);
        }
    } else if (name=="low_odom_gap_restarts_fresh_feedback_window") {
        frozen(c);
        feedback(c,100,.35,0.,0.,false); c.externalLandingTick();
        feedback(c,100.08,.35,0.,0.,false); c.externalLandingTick();
        for (int i=0;i<5;++i) {
            feedback(c,100.40+i*.04,.35,0.,0.,false); c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==4 ? 1 : 0));
        }
    } else if (name=="low_pending_3ms_keeps_window_without_early_posctl") {
        frozen(c);
        for (int i=0;i<5;++i) {
            const double t=100+i*.04;
            feedback(c,t,.35,0.,0.,false,-.08);
            c.motion_odom_.header.stamp=ros::Time(t+.003);
            c.motion_time_pending_=true; c.feedback_valid_=false;
            c.externalLandingTick();
            assert(c.set_mode_client.calls==0);
            ros::clock=t+.003; state(c);
            c.motion_time_pending_=false; c.feedback_valid_=true;
            c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==4 ? 1 : 0));
        }
    } else if (name=="latched_h_does_not_follow_cropped_or_missing_images") {
        frozen(c);
        for (int i=0;i<5;++i) {
            feedback(c,100+i*.04,.35,0.,0.,false,-.08);
            c.land_mark_point.pose.position={8,9,0};
            c.external_landing_last_mark_stamp_=ros::Time(90);
            c.external_landing_last_mark_receipt_=ros::Time(90);
            c.externalLandingTick();
            assert(c.patrol_cmd.pose.position.x==1 && c.patrol_cmd.pose.position.y==2);
            assert(c.set_mode_client.calls==(i==4 ? 1 : 0));
        }
    } else { return 2; }
}
'''


class ProductionHSettlementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (PACKAGE / 'src/patrol_control.cpp').read_text(encoding='utf-8')
        prefix = PROGRAM[:PROGRAM.index('int main(int argc, char** argv)')]
        stub_start = prefix.index('    SettlementWindowStub landing_capture_window_')
        stub_end = prefix.index('    int external_landing_stable_count_', stub_start)
        prefix = prefix[:stub_start] + FEEDBACK + prefix[stub_end:]
        prefix = prefix.replace('#include <array>', '#include <array>\n#include <Eigen/Core>\n#include "patrol_control/landing_handoff_stability.h"\nusing patrol_control::LandingHandoffSample;')
        prefix = prefix.replace('bool isZero() const', 'double toSec() const { return value; }\n    bool isZero() const')
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
            'bool LLController::landingMotionSettled(',
        )
        methods = '\n'.join(production_method(source, signature) for signature in signatures)
        program = prefix.replace('PRODUCTION_METHODS', methods).replace('PRODUCTION_RUN_POINT_BRANCH', '') + MAIN
        cls.temp = tempfile.TemporaryDirectory(prefix='production_h_settlement_')
        cls.addClassCleanup(cls.temp.cleanup)
        cpp = Path(cls.temp.name) / 'settlement.cpp'
        cls.binary = Path(cls.temp.name) / 'settlement'
        cpp.write_text(program, encoding='utf-8')
        subprocess.run(['g++','-std=c++14','-O0','-Wall','-Wextra',
                        '-Wno-unused-parameter','-Wno-unused-variable','-Wno-sign-compare',
                        '-fsanitize=undefined','-fno-sanitize-recover=all',
                        '-I/usr/include/eigen3','-I',str(PACKAGE/'include'),
                        str(cpp),'-o',str(cls.binary)], check=True, timeout=30)

    def run_case(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=10)


def _make_case_test(name):
    def test_case(self):
        self.run_case(name)
    # nose按方法自身名称发现测试；仅setattr改属性名会漏收原名为lambda的方法。
    test_case.__name__ = 'test_' + name
    test_case.__qualname__ = 'ProductionHSettlementTests.' + test_case.__name__
    return test_case


for _case in ('capture_original_ten_fresh_h_and_xy_without_motion_window',
              'capture_20hz_feedback_four_hz_images_parallel',
              'capture_stale_h_and_bad_xy_reset_visual_count',
              'high_capture_independent_height_tolerance_knowledge',
              'low_accepts_controlled_descent_after_point_fifteen_seconds',
              'low_ceiling_lower_bound_xy_and_fast_motion_reject',
              'low_duplicate_source_does_not_accumulate_dwell',
              'low_odom_gap_restarts_fresh_feedback_window',
              'low_pending_3ms_keeps_window_without_early_posctl',
              'latched_h_does_not_follow_cropped_or_missing_images'):
    setattr(ProductionHSettlementTests, 'test_' + _case, _make_case_test(_case))


if __name__ == '__main__':
    unittest.main()
