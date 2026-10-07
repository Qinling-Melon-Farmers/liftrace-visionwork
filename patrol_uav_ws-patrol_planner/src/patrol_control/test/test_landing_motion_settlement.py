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
void feedback(LLController& c, double t, double z, double error=0., double speed=0., bool mark=true) {
    ros::clock=t; state(c);
    c.uav_pose.pose.position={1+error,2,z};
    c.motion_odom_.pose.pose.position=c.uav_pose.pose.position;
    c.motion_odom_.header.stamp=ros::Time(t);
    c.motion_odom_receipt_=ros::Time(t);
    c.feedback_velocity_=Eigen::Vector3d(speed,0,0);
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
    c.landing_capture_config_=c.landing_settle_config_;
    c.landing_capture_config_.height_tolerance_m=.10;
    c.landing_capture_window_=patrol_control::LandingHandoffStabilityWindow(c.landing_capture_config_);
    return c;
}
int main(int argc, char** argv) {
    assert(argc==2); const std::string name=argv[1];
    auto c=hlanding();
    if (name=="capture_center_speed_height_and_ten_frames") {
        feedback(c,100,1.2,.06); c.externalLandingTick();
        assert(c.external_landing_stable_count_==0);
        feedback(c,100.125,1.2,0.,.10); c.externalLandingTick();
        assert(c.external_landing_stable_count_==0);
        feedback(c,100.25,1.31); c.externalLandingTick();
        assert(c.external_landing_stable_count_==0);
        for (int i=0;i<14;++i) {
            feedback(c,100.375+i*.125,1.2); c.externalLandingTick();
            assert(c.external_landing_alignment_complete_==(i==13));
            assert(c.external_landing_stable_count_==(i<4 ? 0 : i-3));
            assert(c.patrol_cmd.pose.position.z==(i==13 ? .35 : 1.2));
        }
        assert(c.set_mode_client.calls==0);
    } else if (name=="capture_accepts_four_to_six_cm_height_error_only_at_high_view") {
        // 真实高位窗口允许 4～6cm 高度偏差，仍需停稳 0.5s 和十张不同 H 图像。
        for (int i=0;i<=55;++i) {
            feedback(c,100+i*.05,1.2+(i%2 ? .04 : .06),0.,0.,i%5==0);
            c.externalLandingTick();
            assert(c.external_landing_stable_count_==(i<10 ? 0 : (i-10)/5+1));
            assert(c.external_landing_alignment_complete_==(i==55));
            assert(c.patrol_cmd.pose.position.z==(i==55 ? .35 : 1.2));
            assert(c.set_mode_client.calls==0);
        }
    } else if (name=="low_handoff_still_rejects_four_to_six_cm_height_error") {
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_.pose.position={1,2,.35};
        // 上偏超出 0.37 ceiling；下偏虽低于 ceiling，仍必须拒绝超出 ±0.02 的高度误差。
        for (double error : {.04,.06,-.04,-.06}) {
            for (int i=0;i<=10;++i) {
                feedback(c,ros::clock+.05,.35+error,0.,0.,false);
                assert(!c.landingMotionSettled(true,0));
                c.externalLandingTick();
                assert(c.set_mode_client.calls==0);
            }
        }
        // 回到原低位目标后仍须重新形成完整 0.5s 窗口。
        const double start=ros::clock+.05;
        for (int i=0;i<=10;++i) {
            feedback(c,start+i*.05,.35,0.,0.,false); c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==10 ? 1 : 0));
        }
    } else if (name=="capture_20hz_with_four_h_frames_per_second") {
        // 控制与 odom 按 20Hz 更新，H 只有 4Hz；不能把图像间隔当作 odom 断流。
        for (int i=0;i<=55;++i) {
            const bool new_h=i%5==0;
            feedback(c,100+i*.05,1.2,0.,0.,new_h);
            c.externalLandingTick();
            assert(c.external_landing_stable_count_==(i<10 ? 0 : (i-10)/5+1));
            assert(c.external_landing_alignment_complete_==(i==55));
            assert(c.patrol_cmd.pose.position.z==(i==55 ? .35 : 1.2));
        }
        assert(c.set_mode_client.calls==0);
    } else if (name=="capture_pending_3ms_preserves_window_and_four_hz_images") {
        // 每个 20Hz 运动样本先领先本节点时钟 3ms；4Hz H 帧在 pending tick 保留。
        // 追时 tick 复用同一来源样本，不伪造新 odom 或新 H 图像。
        for (int i=0;i<=55;++i) {
            const double t=100+i*.05;
            const bool new_h=i%5==0;
            feedback(c,t,1.2,0.,0.,new_h);
            c.motion_odom_.header.stamp=ros::Time(t+.003);
            c.motion_time_pending_=true;
            c.feedback_valid_=false;
            const int before=c.external_landing_stable_count_;
            assert(!c.landingMotionSettled(false,0));
            c.externalLandingTick();
            assert(c.external_landing_stable_count_==before);
            assert(c.external_landing_new_mark_==new_h);
            assert(!c.external_landing_alignment_complete_);
            assert(c.patrol_cmd.pose.position.z==1.2);
            assert(c.set_mode_client.calls==0);

            ros::clock=t+.003;
            state(c);
            c.motion_time_pending_=false;
            c.feedback_valid_=true;
            c.externalLandingTick();
            assert(!c.external_landing_new_mark_);
            assert(c.external_landing_stable_count_==(i<10 ? 0 : (i-10)/5+1));
            assert(c.external_landing_alignment_complete_==(i==55));
            assert(c.patrol_cmd.pose.position.z==(i==55 ? .35 : 1.2));
            assert(c.set_mode_client.calls==0);
        }
    } else if (name=="handoff_pending_3ms_preserves_window_without_early_posctl") {
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_.pose.position={1,2,.35};
        // 低位连续样本均经历 pending；到满 0.5s 的最后样本也必须等时钟追上。
        for (int i=0;i<=10;++i) {
            const double t=100+i*.05;
            feedback(c,t,.35,0.,0.,false);
            c.motion_odom_.header.stamp=ros::Time(t+.003);
            c.motion_time_pending_=true;
            c.feedback_valid_=false;
            assert(!c.landingMotionSettled(true,0));
            c.externalLandingTick();
            assert(c.external_landing_alignment_complete_);
            assert(!c.external_landing_auto_land_requested_);
            assert(c.patrol_cmd.pose.position.z==.35);
            assert(c.set_mode_client.calls==0);

            ros::clock=t+.003;
            state(c);
            c.motion_time_pending_=false;
            c.feedback_valid_=true;
            c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==10 ? 1 : 0));
            assert(c.external_landing_auto_land_requested_==(i==10));
        }
    } else if (name=="capture_and_handoff_have_independent_windows") {
        for (int i=0;i<5;++i) {
            feedback(c,100+i*.125,1.2);
            assert(c.landingMotionSettled(false,0)==(i==4));
        }
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_.pose.position={1,2,.35};
        for (int i=0;i<5;++i) {
            feedback(c,100.625+i*.125,.35,0.,0.,false); c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==4 ? 1 : 0));
        }
    } else if (name=="low_handoff_rejects_speed_height_and_final_xy") {
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_.pose.position={1,2,.35};
        feedback(c,100,.35,0.,.087,false); c.externalLandingTick();
        assert(c.set_mode_client.calls==0);
        feedback(c,100.125,.32,0.,0.,false); c.externalLandingTick();
        assert(c.set_mode_client.calls==0);
        feedback(c,100.25,.35,.06,0.,false); c.externalLandingTick();
        assert(c.set_mode_client.calls==0);
        for (int i=0;i<5;++i) {
            feedback(c,100.375+i*.125,.35,0.,0.,false); c.externalLandingTick();
            assert(c.set_mode_client.calls==(i==4 ? 1 : 0));
        }
    } else if (name=="latched_h_does_not_follow_cropped_or_missing_images") {
        c.external_landing_alignment_complete_=true;
        c.external_landing_aligned_goal_.pose.position={1,2,.35};
        for (int i=0;i<5;++i) {
            feedback(c,100+i*.125,.35,0.,0.,false);
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


for _case in ('capture_center_speed_height_and_ten_frames',
              'capture_accepts_four_to_six_cm_height_error_only_at_high_view',
              'low_handoff_still_rejects_four_to_six_cm_height_error',
              'capture_20hz_with_four_h_frames_per_second',
              'capture_pending_3ms_preserves_window_and_four_hz_images',
              'handoff_pending_3ms_preserves_window_without_early_posctl',
              'capture_and_handoff_have_independent_windows',
              'low_handoff_rejects_speed_height_and_final_xy',
              'latched_h_does_not_follow_cropped_or_missing_images'):
    setattr(ProductionHSettlementTests, 'test_' + _case,
            lambda self, name=_case: self.run_case(name))


if __name__ == '__main__':
    unittest.main()
