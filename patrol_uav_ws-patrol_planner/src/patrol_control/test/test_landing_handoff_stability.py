"""离线编译并执行生产停稳 helper，不启动 ROS。"""
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
PROGRAM = r'''
#include "patrol_control/landing_handoff_stability.h"
#include <cassert>
#include <cstring>
#include <limits>
#include <string>
using namespace patrol_control;

LandingHandoffSample sample(double stamp) {
    LandingHandoffSample s;
    s.source_stamp_sec = stamp; s.receipt_stamp_sec = stamp + .01;
    s.height_m = .35; s.max_handoff_height_m = .37;
    s.xy_error_m = .025; s.z_error_m = 0;
    s.horizontal_speed_mps = .01; s.vertical_speed_mps = -.02;
    s.alignment_latched = s.control_ready = s.feedback_valid = true;
    return s;
}
LandingHandoffStabilityResult tick(LandingHandoffStabilityWindow& w, double t) {
    return w.update(t + .02, sample(t));
}
void settled(LandingHandoffStabilityWindow& w, double t) {
    for (int i = 0; i < 5; ++i) {
        auto r = tick(w, t + i * .125);
        assert(r.ready == (i == 4));
        assert(r.sample_count == static_cast<std::size_t>(i + 1));
    }
}
int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string name = argv[1];
    LandingHandoffStabilityWindow w;
    if (name == "settle_and_reset") {
        settled(w, 100); w.reset(); assert(!tick(w, 101).ready);
    } else if (name == "source_samples_and_duration") {
        auto s = sample(100);
        for (int i = 0; i < 8; ++i) {
            auto r = w.update(100.02 + i * .02, s);
            assert(!r.ready && r.sample_count == 1 && r.stable_for_sec == 0);
        }
        w.reset();
        LandingHandoffStabilityConfig c; c.stable_duration_sec = .05;
        LandingHandoffStabilityWindow three(c);
        assert(!tick(three, 100).ready);
        assert(!tick(three, 100.0625).ready); // 只达到时长，样本数不足时不能放行。
        assert(tick(three, 100.125).ready);
        LandingHandoffStabilityConfig d; d.max_sample_gap_sec = .3;
        LandingHandoffStabilityWindow duration(d);
        assert(!tick(duration, 100).ready);
        assert(!tick(duration, 100.25).ready);
        assert(tick(duration, 100.5).ready); // 正好三个递增源样本。
    } else if (name == "final_xy_and_observed_flight_speed") {
        settled(w, 100);
        auto s = sample(100.625); s.xy_error_m = .071;
        auto r = w.update(100.645, s);
        assert(!r.ready && std::strcmp(r.reason, "xy_error") == 0);
        s.xy_error_m = .03; s.horizontal_speed_mps = .109;
        r = w.update(100.645, s);
        assert(!r.ready && std::strcmp(r.reason, "horizontal_speed") == 0);
        settled(w, 101); // 超差后必须重新完成整个稳窗。
    } else if (name == "height_bound_and_target_error") {
        auto s = sample(100); s.height_m = .38; s.z_error_m = 0;
        assert(std::strcmp(w.update(100.02, s).reason, "above_handoff_height") == 0);
        s.height_m = .32; s.z_error_m = -.03;
        assert(std::strcmp(w.update(100.02, s).reason, "height_error") == 0);
        // 共用高度基准时，本地 Z 和离地高度应得到相同判断。
        for (int i = 0; i < 5; ++i) {
            s = sample(101 + i * .125);
            s.height_m -= .26349584877491;
            s.max_handoff_height_m -= .26349584877491;
            assert(w.update(s.source_stamp_sec + .02, s).ready == (i == 4));
        }
    } else if (name == "vertical_speed_both_signs") {
        for (double vz : {-.119, .119}) {
            auto s = sample(100); s.vertical_speed_mps = vz;
            assert(std::strcmp(w.update(100.02, s).reason, "vertical_speed") == 0);
        }
    } else if (name == "stale_future_or_invalid_stamp") {
        for (int kind = 0; kind < 5; ++kind) {
            settled(w, 100); auto s = sample(100.625);
            if (kind == 0) s.source_stamp_sec = 100.4;
            if (kind == 1) s.receipt_stamp_sec = 100.4;
            if (kind == 2) s.source_stamp_sec = 100.7;
            if (kind == 3) s.receipt_stamp_sec = 100.7;
            if (kind == 4) s.source_stamp_sec = 0;
            auto r = w.update(100.645, s);
            assert(!r.ready && r.sample_count == 0);
            settled(w, 101);
            w.reset();
        }
    } else if (name == "gaps_and_clock_reversal") {
        tick(w, 100); tick(w, 100.125);
        auto r = tick(w, 100.5);
        assert(!r.ready && r.sample_count == 1); // 不能累计断流期间的稳定时间。
        for (int kind = 0; kind < 3; ++kind) {
            w.reset(); tick(w, 100); auto s = sample(100.0625);
            double now = 100.0825;
            if (kind == 0) { now = 100.015; s = sample(100); }
            if (kind == 1) s.source_stamp_sec = 99.99;
            if (kind == 2) s.receipt_stamp_sec = 100.005;
            assert(std::strcmp(w.update(now, s).reason, "time_reversed") == 0);
        }
    } else if (name == "short_image_gap_does_not_revoke_anchor") {
        // 窗口不接收新 H 图像：检测短暂丢帧、odom 仍新鲜时保留已有锁点。
        for (int i = 0; i < 5; ++i)
            assert(tick(w, 100 + i * .125).ready == (i == 4));
        w.reset(); tick(w, 100); tick(w, 100.125);
        assert(tick(w, 100.3125).sample_count == 3); // 允许门槛内的短 odom 间隔。
        assert(!tick(w, 100.4375).ready);
        assert(tick(w, 100.5625).ready);
    } else if (name == "ownership_and_feedback_loss") {
        for (int kind = 0; kind < 3; ++kind) {
            w.reset(); settled(w, 100); auto s = sample(100.625);
            if (kind == 0) s.alignment_latched = false;
            if (kind == 1) s.control_ready = false;
            if (kind == 2) s.feedback_valid = false;
            assert(!w.update(100.645, s).ready);
            assert(!tick(w, 100.75).ready);
        }
    } else if (name == "nonfinite_and_invalid_config") {
        for (int kind = 0; kind < 9; ++kind) {
            auto s = sample(100); double now = 100.02;
            double* values[] = {&now, &s.source_stamp_sec, &s.receipt_stamp_sec,
                &s.height_m, &s.max_handoff_height_m, &s.xy_error_m, &s.z_error_m,
                &s.horizontal_speed_mps, &s.vertical_speed_mps};
            *values[kind] = std::numeric_limits<double>::quiet_NaN();
            assert(!w.update(now, s).ready);
        }
        LandingHandoffStabilityConfig c; c.min_samples = 2;
        LandingHandoffStabilityWindow invalid(c);
        assert(std::strcmp(tick(invalid, 100).reason, "invalid_config") == 0);
        c.min_samples = 3; c.max_odom_age_sec = 0;
        assert(!c.valid());
    } else { assert(false); }
}
'''


class HandoffStabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='h_handoff_stability_')
        source = Path(cls.temp.name) / 'test.cpp'
        cls.binary = Path(cls.temp.name) / 'test'
        source.write_text(PROGRAM, encoding='utf-8')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                        '-I', str(PACKAGE / 'include'), str(source), '-o', str(cls.binary)],
                       check=True, capture_output=True, text=True, timeout=30)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def check_case(self, name):
        subprocess.run([str(self.binary), name], check=True, capture_output=True,
                       text=True, timeout=10)


for _case in ('settle_and_reset', 'source_samples_and_duration',
              'final_xy_and_observed_flight_speed', 'height_bound_and_target_error',
              'vertical_speed_both_signs', 'stale_future_or_invalid_stamp',
              'gaps_and_clock_reversal', 'short_image_gap_does_not_revoke_anchor',
              'ownership_and_feedback_loss', 'nonfinite_and_invalid_config'):
    setattr(HandoffStabilityTests, 'test_' + _case,
            lambda self, name=_case: self.check_case(name))


if __name__ == '__main__':
    unittest.main()
