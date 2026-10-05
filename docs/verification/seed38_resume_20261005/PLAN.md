# seed38 高位续扫开关对照
本轮用户明确授权两轮对照。先修复释放许可乱序记账，再以同一提交、同seed38靶位、三扫描线、镜头AGL 2m（FC 2.16m）运行关闭/开启高位续扫各一轮。净空10×10m、原始均匀四树、80cm H、障碍柱和0.25m水平膨胀、最新速度优化/全域直线偏好保持相同。唯一策略差异为 resume_survey_enabled。

沿用既有sim_run.sh单实例包装及原始Gate，不改600秒任务预算，不自动重跑；第一轮结束分析首个失败阶段后才能运行第二轮。两轮持续记录下视和俯视视频、视觉中心/释放回执bag、轨迹与阶段事件，生成汇报视频。以正确目标释放、实际触地/接触、低空补扫和总时长评价；没触发续扫时不宣称续扫收益。

本次仅SITL，不连接板端、不启动真实执行器。快mock不能证明真实1秒执行器的硬件连续性；许可乱序由定向生产方法回归额外覆盖。

## 2026-10-05 correction run: resume_on_fixed

The original A/B source remains `7a9502049b9448856152036632ef31857ada2582`.
Preserve both original runs, their scene inputs and
`logs/seed38_resume_20261005_batch/matrix.json` without modification.

After the main proxy fix and these runner changes are committed, run exactly
one correction using the latest clean committed HEAD:

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python docs/verification/seed38_resume_20261005/correction_preflight.py --rerun-fixed
python docs/verification/seed38_resume_20261005/run_case.py --rerun-fixed --model /home/xhj/liftrace/deliverables/liftrace_five_class_20260928_models/flight_5cls_20260928.pt --execute-authorized
```

Invoke WSL commands with `wsl -e bash -c '...'` from Windows, with login disabled.
The correction uses the original `resume_on` scene, seed38, camera AGL 2m,
FC AGL 2.16m, world, three survey lines and thread settings. Its variant is
`resume_on_fixed`; results go to the separate
`logs/seed38_resume_20261005_fixed_batch/matrix.json` and a new
`logs/seed38_resume_resume_on_fixed_seed38_<timestamp>/` run directory.
An existing correction matrix blocks any overwrite or automatic retry.

`replay.launch` defaults `stop_on_collision` to false and places the final
typed override at `/navigation_vcl06_assertion/stop_on_collision` after all
includes and YAML. The fixed runner explicitly passes false; historical case
commands explicitly pass true but remain source-pinned and cannot run after
the source changes. Existing Gate logic records collisions as FAIL while
allowing observation through ground/deadline; collision facts and checks are
unchanged. Keep the 600s mission limit and the existing single-instance
`sim_run.sh` cleanup. If the LAND tail persists after ground, the main operator
may stop it manually after 30s using `stop_toudi3_sim.sh` and verify cleanup.

Downward, overview and follow video nodes remain enabled; `SIM_NO_RECORD=1`
only disables desktop capture. `correction_preflight.py` is a static read-only XML/YAML check with
existing conda/PyYAML, without ROS imports or processes. It renders the direct
final replay parameters from the actual runner argv and compares scene inputs
against the frozen commit; it does not claim a full ROS launch expansion.
It never rewrites historical `preflight.json` or `effective_parameters.json`.
The original full ROS XML `preflight.py` is restored exactly from `7a950204`;
the main agent validates full XML expansion separately with source overlays,
without rewriting frozen generated parameters or rebuilding unnecessarily.

Change scope: replay launch, runner, correction preflight and this plan only. Main owns
proxy changes, shared change-log entry, commit, execution and report/analysis.
Offline verification: fixed and historical explicit-true modes both PASS
under `rl_drone`; the fixed rendered parameter is false and `git diff --check`
passes. Dynamic verification remains pending the authorized main-agent run.
