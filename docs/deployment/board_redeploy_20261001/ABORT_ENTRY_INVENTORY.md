# 任务层终止入口静态清单

2026-10-01。按调用点统计，不等于独立故障数；包含转发、实验段正常结束及故障出口。范围为 uav_mission Python 核心/节点和专项运行脚本；不包含 PX4 内部保护、C++ 规划器全部失败条件。

共 91 个调用点。

| 文件:行 | 调用 |
|---|---|
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py:263 | self._finish(False,'ascent_not_verified',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py:526 | self._finish(False,'fallback_route_unavailable',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py:201 | self._fail_closed('survey_binding_mismatch',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py:203 | self._fail_closed('survey_interrupt_failed',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py:284 | self._finish(False,'no_local_descent_route',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_full.py:671 | self._fail_closed('unexpected_delivery_dispatch',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_probe.py:127 | super().abort('research_segment_end:'+reason,now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_probe.py:114 | self._finish(False,'ascent_not_verified',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_probe.py:138 | self._finish(False,'motion_failed:'+self.stage,now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_probe.py:213 | self._finish(False,'reacquisition_timeout',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_probe.py:155 | self._finish(False,'no_high_view_hint',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/high_view_probe.py:157 | self._finish(False,'hint_expired_before_revisit',now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_core.py:1049 | self._abort_action(reason, now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_core.py:1242 | self._abort_action(                 "safety_motion_timed_out", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_core.py:1082 | self._abort_action(                     "return_home_failed", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_core.py:1104 | self._abort_action(                     "landing_failed", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:195 | self.core.abort(reason, now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:212 | self._fail_closed(                 "route_dispatch_after_completion", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:215 | self._fail_closed(                 "route_dispatch_while_active", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:235 | self._fail_closed(                 "scheduler_phase_mismatch", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:238 | self._fail_closed(                 "scheduler_active_decision_mismatch", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:241 | self._fail_closed(                 "scheduler_route_decision_mismatch", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:257 | self._fail_closed(                 "coverage_completion_without_decision", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:269 | self._fail_closed("search_action_missing", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:271 | self._fail_closed("search_route_binding_mismatch", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:284 | self._fail_closed(                 "search_replacement_command_invalid", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:288 | self._fail_closed(                 "route_interrupt_failed", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:405 | self._fail_closed(                 "mission_has_no_active_decision", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:485 | self.core.abort(reason, now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:225 | self._fail_closed(                 "route_dispatch_transaction_failed", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:247 | self._fail_closed(                 "mission_selection_failed", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:251 | self._fail_closed(                     "mission_selection_command_invalid", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:280 | self._fail_closed("search_replacement_failed", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:300 | self._fail_closed(                 "route_result_reduction_failed", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:124 | self._fail_closed(                 "runtime_clock_rollback", safe_now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:129 | self._fail_closed(                 "runtime_clock_invalid", safe_now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:393 | self._fail_closed("lease_reduction_failed", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:422 | self._fail_closed(reason, now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:483 | self._fail_closed(                         "abort_route_retirement_failed", now, route_outcome) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:441 | self._fail_closed(                             "route_interrupt_failed",                             now,                             route_outcome,                         ) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/mission_runtime.py:372 | self._fail_closed(                                     "route_interrupt_failed",                                     now,                                     route_outcome,                                 ) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:842 | self._fail_closed("planner_status_unhandled") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:482 | self._fail_closed("executor_clock_rollback") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:622 | self._fail_closed("decision_sequence_conflict") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:626 | self._fail_closed(contract_error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:633 | self._fail_closed("decision_from_future") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:635 | self._fail_closed("decision_received_after_deadline") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:641 | self._fail_closed("decision_issue_stamp_conflict") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:723 | self._fail_closed("planner_event_sequence_conflict") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:728 | self._fail_closed("planner_event_from_future") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:739 | self._fail_closed("planner_event_precedes_dispatch") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:744 | self._fail_closed("planner_goal_sequence_mismatch") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:746 | self._fail_closed("planner_requested_goal_mismatch") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:749 | self._fail_closed("planner_effective_goal_frame_mismatch") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:752 | self._fail_closed("planner_effective_goal_height_invalid") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:759 | self._fail_closed("planner_effective_goal_offset_exceeded") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:770 | self._fail_closed("planner_event_not_active_goal") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:841 | self._fail_closed("active_planner_goal_cancelled") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:1053 | self._fail_closed("planner_effective_goal_missing") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:480 | self._fail_closed("executor_clock_invalid") |
| patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/planner_execution.py:657 | self._fail_closed("pending_release_fence_conflict") |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_high_view_full.py:44 | self._handle_callback_exception('navigation_hints',error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_high_view_full.py:88 | self._handle_callback_exception('cost_map',error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_high_view_probe.py:42 | self._handle_callback_exception('camera_info',ValueError('calibration_changed')) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_high_view_probe.py:53 | self._handle_callback_exception('probe_pose',error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py:352 | self._runtime.abort(                     reason, rospy.Time.now().to_sec()) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py:526 | self._runtime.abort("manual_abort_requested", now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py:412 | self._handle_callback_exception("candidates", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py:465 | self._handle_callback_exception("result", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py:565 | self._handle_callback_exception("timer", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_mission_manager.py:554 | self._runtime.abort(reason, now) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1119 | self._handle_callback_exception("decision", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1137 | self._handle_callback_exception("planner_status", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1154 | self._handle_callback_exception("odom", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1163 | self._handle_callback_exception("targets", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1188 | self._handle_callback_exception("release_context", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1250 | self._handle_callback_exception("release_result", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1285 | self._handle_callback_exception("control_state", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1293 | self._handle_callback_exception("align_mode", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1301 | self._handle_callback_exception("landed_state", error) |
| patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py:1318 | self._handle_callback_exception("timer", error) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_manager.py:65 | self._handle_callback_exception('camera_info',ValueError('calibration_changed')) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_manager.py:61 | self._handle_callback_exception('navigation_hints',error) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_manager.py:99 | self._handle_callback_exception('cost_map',e) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_manager.py:73 | self._handle_callback_exception('board_pose',e) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_runtime.py:93 | self._finish(False,'board_memorized_target_incomplete:'+reason,now) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_runtime.py:38 | self._fail_closed('board_line_finished_without_delivery',now) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_runtime.py:79 | self._finish(False,'board_no_valid_target_recorded',now) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_runtime.py:111 | self._fail_closed('board_line_finished_before_delivery_count',now) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_runtime.py:160 | self._finish(False,'board_capture_route_incomplete',now) |
| deployment/board_trials_4x4/common/uav_board_trials/scripts/trial_runtime.py:84 | self._finish(False,'board_full_circle_incomplete',now) |
