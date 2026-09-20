#!/usr/bin/python3
import json
import time

import rospy
import source_modules
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import TriggerResponse
from navigation_mission_manager import NavigationMissionManager
from uav_mission.mission_core import MissionPhase
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.search_types import Waypoint
from single_drop_core import SingleDropCore, StrictRoute, landing_ready


class TestMissionManager(NavigationMissionManager):
    def __init__(self):
        self.state = None
        self.state_at = 0.0
        self.odom = None
        self.extended = None
        self.extended_at = 0.0
        self.land_since = None
        self.land_requested_at = None
        self.last_land_request = 0.0
        self.land_attempts = 0
        self.land_mode_seen = False
        self.started_once = False
        self.start_pending = False
        self.start_deadline = 0.0
        self.start_stable_since = None
        self.start_armed_seen = False
        self.start_offboard_seen = False
        self.control_ready = False
        self.endpoint = tuple(rospy.get_param("~mission/landing_xy"))
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.test_status = rospy.Publisher("/test_4x5/status", String, queue_size=1, latch=True)
        super().__init__()
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=1)
        self.extended_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.on_extended, queue_size=1)
        self.odom_sub = rospy.Subscriber("/navigation/local_odom", Odometry, self.on_odom, queue_size=1)
        self.ready_sub = rospy.Subscriber("/mission/control_ready", Bool, self.on_control_ready, queue_size=1)

    def on_control_ready(self, message):
        with self._lock:
            self.control_ready = message.data

    def on_state(self, message):
        with self._lock:
            self.state, self.state_at = message, rospy.Time.now().to_sec()
            if self.land_requested_at is not None and message.mode == "AUTO.LAND":
                self.land_mode_seen = True

    def on_extended(self, message):
        with self._lock:
            self.extended, self.extended_at = message, rospy.Time.now().to_sec()

    def on_odom(self, message):
        with self._lock:
            self.odom = message

    def _new_runtime(self):
        config = self._mission_config()
        points = tuple(Waypoint(*point) for point in rospy.get_param("~mission/post_delivery_route"))
        return MissionRuntime(SingleDropCore(config), StrictRoute(points, "hardware-red-cross-4x5-r1", 2))

    def _on_start(self, request):
        with self._lock:
            if self.started_once:
                return TriggerResponse(success=False, message="one_mission_per_process")
            if not self.start_pending:
                self.start_pending = True
                self.start_deadline = time.monotonic() + 120.0
                self.start_stable_since = None
                self.start_armed_seen = False
                self.start_offboard_seen = False
            self._last_reason = "start_queued_waiting_for_armed_OFFBOARD_takeoff_stable_Z_0.25"
            self._publish_status(force=True)
            return TriggerResponse(success=True, message=self._last_reason)

    def _on_abort(self, request):
        with self._lock:
            if self.start_pending:
                self.start_pending = False
                self.start_stable_since = None
                self._last_reason = "queued_start_cancelled"
                self._publish_status(force=True)
                return TriggerResponse(success=True, message=self._last_reason)
            return super()._on_abort(request)

    def pending_start_tick(self):
        now = rospy.Time.now().to_sec()
        wall_now = time.monotonic()
        fresh = (self.state is not None and self.state.connected
                 and 0 <= now - self.state_at <= 0.5)
        cancelled = (fresh and ((self.start_armed_seen and not self.state.armed)
                     or (self.start_offboard_seen and self.state.mode != "OFFBOARD")))
        if wall_now >= self.start_deadline or cancelled:
            self.start_pending = False
            self.start_stable_since = None
            self._last_reason = "queued_start_cancelled" if cancelled else "queued_start_timeout"
            self._publish_status(force=True)
            return
        if fresh:
            self.start_armed_seen = self.start_armed_seen or self.state.armed
            self.start_offboard_seen = self.start_offboard_seen or self.state.mode == "OFFBOARD"
        if not self.control_ready or not landing_ready(now, self.state, self.state_at, self.odom, (0.0, 0.0)):
            self.start_stable_since = None
            return
        if self.start_stable_since is None:
            self.start_stable_since = wall_now
        if wall_now - self.start_stable_since < 1.0:
            return
        self.start_pending = False
        result = super()._on_start(None)
        self.started_once = result.success
        self._last_reason = "mission_started_after_takeoff" if result.success else "queued_start_rejected:" + result.message
        self._publish_status(force=True)

    def _publish_action(self, action):
        if action is not None and action.command == "LAND":
            if not self._runtime.route.is_complete:
                aborted = self._runtime.abort("landing_before_route_complete", rospy.Time.now().to_sec())
                super()._publish_action(aborted.action)
            return
        super()._publish_action(action)

    def _on_timer(self, event):
        with self._lock:
            if self.start_pending:
                self.pending_start_tick()
                self._publish_status(force=True)
                return
            if self._runtime is not None and self._runtime.core.phase == MissionPhase.LAND:
                self.land_tick()
                self._publish_status(force=True)
                return
            if (self._runtime is not None and self.started_once and self.state is not None
                    and 0 <= rospy.Time.now().to_sec() - self.state_at <= 0.5
                    and (not self.state.armed or self.state.mode != "OFFBOARD")
                    and self._runtime.core.phase not in (MissionPhase.COMPLETE, MissionPhase.ABORTED)):
                outcome = self._runtime.abort("pilot_mode_change_or_disarmed", rospy.Time.now().to_sec())
                self._publish_action(outcome.action)
                self._last_reason = "pilot_mode_change_or_disarmed"
                self._publish_status(force=True)
                return
            super()._on_timer(event)

    def land_tick(self):
        now = rospy.Time.now().to_sec()
        core = self._runtime.core
        if not self._runtime.route.is_complete:
            return
        state_fresh = self.state is not None and 0 <= now - self.state_at <= 0.5
        if (self.land_mode_seen and state_fresh and self.state.connected and not self.state.armed
                and self.extended is not None and 0 <= now - self.extended_at <= 0.5
                and self.extended_at > self.land_requested_at
                and self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND):
            core.phase = MissionPhase.COMPLETE
            core.active_action = None
            self._last_reason = "route_complete_landed_disarmed"
            return
        if core.active_action is not None and now > core.active_action.deadline_at:
            outcome = self._runtime.abort("landing_confirmation_timeout", now)
            super()._publish_action(outcome.action)
            return
        if self.land_requested_at is not None:
            self._last_reason = "AUTO_LAND_requested_waiting_for_landed_disarmed"
            return
        if not landing_ready(now, self.state, self.state_at, self.odom, self.endpoint):
            self.land_since = None
            self._last_reason = "waiting_for_final_waypoint_settle"
            return
        if self.land_since is None:
            self.land_since = now
        if now - self.land_since < 1.0 or now - self.last_land_request < 2.0 or self.land_attempts >= 3:
            return
        self.land_attempts += 1
        self.last_land_request = now
        try:
            self.set_mode.wait_for_service(timeout=0.2)
            response = self.set_mode(base_mode=0, custom_mode="AUTO.LAND")
            if response.mode_sent:
                self.land_requested_at = now
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("AUTO.LAND request failed: %s", error)

    def _publish_status(self, force=False):
        super()._publish_status(force)
        if self._runtime is None:
            self.test_status.publish(String(data=json.dumps({
                "phase": "WAITING_FOR_TAKEOFF" if self.start_pending else "IDLE",
                "start_pending": self.start_pending,
                "last_reason": self._last_reason,
            }, sort_keys=True)))
        if self._runtime is not None:
            self.test_status.publish(String(data=json.dumps({
                "delivery_outcome": self._runtime.core.delivery_outcome,
                "attempt_started": self._runtime.core.attempt_started,
                "committed_slots": self._runtime.core.committed_slots,
                "route_index": self._runtime.route.current_index,
                "route_complete": self._runtime.route.is_complete,
                "phase": self._runtime.core.phase.value,
                "last_reason": self._last_reason,
            }, sort_keys=True)))


if __name__ == "__main__":
    rospy.init_node("mission_manager")
    TestMissionManager()
    rospy.spin()
