#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from uav_vision.msg import TargetCandidate
from uav_mission.candidate_policy import CandidatePolicy
from uav_mission.search_policy import SearchPolicy
from uav_mission.search_types import MissionState, SearchContext

# 总结一下，本file
# 0.本file规定goal，且发布
# 1.接受mavros发来的pose信息，并且判断是否到了位置上

class TargetSearchManager:
    def __init__(self):
        rospy.init_node("target_search_manager_py")
        # 显示初始化变量
        self.current_pose = None
        self.current_goal = None
        self.arrival_threshold = float(
            rospy.get_param("~arrival/threshold", 0.3)
        )
        self.goal_reached = False
        self.selected_target = None
        self.candidate_policy = CandidatePolicy(
            minimum_state=int(rospy.get_param("~target/minimum_state", 2))
        )
        self.search_context = None
        self.approach_altitude = float(
            rospy.get_param("~target/approach_altitude", 1.2)
        )
        self.hold_duration = float(
            rospy.get_param("~target/hold_duration", 3.0)
        )
        self.hold_started_at = None


        # self.search_waypoints = [
        #     (0.0, 0.0, 5.0),
        #     (2.0, 1.0, 5.0),
        #     (2.0, 2.0, 5.0),
        # ] , historical site
        # self.current_waypoint_index = 0
        self.search_policy = SearchPolicy(
                min_x=rospy.get_param("~search/min_x", -3.6),
                max_x=rospy.get_param("~search/max_x", 2.6),
                min_y=rospy.get_param("~search/min_y", -2.0),
                max_y=rospy.get_param("~search/max_y", 6.0),
                lane_spacing=rospy.get_param("~search/lane_spacing", 1.2),
                altitude=rospy.get_param("~search/altitude", 2.2),
            )

        rospy.loginfo(
            "Mission parameters: area=[%.2f, %.2f] x [%.2f, %.2f], "
            "spacing=%.2f, search_z=%.2f, approach_z=%.2f, "
            "arrival=%.2f, hold=%.2f",
            self.search_policy.min_x,
            self.search_policy.max_x,
            self.search_policy.min_y,
            self.search_policy.max_y,
            self.search_policy.lane_spacing,
            self.search_policy.altitude,
            self.approach_altitude,
            self.arrival_threshold,
            self.hold_duration,
        )
        
        # 状态
        self.mission_state = MissionState.SEARCH

        # 创建发布器
        self.goal_pub = rospy.Publisher(
                    "/fastplanner/goal",
                    PoseStamped,
                    queue_size=1
                )
        
        rospy.sleep(1.0)
        # 发布目标点
        waypoint = self.search_policy.current_waypoint
        self.publish_goal(*waypoint.as_tuple())

        # 创建pose订阅器
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )

        self.target_sub = rospy.Subscriber(
            "/uav_vision/selected_target",
            TargetCandidate,
            self.target_callback,
            queue_size=1
        )

        self.mission_timer = rospy.Timer(
            rospy.Duration(0.1),
            self.mission_timer_callback
        )
        
    def pose_callback(self, msg):
        self.current_pose = msg
        rospy.loginfo_throttle(
            1.0,
            "current position: x=%.2f, y=%.2f, z=%.2f",
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

        distance = self.distance_to_goal()
        rospy.loginfo_throttle(
            1.0,
            "distance to goal: %.2f m",
            distance
        )

        if distance <= self.arrival_threshold and not self.goal_reached:
            self.goal_reached = True
            
            if self.mission_state == MissionState.SEARCH:
                self.advance_to_next_point()
            elif self.mission_state == MissionState.APPROACH:
                # APPROACH是什么意思？就是中断search，approach target
                self.mission_state = MissionState.TARGET_HOLD
                self.hold_started_at = rospy.Time.now()
                # 到地方，开始计时
                rospy.loginfo("Target reached, holding")

                

    def target_callback(self, msg):
        if self.mission_state != MissionState.SEARCH:
            return

        if not self.candidate_policy.accept(msg):
            return 

        self.search_context = SearchContext(
            waypoint_index=self.search_policy.current_index,
            interrupted_goal=self.search_policy.current_waypoint,
            search_altitude=self.search_policy.altitude
        )
         
        self.selected_target = msg

        rospy.loginfo(
            "Accepted new target: id=%d, class=%s, state=%d, "
            "position=(%.2f, %.2f, %.2f)",
            msg.id,
            msg.class_name,
            msg.state,
            # msg.map_valid,
            msg.map_point.x,
            msg.map_point.y,
            msg.map_point.z
        )

        self.mission_state = MissionState.APPROACH
        self.publish_goal(
            self.selected_target.map_point.x,
            self.selected_target.map_point.y,
            # 直接用msg语义不明
            self.approach_altitude
            # 应当已manager设定的安全高度靠近目标，不可直接使用TargetCandidate
        )

    def mission_timer_callback(self, event):
        if self.mission_state != MissionState.TARGET_HOLD:
            return
        if self.hold_started_at is None:
            return

        elapsed = (rospy.Time.now() - self.hold_started_at).to_sec()
        if elapsed < self.hold_duration:
            return

        self.hold_started_at = None
        self.resume_search()
        # 这个函数要求停够一定时间才能恢复search状态，
        # 后续这个逻辑可以靠是否投放来完成

    def distance_to_goal(self):
        if self.current_pose is None:
            return float("inf")

        current = self.current_pose.pose.position
        target = self.current_goal.pose.position

        dx = target.x - current.x
        dy = target.y - current.y
        dz = target.z - current.z

        return math.sqrt(dx**2 + dy**2 + dz**2)

    def publish_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = "camera_init"
        # 坐标系
        # goal.header.stamp = rospy.Time.now()
        # 时间戳
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0

        self.current_goal = goal
        self.goal_reached = False
        self.current_goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.current_goal)
        # 发布点
        rospy.loginfo("New goal: x=%.2f, y=%.2f, z=%.2f",
        x, y, z)

    def advance_to_next_point(self):
        waypoint = self.search_policy.advance()

        if waypoint is None:
            self.mission_state = MissionState.SEARCH_COMPLETE
            rospy.loginfo("Search route completed")
            return

        self.publish_goal(*waypoint.as_tuple())

    def resume_search(self):
        if self.search_context is None:
            rospy.logwarn("Cannot resume search: no saved search context")
            return
        
        self.mission_state = MissionState.RESUME_SEARCH
        waypoint = self.search_policy.restore(self.search_context.waypoint_index)

        if waypoint is None:
            self.mission_state = MissionState.SEARCH_COMPLETE
            self.search_context = None
            self.selected_target = None
            self.hold_started_at = None
            rospy.loginfo("Search was already completed")
            return

        self.publish_goal(*waypoint.as_tuple())
        self.search_context = None
        self.selected_target = None
        self.mission_state = MissionState.SEARCH

        rospy.loginfo(
            "Search resumed at waypoint index %d",
            self.search_policy.current_index,
        )




def main():
    manager = TargetSearchManager()
    rospy.spin()



if __name__ == "__main__":
    main()    


    
