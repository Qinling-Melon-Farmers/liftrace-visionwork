#!/usr/bin/python3
import copy

import rospy
import source_modules
from navigation_frame_adapter import FrameAdapter
from single_drop_core import bounded_z


class BoundedFrameAdapter(FrameAdapter):
    def callback(self, message, args):
        if args[0] == self.mission:
            try:
                message = copy.deepcopy(message)
                message.pose.position.z = bounded_z(message.pose.position.z)
            except ValueError as error:
                rospy.logerr_throttle(2.0, str(error))
                return
        super().callback(message, args)


if __name__ == "__main__":
    rospy.init_node("navigation_frame_adapter")
    BoundedFrameAdapter()
    rospy.spin()
