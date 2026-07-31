#!/usr/bin/env python3
"""Assert that Phase-D shadow publishers cannot feed patrol_control."""

import sys
import time

import rosgraph
import rospy


EXPECTED_SHADOW_PUBLISHERS = {
    "/uav_vision_shadow/selected_target": "/target_memory",
    "/uav_vision_shadow/drop_offset": "/drop_aligner",
    "/uav_vision_shadow/drop_ready": "/drop_aligner",
    "/uav_vision_shadow/release_evidence": "/drop_aligner",
    "/uav_vision_shadow/legacy/yolo_detect": "/detect_compat_bridge",
}

PROTECTED_GLOBAL_PUBLISHERS = {
    "/uav_vision/selected_target": "/target_memory",
    "/uav_vision/drop_offset": "/drop_aligner",
    "/uav_vision/drop_ready": "/drop_aligner",
    "/uav_vision/release_evidence": "/drop_aligner",
    "/yolo_detect": "/detect_compat_bridge",
}


def _mapping(entries):
    return {name: set(nodes) for name, nodes in entries}


def _contract_ok(master):
    publishers, subscribers, _services = master.getSystemState()
    publishers = _mapping(publishers)
    subscribers = _mapping(subscribers)
    missing = []
    leaked = []
    for topic, node in EXPECTED_SHADOW_PUBLISHERS.items():
        if node not in publishers.get(topic, set()):
            missing.append("{}<-{}".format(topic, node))
    for topic, node in PROTECTED_GLOBAL_PUBLISHERS.items():
        if node in publishers.get(topic, set()):
            leaked.append("{}<-{}".format(topic, node))
    if "/patrol_control" not in subscribers.get("/uav_vision/selected_target", set()):
        missing.append("/uav_vision/selected_target->/patrol_control")
    for topic in (
        "/uav_vision_shadow/selected_target",
        "/uav_vision_shadow/drop_offset",
        "/uav_vision_shadow/drop_ready",
        "/uav_vision_shadow/release_evidence",
    ):
        if "/patrol_control" in subscribers.get(topic, set()):
            leaked.append("{}->/patrol_control".format(topic))
    return not missing and not leaked, missing, leaked


def main():
    rospy.init_node("shadow_contract_assertion")
    master = rosgraph.Master(rospy.get_name())
    deadline = time.monotonic() + float(rospy.get_param("~timeout_sec", 45.0))
    last_missing, last_leaked = [], []
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            passed, last_missing, last_leaked = _contract_ok(master)
            if passed:
                rospy.loginfo("V-SIM-02 shadow isolation contract PASS")
                return 0
        except Exception as error:
            last_missing = [str(error)]
        time.sleep(0.25)
    rospy.logerr(
        "V-SIM-02 shadow isolation contract FAIL missing=%s leaked=%s",
        last_missing, last_leaked,
    )
    return 5


if __name__ == "__main__":
    sys.exit(main())
