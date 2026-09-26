"""Export additional ROS1 topics for the 2026-09-26 offline review."""
import argparse
import json
from pathlib import Path
import rosbag

TOPICS = ["/mavros/extended_state", "/camera/camera_info",
          "/navigation/planner_bridge_status", "/mission/release_permission",
          "/mission/release_commitment_evidence",
          "/uav_vision/alignment_target_context", "/uav_vision/drop_ready",
          "/mission/command"]

def plain(m):
    if hasattr(m, "__slots__"):
        return {k: plain(getattr(m,k)) for k in m.__slots__}
    if isinstance(m,(tuple,list)):
        return [plain(v) for v in m]
    return m

if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag",type=Path);p.add_argument("output",type=Path)
    a=p.parse_args()
    rows={k:[] for k in TOPICS}
    with rosbag.Bag(str(a.bag)) as bag:
        start=bag.get_start_time()
        for topic,msg,t in bag.read_messages(topics=TOPICS):
            if topic=="/camera/camera_info" and rows[topic]:
                continue
            rows[topic].append(dict(t=t.to_sec()-start,m=plain(msg)))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(rows))
    print({k:len(v) for k,v in rows.items()})
