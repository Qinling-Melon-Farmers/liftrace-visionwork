"""Read-only rosbag child owned by a board trial; never publishes control data."""
import json,os,signal,subprocess,time,shutil
from pathlib import Path

def topics_for(settings):
    camera=settings.get("compressed_image_topic",settings.get("image_topic","/camera/image_raw")+"/compressed")
    info=settings.get("camera_info_topic","/camera/camera_info")
    return list(dict.fromkeys([camera,info,
        "/tf","/tf_static","/rosout_agg","/Odometry","/mavros/vision_pose/pose",
        "/mavros/local_position/pose","/mavros/local_position/odom",
        "/mavros/local_position/velocity_local","/mavros/state","/mavros/extended_state",
        "/mavros/statustext/recv","/mavros/setpoint_position/local","/mavros/setpoint_raw/target_local",
        "/navigation/local_pose","/navigation/local_odom","/navigation/setpoint_mission",
        "/fastplanner/goal","/planning/goal_status","/planning/bspline","/planning/pos_cmd",
        "/planning/replan","/planning/new","/planning_vis/trajectory",
        "/planning/progress",
        "/navigation/mission_command_raw","/navigation/mission_result","/navigation/mission_status",
        "/navigation/planner_bridge_status","/mission/command","/mission/control_ready","/detect/waypoint_mark_point","/detect/land_mark_point","/detect/point_class",
        "/uav_vision/detections","/uav_vision/detections_resolved","/uav_vision/detections_refined",
        "/uav_vision/detections_mapped","/uav_vision/navigation_hints","/uav_vision/targets",
        "/uav_vision/selected_target","/uav_vision/align_mode","/uav_vision/alignment_target_context","/uav_vision/release_evidence_context",
        "/uav_vision/drop_offset","/uav_vision/drop_ready","/uav_vision/release_evidence",
        "/uav_vision/perf","/mission/release_permission","/mission/release_permission_active",
        "/mission/release_result","/uav_high_view/probe_status",
        "/board_trials/run_metadata","/board_trials/mock_release","/board_trials/landing_context","/board_trials/auto_land_status",
        "/freedom/static_pointcloud","/sdf_map/occupancy","/sdf_map/occupancy_inflate"]))

def summarize(directory,required):
    import rosbag
    root=Path(directory);counts={};errors=[]
    bags=sorted(root.glob("flight_debug*.bag"))
    for path in bags:
        try:
            with rosbag.Bag(str(path)) as bag:
                for topic,info in bag.get_type_and_topic_info().topics.items():
                    counts[topic]=counts.get(topic,0)+info.message_count
        except Exception as error:errors.append(path.name+": "+str(error))
    active=[p.name for p in root.glob("flight_debug*.bag.active")]
    missing=[t for t in required if not counts.get(t)]
    result=dict(status="PASS" if bags and counts and not errors and not active and not missing else "INCOMPLETE",
                bags=[p.name for p in bags],active=active,topic_counts=counts,
                missing_required=missing,errors=errors,total_bytes=sum(p.stat().st_size for p in bags))
    (root/"bag_recording.json").write_text(json.dumps(result,indent=2))
    return result

class TrialBag:
    def __init__(self,directory,settings,env):
        self.out=Path(directory);self.settings=settings;self.env=env
        self.topics=topics_for(settings);self.process=None;self.stream=None
        self.started=None;self.closed=False;self.last_check=0.;self.notified=False
        self.stop_reason="normal_shutdown"
    def start(self):
        if shutil.disk_usage(self.out).free<2*1024**3:raise RuntimeError("Less than 2GiB available for bag")
        self.stream=(self.out/"rosbag.log").open("w")
        self.command=["rosbag","record","--lz4","--split","--size=512","--buffsize=128",
                      "--min-space=2G","--repeat-latched","-O",str(self.out/"flight_debug.bag"),
                      *self.topics,"__name:=board_trial_bag"]
        (self.out/"bag_topics.json").write_text(json.dumps(dict(topics=self.topics,
            command=self.command,camera_transport="compressed",raw_lidar_recorded=False),indent=2))
        try:
            self.process=subprocess.Popen(self.command,env=self.env,stdout=self.stream,stderr=subprocess.STDOUT,start_new_session=True)
            self.started=time.monotonic()
            time.sleep(.5)
            if self.process.poll() is not None:raise RuntimeError("rosbag exited during startup; inspect rosbag.log")
        except Exception:
            self.stream.close()
            raise
    def check(self):
        if self.closed or time.monotonic()-self.last_check<5:return
        self.last_check=time.monotonic()
        if self.process.poll() is not None:
            if not self.notified:
                print("RECORDING_WARNING: rosbag exited; flight control remains unchanged",flush=True)
                self.notified=True
            return
        if shutil.disk_usage(self.out).free<2*1024**3 or time.monotonic()-self.started>900:
            self.stop_reason="recording_storage_or_time_budget"
            print("RECORDING_WARNING: closing only rosbag at storage/time limit",flush=True)
            try:self.close()
            except Exception as error:print("RECORDING_WARNING: bag finalization failed: "+str(error),flush=True)
    def close(self):
        if self.closed:return
        self.closed=True
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.stop_reason="recorder_shutdown_timeout"
                os.killpg(self.process.pid,signal.SIGTERM)
                try:self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=5)
        if self.stream:self.stream.close()
        required=[self.topics[0],self.topics[1],"/mavros/local_position/pose","/uav_vision/detections"]
        result=summarize(self.out,required);result["stop_reason"]=self.stop_reason
        result["recorder_exit_code"]=self.process.returncode if self.process else None
        if self.stop_reason!="normal_shutdown" or result["recorder_exit_code"] not in (0,130,-signal.SIGINT):
            result["status"]="INCOMPLETE"
        (self.out/"bag_recording.json").write_text(json.dumps(result,indent=2))
        print("BAG_CLOSED:",result["status"],result["bags"],flush=True)
        return result
