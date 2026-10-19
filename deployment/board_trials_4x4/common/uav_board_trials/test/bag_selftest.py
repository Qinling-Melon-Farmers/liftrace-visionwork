"""Short isolated recorder test. No MAVROS, controller, sensor driver or actuator."""
import os,sys,subprocess,signal,time,json,socket,xmlrpc.client
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
out=ROOT/"deployment_results"/("recording_selftest_"+time.strftime("%H%M%S"))
out.mkdir(parents=True,exist_ok=False)
for name in ("rosmaster","roslaunch","mavros_node","patrol_control"):
    if subprocess.run(["pgrep","-x",name],stdout=subprocess.DEVNULL).returncode==0:
        raise SystemExit("Selftest refuses an existing ROS application: "+name)
os.environ["ROS_MASTER_URI"]="http://127.0.0.1:11319"
os.environ["ROS_IP"]="127.0.0.1";os.environ.pop("ROS_HOSTNAME",None)
os.environ["ROS_LOG_DIR"]=str(out/"roslog")
corelog=(out/"core.log").open("w")
core=subprocess.Popen(["roscore","-p","11319"],stdout=corelog,stderr=subprocess.STDOUT,start_new_session=True)
recorder=None
try:
    socket.setdefaulttimeout(2)
    ready=False
    for _ in range(30):
        if core.poll() is not None:raise RuntimeError("selftest core exited")
        try:
            ready=xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getPid("/selftest")[0]==1
            if ready:break
        except Exception:pass
        time.sleep(.2)
    assert ready,"selftest master timeout"
    import rospy,cv2,numpy as np
    from sensor_msgs.msg import CompressedImage,CameraInfo
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from uav_vision.msg import TargetDetectionArray
    from trial_bag import TrialBag
    rospy.init_node("board_recording_selftest",disable_signals=True)
    publishers=[
        rospy.Publisher("/camera/image_raw/compressed",CompressedImage,queue_size=2),
        rospy.Publisher("/camera/camera_info",CameraInfo,queue_size=2),
        rospy.Publisher("/mavros/local_position/pose",PoseStamped,queue_size=2),
        rospy.Publisher("/uav_vision/detections",TargetDetectionArray,queue_size=2)]
    static=rospy.Publisher("/tf_static",TFMessage,queue_size=1,latch=True)
    tf=TransformStamped();tf.header.frame_id="SELFTEST_map";tf.child_frame_id="SELFTEST_camera";tf.transform.rotation.w=1.
    static.publish(TFMessage([tf]))
    metadata=rospy.Publisher("/board_trials/run_metadata",String,queue_size=1,latch=True)
    metadata.publish(String(data='{"kind":"recorder_selftest_not_flight"}'))
    recorder=TrialBag(out,{},os.environ.copy());recorder.start()
    deadline=time.monotonic()+10
    while not all(p.get_num_connections() for p in publishers) and time.monotonic()<deadline:time.sleep(.1)
    assert all(p.get_num_connections() for p in publishers),"recorder subscriptions not ready"
    encoded=cv2.imencode(".jpg",np.full((24,32,3),120,np.uint8))[1].tobytes()
    for _ in range(25):
        stamp=rospy.Time.now()
        image=CompressedImage();image.header.stamp=stamp;image.header.frame_id="SELFTEST_camera";image.format="jpeg";image.data=encoded
        info=CameraInfo();info.header=image.header;info.width=32;info.height=24;info.K=[30.,0.,16.,0.,30.,12.,0.,0.,1.]
        pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id="SELFTEST_map";pose.pose.orientation.w=1.
        det=TargetDetectionArray();det.header=image.header;det.source="recorder_selftest_not_inference"
        for pub,msg in zip(publishers,[image,info,pose,det]):pub.publish(msg)
        time.sleep(.1)
    time.sleep(.2)
    result=recorder.close()
    assert result["status"]=="PASS",result
    assert result["topic_counts"].get("/tf_static",0)>0
    assert result["topic_counts"].get("/board_trials/run_metadata",0)>0
    import rosbag
    with rosbag.Bag(str(out/result["bags"][0])) as bag:
        msg=next(bag.read_messages(topics=["/camera/image_raw/compressed"])).message
        assert cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR).shape==(24,32,3)
    result["jpeg_decode"]="PASS";result["kind"]="isolated_synthetic_recorder_test_no_flight"
    (out/"selftest.json").write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
finally:
    try:
        if recorder is not None:recorder.close()
    finally:
        if core.poll() is None:
            os.killpg(core.pid,signal.SIGINT)
            try:core.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(core.pid,signal.SIGTERM);core.wait(timeout=5)
        corelog.close()
print("SELFTEST_OUTPUT",out)
