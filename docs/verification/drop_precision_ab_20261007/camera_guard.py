#!/usr/bin/env python3
"""Evaluation-only camera check and bounded evidence collection.

No truth inputs, pose commands, arming or release calls. The existing mission
start Trigger is forwarded only after camera/parameter checks pass. Any mismatch
exits this required node, so roslaunch and the owning sim_run wrapper shut down.
"""
import json
import threading
import time
from pathlib import Path
from preflight import camera_inputs, check_camera_info, check_parameters, flatten


def plain(value):
    if hasattr(value,'__slots__'):
        return {key:plain(getattr(value,key)) for key in value.__slots__}
    if isinstance(value,(list,tuple)):return [plain(x) for x in value]
    return value


def main():
    import rospy
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Log
    from std_srvs.srv import Trigger, TriggerResponse
    rospy.init_node('drop_precision_camera_guard')
    run=Path(rospy.get_param('~run_dir')).resolve()
    out=run/'camera_evidence';out.mkdir(exist_ok=False)
    contract_path=Path(rospy.get_param('~contract')).resolve()
    contract,sdf=camera_inputs(contract_path.parent)
    if sdf != Path(rospy.get_param('~vehicle_sdf')).resolve():
        raise ValueError('Camera observer and actual spawn SDF differ')
    (out/'expected_contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    (out/'spawn_input.sdf').write_bytes(sdf.read_bytes())
    timeout=float(rospy.get_param('~startup_wall_timeout',90.0))
    limit=int(rospy.get_param('~max_images',6))
    if not 1<=limit<=6:raise ValueError('Only 1..6 startup/review images may be saved')
    lock=threading.RLock();started=time.monotonic();bridge=CvBridge()
    state=dict(status='WAITING',info=None,image=None,failed=None,images=0,first_image_stamp=None)
    frame_log=(out/'frames.jsonl').open('x')
    diag_log=(run/'exact_rejection_details.jsonl').open('x')
    schedule=[0.,1.,4.,15.,30.,50.]

    def write_status(status,reason=None):
        record=dict(status=status,reason=reason,wall=time.time(),ros_s=rospy.get_time(),
            expected_profile=contract['profile_id'],camera_info=state['info'],
            first_image=state['image'],saved_images=state['images'],
            note='Runtime publication and spawn-input consistency. Images/TF support offline rendered-ray checks; no visual calibration proof is inferred.')
        dest=run/'camera_check.json';temp=dest.with_suffix('.tmp')
        temp.write_text(json.dumps(record,indent=2)+'\n');temp.replace(dest)

    def fail(reason):
        with lock:
            if state['failed'] is None:
                state['failed']=str(reason);state['status']='FAIL'
                write_status('FAIL',str(reason));rospy.logerr('Camera check FAIL: %s',reason)

    def maybe_pass():
        if state['failed'] or state['status']=='PASS' or not state['info'] or not state['image']:return
        check_parameters(flatten(rospy.get_param('/')))
        state['status']='PASS';write_status('PASS')
        rospy.loginfo('Camera check PASS: centered K/D0, image size/frame, exactON/zero/NMSfalse/record-only verified')

    def camera(msg):
        try:
            data=plain(msg);check_camera_info(data,contract)
            with lock:
                if state['info'] is None:
                    state['info']=data;(out/'first_camera_info.json').write_text(json.dumps(data,indent=2)+'\n')
                maybe_pass()
        except Exception as error:fail(error)

    def image(msg):
        try:
            if [msg.width,msg.height]!=[contract['width'],contract['height']] or msg.header.frame_id!=contract['frame_id']:
                raise ValueError('Actual image dimensions/frame do not match CameraInfo/SDF contract')
            with lock:
                if state['failed']:return
                stamp=msg.header.stamp.to_sec()
                if state['first_image_stamp'] is None:state['first_image_stamp']=stamp
                if state['images']<limit and stamp-state['first_image_stamp']>=schedule[state['images']]:
                    path=out/f'frame_{state["images"]:02d}.png'
                    pixels=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                    if not cv2.imwrite(str(path),pixels):raise ValueError('Could not save camera evidence')
                    meta=dict(file=path.name,source_ros_s=stamp,receipt_ros_s=rospy.get_time(),
                        width=msg.width,height=msg.height,frame_id=msg.header.frame_id,
                        camera_info_source=state['info']['header']['stamp'] if state['info'] else None)
                    frame_log.write(json.dumps(meta)+'\n');frame_log.flush();state['images']+=1
                    if state['image'] is None:state['image']=meta
                maybe_pass()
        except Exception as error:fail(error)

    def static_tf(msg):
        path=out/'first_tf_static.json'
        with lock:
            if not path.exists():path.write_text(json.dumps(plain(msg),indent=2)+'\n')

    def diagnostic(msg):
        text=msg.msg.lower()
        if ('exact' in text and any(k in text for k in ('reject','capture','hold','invalid','stop','commit'))) or 'commitment_position_drift' in text:
            with lock:
                diag_log.write(json.dumps(dict(source_ros_s=msg.header.stamp.to_sec(),receipt_ros_s=rospy.get_time(),node=msg.name,level=msg.level,text=msg.msg))+'\n');diag_log.flush()

    upstream=rospy.ServiceProxy(rospy.get_param('~upstream_start_service'),Trigger)
    def start(_request):
        with lock:
            if state['status']!='PASS':return TriggerResponse(False,'camera_check_'+state['status'].lower())
        try:return upstream()
        except rospy.ServiceException as error:return TriggerResponse(False,str(error))

    rospy.Subscriber(rospy.get_param('~camera_info_topic'),CameraInfo,camera,queue_size=1)
    rospy.Subscriber(rospy.get_param('~image_topic'),Image,image,queue_size=1,buff_size=4000000)
    rospy.Subscriber('/tf_static',TFMessage,static_tf,queue_size=10)
    rospy.Subscriber('/rosout_agg',Log,diagnostic,queue_size=100)
    service=rospy.Service('~start_mission_checked',Trigger,start)
    write_status('WAITING')
    try:
        while not rospy.is_shutdown():
            if state['failed']:return 2
            if state['status']!='PASS' and time.monotonic()-started>timeout:
                fail('camera_startup_wall_timeout');return 2
            time.sleep(.1)
    finally:
        frame_log.close();diag_log.close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
