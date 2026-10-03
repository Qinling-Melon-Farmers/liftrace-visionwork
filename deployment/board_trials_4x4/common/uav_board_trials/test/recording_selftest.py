"""Synthetic camera codec/annotation check, without a ROS master or hardware."""
from pathlib import Path
import tempfile,json
from unittest.mock import patch,Mock
import cv2,numpy as np,rospy
from sensor_msgs.msg import Image
from uav_vision.msg import TargetDetection,TargetDetectionArray
from trial_recorder import Recorder
out=Path(tempfile.mkdtemp(prefix='board_recording_synthetic_'))
with patch('rospy.get_param',side_effect=lambda k,d=None: str(out) if k=='~directory' else d),patch('rospy.Subscriber',return_value=Mock()),patch('rospy.Timer',return_value=Mock()),patch('rospy.on_shutdown'),patch('rospy.Time.now',return_value=rospy.Time.from_sec(10.25)):
    r=Recorder();im=Image();im.header.stamp=rospy.Time.from_sec(10.);im.height=360;im.width=640;im.encoding='bgr8';im.step=1920;im.data=np.zeros((360,640,3),np.uint8).tobytes();r.image(im)
    det=TargetDetection();det.header=im.header;det.class_name='panzer';det.class_confidence=.9;det.geometry_confidence=.8;det.roi.x_offset=100;det.roi.y_offset=100;det.roi.width=80;det.roi.height=60;det.center_px.x=140;det.center_px.y=130;det.center_refined=True;det.map_valid=True
    array=TargetDetectionArray();array.header=im.header;array.detections=[det];r.detections('yolo',array);r.detections('mapped',array)
    r.frame(None);r.frame(None);r.close()
cap=cv2.VideoCapture(str(out/'camera_annotated.mp4'));ok,frame=cap.read();count=cap.get(cv2.CAP_PROP_FRAME_COUNT);cap.release();assert ok and count==2 and frame.shape[:2]==(444,640) and frame.max()>100
result=dict(synthetic_not_flight=True,output=str(out),frames=count,shape=list(frame.shape),video_decode=True)
root=Path(__file__).resolve().parents[5];(root/'deployment/board_trials_4x4/validation_recording.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
