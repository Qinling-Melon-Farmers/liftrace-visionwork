"""Exercise actual PT/RKNN message building without GPU/NPU or a ROS master."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from sensor_msgs.msg import Image
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
NAMES = {0:"bridge", 1:"panzer", 2:"pillbox", 3:"tent", 4:"tank", 5:"red_cross"}

def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/"scripts"/(name+".py"))
    m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m

PT=load("target_detector")
RK=load("target_detector_rknn")

class Boxes:
    cls=np.arange(7)  # includes an unknown class to check rejection
    conf=np.ones(7)*.9
    xyxy=Mock()
    xyxy.__getitem__=Mock(return_value=SimpleNamespace(
        cpu=lambda:SimpleNamespace(numpy=lambda:np.array([0,0,10,10]))))
    def __len__(self): return 7

class DetectorClassProfileTest(unittest.TestCase):
    def make_pt(self, profile=None):
        params={} if profile is None else {"~class_profile":profile}
        with tempfile.NamedTemporaryFile() as f:
            params["~model_path"]=f.name
            model=Mock(names=NAMES)
            model.predict.return_value=[SimpleNamespace(boxes=Boxes())]
            with patch.object(PT.rospy,"init_node"),patch.object(PT.rospy,"get_param",
                    side_effect=lambda key,default=None:params.get(key,default)), \
                 patch.object(PT.rospy,"Publisher"),patch.object(PT.rospy,"Subscriber"), \
                 patch.object(PT,"YOLO",return_value=model),patch.object(PT.rospy,"loginfo"):
                node=PT.TargetDetector()
        node._publish_perf=Mock()
        return node

    def test_pt_default_filters_tank_and_keeps_red_cross_id_five(self):
        node=self.make_pt()
        self.assertEqual(node._class_profile,"r2026")
        node._on_image(Image(height=12,width=12,encoding="bgr8",step=36,data=bytes(432)))
        arr=node._detections_pub.publish.call_args[0][0]
        self.assertEqual([d.class_name for d in arr.detections],
                         ["bridge","panzer","pillbox","tent","red_cross"])
        self.assertEqual(arr.completed_sources,["target_detector"])

    def test_pt_explicit_full_keeps_historical_tank(self):
        node=self.make_pt("full")
        node._on_image(Image(height=12,width=12,encoding="bgr8",step=36,data=bytes(432)))
        self.assertEqual([d.class_name for d in node._detections_pub.publish.call_args[0][0].detections],
                         list(NAMES.values()))

    def test_rknn_same_filter_without_renumbering(self):
        for profile in ("r2026","full"):
            with self.subTest(profile=profile):
                node=RK.TargetDetectorRKNN.__new__(RK.TargetDetectorRKNN)
                node._class_profile,node._allowed_classes=RK.resolve_class_profile(profile)
                raw=[{"class_id":i,"score":.9,"bbox":[0,0,10,10]} for i in range(7)]
                arr=node._build_msg(Header(),raw,SimpleNamespace(names=NAMES))
                expected=[c for c in NAMES.values() if c!="tank" or profile=="full"]
                self.assertEqual([d.class_name for d in arr.detections],expected)
                self.assertEqual(arr.detections[-1].class_name,"red_cross")
                self.assertTrue(all(not d.geometry_verified for d in arr.detections))

    def test_rknn_retired_only_frame_still_completes_source(self):
        node=RK.TargetDetectorRKNN.__new__(RK.TargetDetectorRKNN)
        node._class_profile,node._allowed_classes=RK.resolve_class_profile("r2026")
        arr=node._build_msg(Header(),[{"class_id":4,"score":.99,"bbox":[0,0,10,10]}],
                            SimpleNamespace(names=NAMES))
        self.assertEqual(arr.detections,[])
        self.assertEqual(arr.completed_sources,["target_detector"])

    def test_rknn_does_not_load_split_tank_in_2026(self):
        params={"~tank_model_path":"retired.rknn", "~tank_metadata_path":"retired.yaml"}
        handles=[]
        def handle(model,metadata,tag):
            handles.append((model,metadata,tag))
            return SimpleNamespace(available=lambda:tag=="unified")
        with patch.object(RK.rospy,"init_node"),patch.object(RK.rospy,"get_param",
                side_effect=lambda key,default=None:params.get(key,default)), \
             patch.object(RK.rospy,"Publisher"),patch.object(RK.rospy,"Subscriber"), \
             patch.object(RK,"_RknnHandle",side_effect=handle),patch.object(RK.rospy,"loginfo"):
            node=RK.TargetDetectorRKNN()
        self.assertEqual(handles[-1],("","","tank"))
        self.assertEqual(node._class_profile,"r2026")

    def test_unknown_profile_fails_before_runtime(self):
        with self.assertRaises(ValueError): self.make_pt("typo")

    def test_launch_threads_profile_to_both_backends(self):
        count=0
        for name in ("phase_d.launch","phase_d_board.launch","video_replay_annotation.launch"):
            root=ET.parse(ROOT/"launch"/name).getroot()
            arg=next(a for a in root.findall("arg") if a.attrib.get("name")=="class_profile")
            self.assertEqual(arg.attrib["default"],"r2026")
            for n in root.findall(".//node"):
                if n.attrib.get("type") not in ("target_detector.py","target_detector_rknn.py"):continue
                p=next(p for p in n.findall("param") if p.attrib.get("name")=="class_profile")
                self.assertEqual(p.attrib["value"],"$(arg class_profile)")
                count+=1
        self.assertEqual(count,4)
        for name in ("target_detector.yaml","target_detector_rknn.yaml"):
            self.assertEqual(yaml.safe_load((ROOT/"config"/name).read_text())["class_profile"],"r2026")

if __name__=="__main__": unittest.main()