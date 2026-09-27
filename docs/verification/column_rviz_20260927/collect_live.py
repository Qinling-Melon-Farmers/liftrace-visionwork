import os,sys,time
os.environ["QT_QPA_PLATFORM"]="xcb"
import rospy
from rviz import bindings as rviz
from python_qt_binding.QtWidgets import QApplication
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from pathlib import Path
R=Path("/home/xhj/liftrace-worktrees/r2026-high-view-search")
out=R/"docs/verification/column_rviz_20260927"
app=QApplication(sys.argv)
rospy.init_node("column_rviz_review",anonymous=True,disable_signals=True)
frame=rviz.VisualizationFrame()
frame.setSplashPath("")
frame.initialize()
frame.resize(1400,1000)
frame.show()
mgr=frame.getManager()
mgr.setFixedFrame("map")
grid=mgr.createDisplay("rviz/Grid","Ground grid",True)
grid.subProp("Cell Size").setValue(1.)
pubs=[]
displays=[]
for key,color in [("raw","225;225;225"),("inflated","50;160;255"),("columns","255;90;40")]:
    pub=rospy.Publisher("/review/"+key,PointCloud2,queue_size=1,latch=True)
    disp=mgr.createDisplay("rviz/PointCloud2",key,True)
    disp.subProp("Topic").setValue("/review/"+key)
    disp.subProp("Style").setValue("Points")
    disp.subProp("Size (Pixels)").setValue(3)
    disp.subProp("Color Transformer").setValue("FlatColor")
    from python_qt_binding.QtGui import QColor
    rgb=list(map(int,color.split(";")))
    disp.subProp("Color").setValue(QColor(*rgb))
    pubs.append(pub);displays.append(disp)
view=mgr.getViewManager().getCurrent()
print("VIEW",view.getClassId(),flush=True)
for name,val in [("Distance",16.),("Yaw",0.8),("Pitch",0.65)]:
    view.subProp(name).setValue(val)
view.subProp("Focal Point").subProp("X").setValue(3.)
view.subProp("Focal Point").subProp("Y").setValue(0.)
view.subProp("Focal Point").subProp("Z").setValue(1.)
def tick(n=30):
    for _ in range(n):
        app.processEvents();time.sleep(.05)

import json
latest={}
def cb(msg,key):latest[key]=msg
subs=[rospy.Subscriber(topic,PointCloud2,cb,callback_args=key,queue_size=1) for key,topic in [("raw","/freedom/static_pointcloud"),("inflated","/sdf_map/occupancy_inflate")]]
deadline=time.monotonic()+240
last=0
captures=0
while time.monotonic()<deadline:
    tick(2)
    if "inflated" not in latest or "raw" not in latest:continue
    if time.monotonic()-last<12:continue
    last=time.monotonic()
    for pub,key in zip(pubs,["raw","inflated","columns"]):
        if key not in latest:continue
        msg=latest[key]
        mgr.setFixedFrame(msg.header.frame_id)
        pub.publish(msg)
        pts=list(point_cloud2.read_points(msg,field_names=("x","y","z"),skip_nans=True))
        data=R/"logs/column_rviz_20260927"/("live_"+key+".xyz")
        data.write_text("\\n".join(" ".join(map(str,pt)) for pt in pts))
    for mode,en in [("raw",[1,0,0]),("inflated",[0,1,0]),("overlay",[1,1,0])]:
        for d,e in zip(displays,en):d.setEnabled(bool(e))
        frame.setWindowTitle("LIVE Gazebo seed31 columns ON | "+mode)
        tick(8)
    view.subProp("Pitch").setValue(1.56)
    tick(8)
    view.subProp("Pitch").setValue(.65)
    captures+=1
    (out/"live_capture.json").write_text(json.dumps({"captures":captures,"stamp":latest["inflated"].header.stamp.to_sec(),"frame":latest["inflated"].header.frame_id,"raw_points":latest["raw"].width*latest["raw"].height,"inflated_points":latest["inflated"].width*latest["inflated"].height,"columns_enabled":rospy.get_param("/fast_planner_node/sdf_map/horizontal_avoidance/enabled",None)},indent=2))
    print("LIVE_CAPTURE",captures,flush=True)
    if captures>=4:break
frame.saveDisplayConfig(str(out/"live.rviz"))
frame.close()
rospy.signal_shutdown("done")
print("LIVE_DONE",captures,flush=True)
