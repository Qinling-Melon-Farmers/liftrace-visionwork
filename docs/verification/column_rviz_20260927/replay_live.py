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
frame.resize(1900,1100)
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
import ctypes
import sip
lib=ctypes.CDLL("/home/xhj/rviz_capture.so")
lib.capture.argtypes=[ctypes.c_void_p,ctypes.c_char_p]
class Capture:
    def save(self,path):lib.capture(int(sip.unwrapinstance(mgr)),path.encode())
capture=Capture()
view=mgr.getViewManager().getCurrent()
print("VIEW",view.getClassId(),flush=True)
for name,val in [("Distance",25.),("Yaw",0.8),("Pitch",0.65)]:
    view.subProp(name).setValue(val)
view.subProp("Focal Point").subProp("X").setValue(3.)
view.subProp("Focal Point").subProp("Y").setValue(0.)
view.subProp("Focal Point").subProp("Z").setValue(1.)
def tick(n=30):
    for _ in range(n):
        app.processEvents();time.sleep(.05)
for scene in ["live","live_slice"]:
    for pub,key in zip(pubs[:2],["raw","inflated"]):
        path=R/"logs/column_rviz_20260927"/(scene+"_"+key+".xyz")
        pts=[list(map(float,line.split())) for line in path.read_text().replace("\\n","\n").splitlines()[::5] if line.strip()]
        pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id="map"),pts))
    for mode,enabled in [("raw",[1,0,0]),("inflated",[1,1,0]),("overlay",[1,1,0])]:
        for d,e in zip(displays,enabled):d.setEnabled(bool(e))
        frame.setWindowTitle(scene+" | "+mode+" | white=raw blue=3D orange=added columns")
        tick()
        capture.save(str(out/(scene+"_"+mode+".png")))
    view.subProp("Pitch").setValue(1.56)
    tick()
    capture.save(str(out/(scene+"_top.png")))
    view.subProp("Pitch").setValue(.65)
frame.saveDisplayConfig(str(out/"columns.rviz"))
print("CAPTURE_DONE",flush=True)
frame.close()
rospy.signal_shutdown("done")