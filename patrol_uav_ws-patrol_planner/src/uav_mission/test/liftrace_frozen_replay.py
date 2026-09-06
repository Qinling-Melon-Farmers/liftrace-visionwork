#!/usr/bin/env python3
import os,time,json,sys
from pathlib import Path
import numpy as np,rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from plan_manage.msg import Bspline,PlannerStatus
rospy.init_node('frozen_map_fixture')
data=np.load(rospy.get_param('~cloud_file'))
out=Path(os.environ['SIM_RUN_DIR'])
goal=np.array(rospy.get_param('~goal'),dtype=float); received=[];events=[]
rospy.Subscriber('/planning/bspline',Bspline,lambda m:received.append(m),queue_size=1)
rospy.Subscriber('/planning/goal_status',PlannerStatus,lambda m:events.append(m.reason),queue_size=100)
pcpub=rospy.Publisher('/freedom/static_pointcloud',PointCloud2,queue_size=1)
odpub=rospy.Publisher('/mavros/local_position/odom',Odometry,queue_size=1)
gpub=rospy.Publisher('/fastplanner/goal',PoseStamped,queue_size=1,latch=True)
cloud=point_cloud2.create_cloud_xyz32(Header(frame_id='camera_init'),data['static'].astype(np.float32))
odom=Odometry();odom.header.frame_id='camera_init';odom.child_frame_id='base_link'
odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=data['pose']
odom.pose.pose.orientation.w=1
started=time.monotonic();last_cloud=0;sent=False
while not rospy.is_shutdown() and time.monotonic()-started<float(rospy.get_param('~timeout',25.0)):
    now=rospy.Time.now();odom.header.stamp=now;odpub.publish(odom)
    if time.monotonic()-last_cloud>.25:
        cloud.header.stamp=now;pcpub.publish(cloud);last_cloud=time.monotonic()
    if not sent and time.monotonic()-started>4:
        msg=PoseStamped();msg.header=Header(seq=1,stamp=now,frame_id='camera_init')
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=goal;msg.pose.orientation.w=1
        gpub.publish(msg);sent=True
    if received:break
    time.sleep(.02)
result={'received_trajectory':bool(received),'events':events,'pose':data['pose'].tolist(),'goal':goal.tolist()}
if received:
    m=received[-1];result['knots']=list(m.knots);result['control_points']=[[p.x,p.y,p.z] for p in m.pos_pts]
    pts=np.array(result['control_points']);end=(pts[-3]+4*pts[-2]+pts[-1])/6
    result['endpoint']=end.tolist();result['endpoint_error']=float(np.linalg.norm(end-goal))
result['status']='PASS' if received and result['endpoint_error']<0.001 else 'FAIL'
(out/'planner_replay_result.json').write_text(json.dumps(result,indent=2))
print('Frozen planner reproduction',result['status'],flush=True)

sys.exit(0 if result['status']=='PASS' else 1)
