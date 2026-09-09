#!/usr/bin/env python3
"""Move only the collision-free recording camera after delivery; no aircraft commands."""
import argparse,json
from pathlib import Path
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--x',type=float,default=0);p.add_argument('--y',type=float,default=8.35);p.add_argument('--z',type=float,default=5);a=p.parse_args()
progress=json.loads((a.run_dir/'live_progress.json').read_text())
if progress['phase'] not in ['POST_DELIVERY_ROUTE','LAND']:
    raise SystemExit('Keep the search view until the post-delivery route begins')
rospy.init_node('overview_view_change',anonymous=True)
service=rospy.get_param('~model_state_service','/gazebo/set_model_state')
rospy.wait_for_service(service,timeout=10)
msg=ModelState();msg.model_name='competition_overview_camera';msg.reference_frame='world'
msg.pose.position.x=a.x;msg.pose.position.y=a.y;msg.pose.position.z=a.z
msg.pose.orientation.x=.5;msg.pose.orientation.y=.5;msg.pose.orientation.z=-.5;msg.pose.orientation.w=.5
reply=rospy.ServiceProxy(service,SetModelState)(msg)
if not reply.success:raise RuntimeError(reply.status_message)
(a.run_dir/'overview_view_change.json').write_text(json.dumps({'ros_sec':rospy.get_time(),'model':msg.model_name,'position':[a.x,a.y,a.z],'purpose':'Recording camera view only'},indent=2))
