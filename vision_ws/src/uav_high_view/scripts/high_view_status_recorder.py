#!/usr/bin/env python3
"""Small simulation research-status log; no control interfaces."""
import json
import os
from pathlib import Path
import rospy
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node('high_view_status_recorder')
    if not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR'):raise RuntimeError('simulation recording only')
    root=Path(os.environ['SIM_RUN_DIR']);last=[None]
    def callback(msg):
        if msg.data==last[0]:return
        data=json.loads(msg.data)
        with (root/'high_view_full_events.jsonl').open('a') as out:out.write(json.dumps(dict(t=rospy.Time.now().to_sec(),status=data))+'\n')
        last[0]=msg.data
    sub=rospy.Subscriber(rospy.get_param('~topic','/uav_high_view/probe_status'),String,callback,queue_size=1)
    diagnostic_subs=[]
    if rospy.get_param('~diagnostic_cloud',False):
        import numpy as np
        from sensor_msgs.msg import PointCloud2
        goal=np.asarray(rospy.get_param('~diagnostic_goal_xyz'),dtype=float)
        radius=float(rospy.get_param('~diagnostic_radius_m',.8))
        interval=float(rospy.get_param('~diagnostic_interval_s',2.))
        if goal.shape!=(3,) or not np.isfinite(goal).all() or not 0<radius<=2. or interval<1.:
            raise ValueError('invalid diagnostic region')
        cloud_last={}
        def cloud_callback(msg,kind):
            now=rospy.Time.now().to_sec()
            if now-cloud_last.get(kind,-1e9)<interval:return
            cloud_last[kind]=now
            fields={f.name:f for f in msg.fields}
            if any(k not in fields or fields[k].datatype!=7 for k in ('x','y','z')):return
            dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                                offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
            a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
            xyz=np.column_stack([a[k].ravel() for k in ('x','y','z')])
            roi=xyz[np.linalg.norm(xyz[:,:2]-goal[:2],axis=1)<radius]
            row=dict(t=now,source=kind,frame=msg.header.frame_id,stamp=msg.header.stamp.to_sec(),
                     points=len(xyz),roi_points=len(roi),goal=goal.tolist(),radius_m=radius)
            if len(roi):
                dist=np.linalg.norm(roi-goal,axis=1)
                row.update(z_min=float(roi[:,2].min()),z_max=float(roi[:,2].max()),
                           nearest_xyz=roi[int(np.argmin(dist))].tolist(),nearest_distance_m=float(dist.min()),
                           near_goal_height=int(np.count_nonzero(abs(roi[:,2]-goal[2])<.25)))
            with (root/'high_goal_cloud_diagnostic.jsonl').open('a') as out:out.write(json.dumps(row)+'\n')
        for kind,param,default in [('source','~diagnostic_source_topic','/freedom/static_pointcloud'),
                                   ('inflated','~diagnostic_inflated_topic','/sdf_map/occupancy_inflate')]:
            diagnostic_subs.append(rospy.Subscriber(rospy.get_param(param,default),PointCloud2,cloud_callback,kind,queue_size=1))
    rospy.spin()
