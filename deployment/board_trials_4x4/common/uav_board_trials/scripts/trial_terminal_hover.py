#!/usr/bin/env python3
"""Site-only single setpoint outlet: terminal vertical descent and manual landing."""
import copy,json,math,threading,sys
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from std_msgs.msg import String
# Catkin relays execute source in a private namespace and cannot be imported
# as helper modules. Resolve the sibling source before the devel/bin relay.
sys.path.insert(0,str(Path(__file__).resolve().parent))
from trial_auto_land import trial_ready,settled

def descend(z,target,speed,dt):
    return max(target,z-speed*min(max(dt,0.),.15))

class TerminalHover:
    def __init__(self):
        self.frame=rospy.get_param('~frame')
        self.ground=float(rospy.get_param('~ground_z'))
        self.target=self.ground+float(rospy.get_param('~hover_agl'))
        self.cap=self.ground+float(rospy.get_param('~max_agl'))
        self.speed=float(rospy.get_param('~descent_speed',.15))
        if not all(math.isfinite(v) for v in (self.target,self.cap,self.speed)) or not 0<self.speed<=.2 or self.target>=self.cap:raise ValueError('invalid hover parameters')
        self.lock=threading.RLock()
        self.raw=None;self.odom=None;self.state=None;self.status={};self.context={}
        self.raw_at=self.state_at=self.status_at=0.
        self.active=False;self.cancelled=False;self.since=None;self.setpoint=None;self.previous=None
        self.pub=rospy.Publisher('/navigation/setpoint_mission',PoseStamped,queue_size=1)
        self.status_pub=rospy.Publisher('/board_trials/terminal_hover_status',String,queue_size=1,latch=True)
        self.subs=[rospy.Subscriber('/board_trials/controller_setpoint',PoseStamped,self.on_raw,queue_size=1),
            rospy.Subscriber('/navigation/local_odom',Odometry,self.on_odom,queue_size=1),
            rospy.Subscriber('/mavros/state',State,self.on_state,queue_size=1),
            rospy.Subscriber('/navigation/mission_status',String,self.on_status,queue_size=1),
            rospy.Subscriber('/board_trials/landing_context',String,self.on_context,queue_size=1)]
        self.timer=rospy.Timer(rospy.Duration(.05),self.tick)
    def on_raw(self,m):
        with self.lock:self.raw=m;self.raw_at=rospy.Time.now().to_sec()
    def on_odom(self,m):
        with self.lock:self.odom=m
    def on_state(self,m):
        with self.lock:self.state=m;self.state_at=rospy.Time.now().to_sec()
    def on_status(self,m):
        with self.lock:
            try:self.status=json.loads(m.data);self.status_at=rospy.Time.now().to_sec()
            except ValueError:self.status={}
    def on_context(self,m):
        with self.lock:
            try:
                v=json.loads(m.data)
                if not -.05<=rospy.Time.now().to_sec()-v['time']<=.5:return
                if len(v['xy'])!=2 or not all(math.isfinite(x) for x in v['xy']+[v['z']]):return
                self.context=v
            except (ValueError,KeyError,TypeError):self.context={}
    def tick(self,_):
        with self.lock:
            now=rospy.Time.now().to_sec();s=self.state;o=self.odom
            fresh=(o is not None and o.header.frame_id==self.frame and -.05<=now-o.header.stamp.to_sec()<=.3)
            if self.active and (s is None or not s.armed or s.mode!='OFFBOARD'):
                self.active=False;self.cancelled=True
                self.status_pub.publish(String(data=json.dumps(dict(stage='PILOT_HANDOFF',time=now))))
            if not self.active and not self.cancelled:
                ready=(s is not None and s.connected and s.armed and s.mode=='OFFBOARD'
                       and 0<=now-self.state_at<=2. and fresh
                       and 0<=now-self.status_at<=15.
                       and trial_ready(self.status,self.context,self.frame)
                       and settled(o,self.frame,self.context['xy'],self.context['z'],.18,.15,.12))
                if ready:
                    self.since=self.since if self.since is not None else now
                    if now-self.since>=1.:
                        self.active=True;self.setpoint=PoseStamped()
                        self.setpoint.header.frame_id=self.frame
                        self.setpoint.pose=copy.deepcopy(o.pose.pose)
                        self.previous=now
                        self.status_pub.publish(String(data=json.dumps(dict(stage='DESCEND_TO_HOVER',target_z=self.target,time=now))))
                else:self.since=None
            if self.active:
                # Stale odometry stops descent; retain last setpoint rather than extrapolating.
                dt=now-self.previous;self.previous=now
                if fresh and s.connected and 0<=now-self.state_at<=2.:
                    self.setpoint.pose.position.z=descend(self.setpoint.pose.position.z,self.target,self.speed,dt)
                output=copy.deepcopy(self.setpoint)
            elif self.cancelled and self.setpoint is not None:
                output=copy.deepcopy(self.setpoint)
            elif self.raw is not None and 0<=now-self.raw_at<=.5:
                output=copy.deepcopy(self.raw)
                if output.header.frame_id not in ('',self.frame):return
                output.header.frame_id=self.frame
            else:return
            output.pose.position.z=min(output.pose.position.z,self.cap)
            output.header.stamp=rospy.Time.now()
            self.pub.publish(output)

if __name__=='__main__':
    rospy.init_node('trial_terminal_hover');TerminalHover();rospy.spin()