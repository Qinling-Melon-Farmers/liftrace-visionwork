import ast,math,unittest
from pathlib import Path
from types import SimpleNamespace as N, MethodType
from uav_mission.motion_observations import odom_world_velocity,FreshPoseWindow
from uav_mission.motion_optimization import MotionOptimization,MovingRecoveryWindow
from uav_mission.planner_execution import OdomSample
from uav_mission.execution_speed import FollowingSpeed
from uav_mission.corridor_speed import CorridorSpeed,CorridorSpeedConfig
P=Path(__file__).resolve().parents[1]
def methods(file,names,ns):
 tree=ast.parse((P/'scripts'/file).read_text())
 for name in names:
  f=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
  f.decorator_list=[];exec(compile(ast.Module(body=[f],type_ignores=[]),str(file),'exec'),ns)
 return ns
class Clock:
 value=10.
 @classmethod
 def now(cls):return N(to_sec=lambda:cls.value)
class PolicyTests(unittest.TestCase):
 def test_actual_parser_rotates_full_attitude_and_preserves_header_twist(self):
  ns=methods('navigation_planner_bridge.py',['_odom_from_message'],dict(math=math,OdomSample=OdomSample,odom_world_velocity=odom_world_velocity,_stamp_to_ns=lambda t:t))
  for deg in (0,5,10):
   a=math.radians(deg);q=N(x=0.,y=math.sin(a/2),z=0.,w=math.cos(a/2))
   m=N(header=N(stamp=1_000_000_000,frame_id='map'),child_frame_id='base_link',pose=N(pose=N(position=N(x=0,y=0,z=1),orientation=q)),twist=N(twist=N(linear=N(x=-.5*math.sin(a),y=0,z=.5*math.cos(a)))))
   v=ns['_odom_from_message'](m,'child');self.assertAlmostEqual(v.vx,0);self.assertAlmostEqual(v.vz,.5)
   m.twist.twist.linear=N(x=0,y=0,z=.5)
   v=ns['_odom_from_message'](m,'header');self.assertEqual(v.vx,0);self.assertEqual(v.vz,.5)
   m.child_frame_id=''
   with self.assertRaises(ValueError):ns['_odom_from_message'](m,'child')
   m.pose.pose.orientation.w=0.
   with self.assertRaises(ValueError):ns['_odom_from_message'](m,'header')
 def sample(self,t,**v):return OdomSample(int(t*1e9),'map',0,0,1,v.get('vx',0),0,v.get('vz',.4))
 def test_early_handoff_counts_unique_fresh_samples_only(self):
  w=MovingRecoveryWindow();call=lambda t,now=None,**v:w.update(self.sample(t,**v),1_000_000_000,int((t if now is None else now)*1e9),.6,.6)
  self.assertFalse(call(1.6));self.assertFalse(call(1.78));self.assertFalse(call(1.78,1.79))
  self.assertTrue(call(1.79));self.assertFalse(call(1.8,2.01))
  self.assertFalse(call(2.1));self.assertFalse(call(2.2,vx=.3));self.assertFalse(call(2.3,vz=-.1))
  self.assertFalse(call(2.4,vz=0));self.assertFalse(call(2.48,vz=0));self.assertTrue(call(2.56,vz=0))
 def manager(self):
  params={'/traj_server/traj_server/target_dist':1.,'/px4_max_distance':1.}
  rospy=N(get_param=lambda k,d=None:params.get(k,d),set_param=lambda k,v:params.__setitem__(k,v),Time=Clock,loginfo=lambda *a:None)
  names=['_motion_pose_ready','_apply_stale_speed_override','_height_stage_ready','_apply_following_speed','_initialize_line_preference']
  ns=methods('navigation_mission_manager.py',names,dict(rospy=rospy,math=math,FreshPoseWindow=FreshPoseWindow,MotionOptimization=MotionOptimization,FollowingSpeed=FollowingSpeed,CorridorSpeed=CorridorSpeed,CorridorSpeedConfig=CorridorSpeedConfig))
  o=N(_pose_max_age=.5,_following_speed_events=[],_runtime=N(stage='SURVEY',core=N(mission_id='test',post_delivery_route_index=1,config=N(landing_xy=(0,0)))))
  for n in names:setattr(o,n,MethodType(ns[n],o))
  return o,params
 def pose(self,t,z=.9):return N(header=N(stamp=N(to_sec=lambda:t),frame_id='map'),pose=N(position=N(x=0,y=0,z=z)))
 def test_stale_override_has_priority_and_requires_three_new_samples(self):
  for initial,expected in ((1.,.2),(.15,.15)):
   o,p=self.manager();p['/traj_server/traj_server/target_dist']=p['/px4_max_distance']=initial
   o._pose=self.pose(9.)
   self.assertFalse(o._motion_pose_ready(10.));self.assertEqual(p['/px4_max_distance'],expected)
   o._apply_following_speed(N(command='RETURN_HOME'),True);self.assertEqual(p['/px4_max_distance'],expected)
   for stamp,now in ((10.,10.),(10.,10.1),(10.1,10.1)):
    o._pose=self.pose(stamp);self.assertFalse(o._motion_pose_ready(now))
   o._pose=self.pose(10.2);self.assertTrue(o._motion_pose_ready(10.2))
 def test_height_transition_waits_below_limit_then_requires_fresh_planner_ack(self):
  o,p=self.manager();ns='/navigation_height_constraint';cap='/external_planner_max_command_z'
  p.update({ns+'/enabled':True,ns+'/frame_id':'map',cap:2.78,'~mission/post_delivery_parameter_stages':[dict(after_completed_waypoints=1,parameters={cap:.78})]})
  a=N(command='RETURN_HOME',reason='post_delivery_route:2',decision_seq=4)
  # ground=-.22: FC AGL1.02 local Z=.80 must not advance to the 1.00m ceiling.
  o._pose=self.pose(10.,.80);self.assertFalse(o._height_stage_ready(a));self.assertEqual(p[cap],2.78)
  o._pose=self.pose(10.,.77);self.assertFalse(o._height_stage_ready(a));self.assertEqual(p[cap],.78)
  p.update({ns+'/planner_applied_max_z':.78,ns+'/planner_applied_frame':'map',ns+'/planner_applied_stamp':9.9})
  self.assertFalse(o._height_stage_ready(a));p[ns+'/planner_applied_stamp']=10.;self.assertTrue(o._height_stage_ready(a))
 def test_global_line_weight_independent_of_motion_and_stage(self):
  for weight in (0.,1.,2.):
   for enabled in (False,True):
    o,p=self.manager();p['~planner_line_preference/weight']=weight;p['~motion_optimization']={'enabled':enabled}
    o._initialize_line_preference()
    for stage,command in [('SURVEY','SEARCH'),('REVISIT','APPROACH'),('RETURN_HOME','RETURN_HOME'),('CORRIDOR','RETURN_HOME')]:
     o._runtime.stage=stage;o._apply_following_speed(N(command=command),True)
     self.assertEqual(p['/fast_planner_node/search/line_deviation_weight'],weight)
if __name__=='__main__':unittest.main()
