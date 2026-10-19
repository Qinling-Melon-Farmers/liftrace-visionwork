#!/usr/bin/env python3
"""SITL-only starter and observer. No target truth enters the mission."""
import csv,json,os,sys,time,threading
from pathlib import Path
import rospy,yaml,rospkg
from std_msgs.msg import String,Bool
from mavros_msgs.msg import State,ExtendedState
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger
sys.path.insert(0,str(Path(rospkg.RosPack().get_path('uav_board_trials'))/'scripts'))
from trial_result import evaluate

class Gate:
    def __init__(self):
        if not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR'):
            raise RuntimeError('SITL clock and sim_run directory required')
        if rospy.get_param('/board_trials/actuator_mode') not in ('mock','none'):
            raise RuntimeError('Real actuator forbidden in simulation')
        self.out=Path(rospy.get_param('~directory'));self.trial=rospy.get_param('~trial')
        self.truth_file=(self.out/'truth_pose.csv').open('w');self.truth_csv=csv.writer(self.truth_file);self.truth_csv.writerow(['t','x','y','z','vx','vy','vz']);self.truth_at=-1.;self.truth=None
        self.settings=yaml.safe_load((self.out/'settings.yaml').read_text())
        self.latest={};self.state=None;self.ext=None;self.pose=None;self.ready=False
        self.start_wall=time.monotonic();self.start_ros=None;self.last_try=0.;self.started=False;self.airborne=False;self.finish_at=None
        self.last_progress=0.;self.done=False;self.lock=threading.RLock();self.commands=[];self.stages=[];self.start_rejections=[]
        self.service=rospy.ServiceProxy('/navigation/start_mission',Trigger)
        self.subs=[rospy.Subscriber('/gazebo/model_states',ModelStates,self.on_truth,queue_size=1)]
        for key,topic in [('mission','/navigation/mission_status'),('high','/uav_high_view/probe_status'),('land_handoff','/board_trials/auto_land_status'),('contact','/mission/gazebo_contact_status')]:
            self.subs.append(rospy.Subscriber(topic,String,lambda msg,k=key:self.text(k,msg),queue_size=1))
        self.subs.extend([rospy.Subscriber('/mavros/state',State,lambda m:setattr(self,'state',m),queue_size=1),
          rospy.Subscriber('/mavros/extended_state',ExtendedState,lambda m:setattr(self,'ext',m),queue_size=1),
          rospy.Subscriber('/navigation/local_pose',PoseStamped,lambda m:setattr(self,'pose',m),queue_size=1),
          rospy.Subscriber('/mission/control_ready',Bool,lambda m:setattr(self,'ready',m.data),queue_size=1)])
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
        rospy.on_shutdown(self.shutdown)
    def on_truth(self,msg):
        if 'iris_mid360' not in msg.name:return
        now=rospy.Time.now().to_sec()
        if now-self.truth_at<.2:return
        self.truth_at=now;i=msg.name.index('iris_mid360');p=msg.pose[i].position;v=msg.twist[i].linear
        with self.lock:
            if self.truth_file.closed:return
            self.truth=[p.x,p.y,p.z];self.truth_csv.writerow([now,p.x,p.y,p.z,v.x,v.y,v.z]);self.truth_file.flush()
    def text(self,key,msg):
        try:value=json.loads(msg.data)
        except ValueError:return
        with self.lock:
            self.latest[key]=value
            if key=='mission':
                if value.get('active_command')=='LAND':self.latest['landing_command']=value
                sig=(value.get('active_decision_seq'),value.get('active_command'))
                if not self.commands or tuple(self.commands[-1]['key'])!=sig:
                    self.commands.append(dict(t=rospy.Time.now().to_sec(),key=sig,status=value))
            if key=='high':
                stage=value.get('stage')
                if not self.stages or self.stages[-1]['stage']!=stage:self.stages.append(dict(t=rospy.Time.now().to_sec(),stage=stage))
    def save(self,reason):
        with self.lock:
            supervisor=dict(trial=self.trial,end_reason=reason,actuator_mode='none' if self.settings['mode'] in ('landing','memory_only') else 'mock')
            result=evaluate(supervisor,self.latest,self.settings)
            checks={}
            commands=[r['key'][1] for r in self.commands];stages=[r['stage'] for r in self.stages]
            if self.settings['mode']=='memory_only':checks['no_approach']= 'APPROACH' not in commands
            if self.settings['mode'] in ('high_view','high_priority','memory_only','high_view_full'):
                checks['survey_and_descent_observed']='SURVEY' in stages and 'DESCEND' in stages
                if self.trial=='high_priority':checks['early_interrupt_observed']=any('SURVEY_INTERRUPTED_TOP3' in str(e) for e in self.latest.get('high',{}).get('events',[]))
            checks['no_collision']=self.latest.get('contact',{}).get('actual_collision_count',-1)==0
            checks['contact_sensor_ready']=self.latest.get('contact',{}).get('ready') is True
            checks['video_recorded']=(self.out/'camera_frames.csv').exists() and (self.out/'overview.mp4').exists()
            result.update(scope='board_runtime_SITL',checks=checks,commands=self.commands,stages=self.stages,
              elapsed_sim_s=(rospy.Time.now().to_sec()-self.start_ros if self.start_ros else None),wall_s=time.monotonic()-self.start_wall,start_rejections=self.start_rejections,
              contact=self.latest.get('contact',{}),final_truth_xyz=self.truth,source_run=os.environ.get('SIM_RUN_DIR'))
            if result['status']=='PASS' and not all(checks.values()):result['status']='INCOMPLETE'
            (self.out/'gate_status.json').write_text(json.dumps(result,indent=2))
            (Path(os.environ['SIM_RUN_DIR'])/'gate_status.json').write_text(json.dumps(result,indent=2))
            (self.out/'supervisor_result.json').write_text(json.dumps(supervisor,indent=2))
            return result
    def shutdown(self):
        if not self.done:self.save('launch_interrupted')
        with self.lock:self.truth_file.close()
    def run(self):
        while not rospy.is_shutdown():
            time.sleep(.2);now=rospy.Time.now().to_sec();wall=time.monotonic()
            if not self.started and self.ready and self.state and self.state.armed and self.state.mode=='OFFBOARD' and wall-self.last_try>2.:
                self.last_try=wall
                try:
                    rospy.wait_for_service('/navigation/start_mission',timeout=.1);res=self.service()
                    if res.success:self.started=True;self.start_ros=now;rospy.logwarn('BOARD SIM mission started: %s',self.trial)
                    else:self.start_rejections.append(dict(t=now,reason=res.message))
                except rospy.ROSException:pass
            if self.pose and self.pose.pose.position.z>.3:self.airborne=True
            mission=self.latest.get('mission',{});reason=None
            if wall-self.last_progress>2.:
                self.last_progress=wall
                with self.lock:
                    payload=dict(started=self.started,t=now,mission=mission,high=self.latest.get('high',{}),
                        handoff=self.latest.get('land_handoff',{}),contact=self.latest.get('contact',{}),
                        pose=([self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z] if self.pose else None))
                    (self.out/'progress.json').write_text(json.dumps(payload,indent=2))
            landed=bool(self.started and self.airborne and self.ext and self.ext.landed_state==1 and self.pose and self.pose.pose.position.z<.15)
            if landed:reason='landed_after_flight'
            elif mission.get('mission_failed') or mission.get('phase')=='ABORTED':reason='mission_failed'
            elif self.latest.get('contact',{}).get('actual_collision_count',0)>0:reason='obstacle_contact'
            elif not self.started and wall-self.start_wall>240:reason='startup_timeout'
            elif self.start_ros and now-self.start_ros>630:reason='mission_timeout'
            elif wall-self.start_wall>2400:reason='host_watchdog'
            if reason:
                if self.finish_at is None:self.finish_at=now
                if now-self.finish_at>=3.:
                    result=self.save(reason);self.done=True
                    rospy.logwarn('BOARD SIM %s: %s (%s)',self.trial,result['status'],reason)
                    rospy.signal_shutdown(reason);return
            else:self.finish_at=None

if __name__=='__main__':rospy.init_node('trial_sim_gate');Gate();rospy.spin()
