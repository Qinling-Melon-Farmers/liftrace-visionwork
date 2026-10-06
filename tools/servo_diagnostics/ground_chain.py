#!/usr/bin/env python3
"""Isolated ground evidence fixture -> production arbiter -> action proxy -> real PWM."""
from pathlib import Path
import argparse,os,sys,time,json,signal,subprocess,threading,fcntl,xmlrpc.client,traceback
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Int8
from uav_vision.msg import ReleaseEvidenceContext,AlignmentTargetContext
from uav_mission.msg import ReleasePermission,ReleaseResult
from patrol_control.srv import ServoAction,ServoActionRequest
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root',type=Path,required=True)
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--raw-node',type=Path,required=True)
parser.add_argument('--namespace',default='/ground_servo_check')
parser.add_argument('--port',type=int,default=11329)
parser.add_argument('--real-output',action='store_true',help='Initialize and pulse the three real servos; obtain operator authorization first')
parser.add_argument('--left-pwm-device',choices=('fd8b0000.pwm','fd8b0010.pwm'),default='fd8b0000.pwm',help='Use PWM0/D2 candidate or explicitly select PWM1/D3 formal wiring')
args=parser.parse_args()
if not args.real_output:parser.error('This diagnostic requires explicitly selected --real-output')
ROOT=args.root.resolve();OUT=args.out.resolve();OUT.mkdir(parents=True,exist_ok=True)
NS=args.namespace.rstrip('/')
if not NS.startswith('/') or NS=='':parser.error('namespace must be absolute')
os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(args.port)
os.environ['ROS_IP']='127.0.0.1'
os.environ.pop('ROS_HOSTNAME',None)
failed=False
devices=('febf0020.pwm','febf0030.pwm',args.left_pwm_device)
left_pin={'fd8b0000.pwm':'GPIO1_D2','fd8b0010.pwm':'GPIO1_D3'}[args.left_pwm_device]
def channels():
 result={}
 for slot,device in enumerate(devices,1):
  matches=[p for p in Path('/sys/class/pwm').glob('pwmchip*') if '/'+device+'/' in str(p.resolve())]
  if len(matches)!=1:raise RuntimeError('Expected one PWM controller for '+device)
  result[slot]=matches[0]/'pwm0'
 return result
children=[];handles=[];rows=[];results=[];latest=None;active=0;stop=threading.Event()
first={};mission='ground-only-'+OUT.name
lock=open('/tmp/liftrace_ground_servo.lock','w')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
def emit(kind,**v):
 row=dict(event=kind,wall=time.time(),**v);rows.append(row)
 print(json.dumps(row),flush=True)
 (OUT/'progress.json').write_text(json.dumps(rows,indent=2))
def spawn(name,args):
 f=open(OUT/(name+'.log'),'w');handles.append(f)
 p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
def alarm(*_):raise TimeoutError('ground test wall timeout')
signal.signal(signal.SIGALRM,alarm);signal.alarm(90)
emit('PWM_mapping',channels={str(slot):str(p) for slot,p in channels().items()},left_pin=left_pin,raw_node=str(args.raw_node.resolve()))
def permission(msg):
 global latest
 latest=msg
def result(msg):
 results.append(dict(slot=msg.payload_slot,success=msg.success,reason=msg.reason,state=msg.execution_state,terminal=msg.terminal,execution_id=msg.execution_id,mission=msg.mission_id,decision=msg.decision_seq))
def waitfor(test,seconds,label):
 start=time.monotonic()
 while time.monotonic()-start<seconds:
  if test():return
  if any(p.poll() is not None for p in children):raise RuntimeError('child exited during '+label)
  time.sleep(.03)
 raise TimeoutError(label+' latest='+str(None if latest is None else (latest.permitted,latest.payload_slot,latest.reason)))
def request(p,rid):
 q=ServoActionRequest();q.request_id=rid
 for k in ['payload_slot','mission_id','decision_seq','attempt','target_id','target_first_seen','target_class','align_mode','permission_epoch','permission_revision']:setattr(q,k,getattr(p,k))
 return q
try:
 # Fail if a flight/control/hardware process appeared since precheck.
 for d in Path('/proc').iterdir():
  if not d.name.isdigit() or int(d.name)==os.getpid():continue
  try:a=(d/'cmdline').read_bytes().split(bytes([0]))
  except OSError:continue
  names=[Path(x.decode(errors='replace')).name for x in a if x and not x.startswith(b'-')]
  if any(n in ['mavros_node','patrol_control','pwm_node1','fast_planner_node','navigation_mission_manager.py','navigation_high_view_probe.py'] for n in names):
   raise RuntimeError('another flight/actuator process exists '+d.name)
 spawn('master',['roscore','-p',str(args.port)])
 def master_ready():
  try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/ground_precheck')[0]==1
  except Exception:return False
 waitfor(master_ready,12,'ROS master')
 rospy.init_node('ground_servo_fixture',disable_signals=True)
 scripts=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts'
 arb=dict(require_evidence_context=True,class_profile='r2026',min_release_altitude=-.05,max_release_altitude=.30,
  align_mode_topic=NS+'/mode',pose_topic=NS+'/pose',control_state_topic=NS+'/control_state',
  evidence_context_topic=NS+'/evidence_context',evidence_topic=NS+'/evidence',commitment_evidence_topic=NS+'/commitment',
  permission_topic=NS+'/permission',permission_state_topic=NS+'/permission_active',authorization_topic=NS+'/authorization',
  result_topic=NS+'/result',alignment_context_topic=NS+'/alignment')
 proxy=dict(service_name=NS+'/Servo',action_service_name=NS+'/servo_action',raw_service_name='/legacy/Servo_raw',
  permission_topic=NS+'/permission',result_topic=NS+'/result',alignment_context_topic=NS+'/alignment')
 for node,params in [('ground_arbiter',arb),('ground_proxy',proxy)]:
  for key,value in params.items():rospy.set_param('/'+node+'/'+key,value)
 spawn('arbiter',['python3',str(scripts/'release_permission_arbiter.py'),'__name:=ground_arbiter'])
 spawn('proxy',['python3',str(scripts/'guarded_servo_proxy.py'),'__name:=ground_proxy'])
 rospy.Subscriber(NS+'/permission',ReleasePermission,permission,queue_size=10)
 rospy.Subscriber(NS+'/result',ReleaseResult,result,queue_size=30)
 rospy.wait_for_service(NS+'/servo_action',timeout=10)
 call=rospy.ServiceProxy(NS+'/servo_action',ServoAction)
 # Invalid action cannot reach raw; raw driver is not running yet.
 bad=ServoActionRequest();bad.request_id=1;bad.payload_slot=1
 denied=call(bad)
 assert not denied.res and denied.execution_state==1
 emit('no_valid_authorization_rejected',reason=denied.reason)
 emit('initializing_three_slots')
 spawn('raw',[str(args.raw_node.resolve()),'__name:=ground_raw_servo','Servo:=/legacy/Servo_raw'])
 rospy.wait_for_service('/legacy/Servo_raw',timeout=15)
 emit('raw_ready')
 pubs={k:rospy.Publisher(NS+'/'+k,t,queue_size=3) for k,t in [('pose',PoseStamped),('control_state',Int8),('mode',String),('evidence_context',ReleaseEvidenceContext)]}
 def feed():
  while not stop.is_set() and not rospy.is_shutdown():
   slot=active
   now=rospy.Time.now()
   p=PoseStamped();p.header.stamp=now;p.header.frame_id='ground_test_only';p.pose.orientation.w=1;p.pose.position.z=.1
   pubs['pose'].publish(p);pubs['control_state'].publish(Int8(data=2));pubs['mode'].publish(String(data='drop_circle' if slot else 'disabled'))
   if slot:
    c=ReleaseEvidenceContext();c.header.stamp=now;c.context_header.stamp=now;c.context_header.frame_id='ground_test_only'
    c.context_valid=True;c.context_reason='synthetic_ground_only';c.context_source='isolated_ground_fixture';c.context_schema_version=1;c.context_active=True
    c.mission_id=mission;c.decision_seq=slot;c.deadline=first[slot]+rospy.Duration(25);c.command=2;c.class_profile='r2026';c.align_mode='drop_circle'
    c.has_semantic_target=True;c.semantic_target_id=100+slot;c.semantic_target_first_seen=first[slot];c.target_observation_stamp=now
    c.semantic_target_class=['panzer','bridge','pillbox'][slot-1];c.attempt=1;c.payload_slot=slot;c.semantic_target_pose=p
    c.max_association_distance_m=.3;c.geometry_target_present=True;c.geometry_target_id=200+slot;c.geometry_target_first_seen=first[slot];c.geometry_target_last_seen=now
    c.geometry_target_class='circle';c.geometry_map_valid=True;c.geometry_target_pose=p;c.association_distance_m=0;c.semantic_geometry_match=True
    e=c.evidence;e.header.stamp=now;e.header.frame_id='ground_test_only';e.align_mode='drop_circle';e.target_present=True;e.target_id=200+slot;e.target_class='circle'
    e.target_confirmed=True;e.geometry_verified=True;e.center_refined=True;e.observation_fresh=True;e.observation_age_sec=0;e.aligned=True;e.stable_frames=20;e.evidence_valid=True
    pubs['evidence_context'].publish(c)
   stop.wait(.04)
 threading.Thread(target=feed,daemon=True).start()
 time.sleep(1)
 for slot in [1,2,3]:
  first[slot]=rospy.Time.now();active=slot
  waitfor(lambda:latest is not None and latest.permitted and latest.payload_slot==slot and latest.decision_seq==slot,7,'permit slot '+str(slot))
  p=latest;q=request(p,100+slot)
  wrong=request(p,200+slot);wrong.payload_slot=slot%3+1
  wrong_ack=call(wrong)
  expected_state=ReleaseResult.EXECUTION_UNKNOWN if slot==3 else ReleaseResult.NOT_STARTED
  expected_reason='payload_slot_locked_uncertain' if slot==3 else 'request_action_identity_mismatch'
  assert not wrong_ack.res and wrong_ack.execution_state==expected_state and wrong_ack.reason==expected_reason, repr(wrong_ack)
  emit('wrong_slot_rejected',requested=wrong.payload_slot,authorized=slot,state=wrong_ack.execution_state,reason=wrong_ack.reason)
  p=latest;q=request(p,100+slot)
  emit('release_start',slot=slot,permission_revision=p.permission_revision)
  start=time.monotonic();ack=call(q);elapsed=time.monotonic()-start
  emit('release_ack',slot=slot,success=ack.res,state=ack.execution_state,reason=ack.reason,elapsed_s=elapsed)
  assert ack.res and ack.execution_state==3 and ack.terminal
  waitfor(lambda:any(x['slot']==slot and x['success'] and x['state']==3 for x in results),3,'completion event')
  # Same action/slot replay must not enter raw again.
  replay=call(q)
  assert not replay.res
  emit('duplicate_rejected',slot=slot,reason=replay.reason)
  active=0
  time.sleep(8)
 waitfor(lambda:latest is not None and latest.reason=='payload_exhausted',3,'all slots committed')
 emit('three_slots_committed',next_slot=latest.payload_slot,reason=latest.reason)
 assert len([x for x in results if x['state']==2 and not x['terminal']])==3
 assert len([x for x in results if x['state']==3 and x['success']])==3
 emit('PASS',scope='synthetic strict evidence -> production arbiter -> fenced action proxy -> physical PWM; no flight controller')
except BaseException as e:
 failed=True
 emit('FAIL',error=repr(e),traceback=traceback.format_exc())
finally:
 stop.set()
 try:rospy.signal_shutdown('ground check finished')
 except Exception:pass
 for p in reversed(children):
  if p.poll() is None:
   try:os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
   except ProcessLookupError:pass
 states={}
 for slot,base in channels().items():
  try:
   if (base/'enable').exists() and (base/'enable').read_text().strip()!='0':(base/'enable').write_text('0')
   states[str(slot)]=dict(path=str(base),**{n:(base/n).read_text().strip() if (base/n).exists() else None for n in ['enable','duty_cycle','period']})
  except Exception as e:states[str(slot)]={'error':repr(e)}
 if any(value.get('enable')!='0' for value in states.values()):failed=True
 (OUT/'results.json').write_text(json.dumps(dict(events=rows,release_results=results,pwm_final=states,children=[dict(pid=p.pid,code=p.poll()) for p in children]),indent=2))
 emit('cleaned',pwm=states)
 signal.alarm(0)

raise SystemExit(1 if failed else 0)
