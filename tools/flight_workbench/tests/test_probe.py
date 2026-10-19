#!/usr/bin/env python3
"""板端探针离线测试：用假 rospy/mavros_msgs/std_msgs 跑 board_probe.py。

覆盖：无 master 时输出结构化错误；有 master 时 State/ExtendedState/String 取值正确
（回归点：mavros_msgs/State 没有 .data 字段，早期实现会在板端刷异常日志）、
服务类型读取、节点列表、以及"只订阅不下发"的边界（假 rospy 里没有任何 Publisher/ServiceProxy）。

    cd tools/flight_workbench && python3 tests/test_probe.py
"""
import json
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_DIR = os.path.dirname(HERE)
RESULTS = []

ROSPY_UP = '''
ERROR = 40
WARN = 30
INFO = 20
Subscribers = []

class Time:
    @staticmethod
    def now():
        return Time()
    def to_sec(self):
        return 100.0

class AnyMsg(object):
    pass

def init_node(name, **kwargs):
    if not _up:
        raise RuntimeError("Unable to communicate with master!")
    globals()["_node"] = name

def is_shutdown():
    return False

def Subscriber(topic, msg_type, callback, **kwargs):
    assert kwargs.get('queue_size') == 1
    if topic.endswith('/fallback') and msg_type is not AnyMsg:
        raise TypeError('typed subscriber unavailable')
    Subscribers.append((topic, msg_type, callback))
    # 订阅瞬间投递一条合成消息，便于离线校验取值路径
    try:
        callback(_sample(topic, msg_type))
    except Exception as error:
        print("SAMPLE_ERROR %s %s" % (topic, error))

def _sample(topic, msg_type):
    name = getattr(msg_type, "__name__", "")
    if name == "State":
        return msg_type()
    if name == "ExtendedState":
        return msg_type()
    if name == "String":
        if topic.endswith('/low_hover'):
            return msg_type('{"stage":"HOLD_FOR_PILOT","profile":"baseline","recorder":true}')
        if topic.endswith('/bad_json'):
            return msg_type('invalid JSON')
        if topic.endswith('/nan_json'):
            return msg_type('{"stage":"HOLD_FOR_PILOT","value":NaN}')
        payload = '{"phase": "HIGH_VIEW_SEARCH", "reason": "ok"}'
        return msg_type(payload)
    if topic.endswith('/typed_error'):
        return AnyMsg()
    if name in ('PoseStamped','Odometry'):
        value=msg_type()
        pose=value.pose.pose if name=='Odometry' else value.pose
        if topic.endswith('/bad_pose'):pose.orientation.w=float('nan')
        if topic.endswith('/zero_pose'):pose.orientation.w=0.
        if topic.endswith('/bad_position'):pose.position.x=float('inf')
        return value
    return msg_type()
'''

FAKE_MSGS = {
    "geometry_msgs/__init__.py": "",
    "geometry_msgs/msg/__init__.py": "from .msg import PoseStamped\n",
    "geometry_msgs/msg/msg.py": textwrap.dedent('''
        from types import SimpleNamespace
        class Stamp:
            def to_sec(self):return 96.0
        class PoseStamped:
            def __init__(self):
                self.header=SimpleNamespace(frame_id='camera_init',stamp=Stamp())
                self.pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.,z=.3),
                    orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=2.))
    '''),
    "nav_msgs/__init__.py": "",
    "nav_msgs/msg/__init__.py": textwrap.dedent('''
        from types import SimpleNamespace
        from geometry_msgs.msg import PoseStamped
        class Odometry:
            def __init__(self):
                sample=PoseStamped()
                self.header=sample.header
                self.pose=SimpleNamespace(pose=sample.pose)
    '''),
    "sensor_msgs/__init__.py": "",
    "sensor_msgs/msg/__init__.py": textwrap.dedent('''
        class BatteryState:
            voltage=14.8
            current=float('nan')
            percentage=float('inf')
    '''),
    "diagnostic_msgs/__init__.py": "",
    "diagnostic_msgs/msg/__init__.py": textwrap.dedent('''
        class KV:
            def __init__(self, key, value):
                self.key, self.value = key, value
        class Status:
            name, level, message = 'FAST_LIO', 1, 'queue waiting'
            values = [KV('output_age_sec', '0.23'), KV('lidar_queue', '2')]
        class DiagnosticArray:
            status = [Status()]
    '''),
    "mavros_msgs/__init__.py": "",
    "mavros_msgs/msg/__init__.py": "",
    "mavros_msgs/msg/_mod.py": textwrap.dedent('''
        class State(object):
            def __init__(self):
                self.connected = True
                self.armed = False
                self.mode = "OFFBOARD"
                self.guided = True
                self.system_status = 3

        class ExtendedState(object):
            def __init__(self):
                self.landed_state = 1
                self.vtol_state = 0
        class RCOut:
            channels=[1100,1200,1300,1400]
        class ActuatorControl:
            group_mix=0
            controls=[.1,.2,.3,.4,float('nan'),0.,0.,0.]
        class ESCStatusItem:
            rpm=1234
            voltage=14.7
            current=2.3
        class ESCStatus:
            esc_status=[ESCStatusItem()]
        class ESCTelemetryItem:
            rpm=2345
            voltage=14.6
            current=float('inf')
            temperature=39.0
            totalcurrent=1.0
            count=2
        class ESCTelemetry:
            esc_telemetry=[ESCTelemetryItem()]
    '''),
    "std_msgs/__init__.py": "",
    "std_msgs/msg/__init__.py": "",
    "std_msgs/msg/_mod.py": textwrap.dedent('''
        class String(object):
            def __init__(self, data=""):
                self.data = data
    '''),
    "rosnode.py": textwrap.dedent('''
        def get_node_names():
            return ["/mavros", "/flight_workbench_probe", "/patrol_control"]
    '''),
    "rosservice.py": textwrap.dedent('''
        def get_service_list():
            return ["/legacy/Servo_raw", "/navigation/start_mission"]

        def get_service_type(name):
            if name == "/legacy/Servo_raw":
                return "patrol_control/Servo"
            return "std_srvs/Trigger"
    '''),
}


def write(path, text):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)


def make_shim(root, master_up):
    write(os.path.join(root, "rospy.py"),
          ("_up = True\n" if master_up else "_up = False\n") + ROSPY_UP)
    for name, body in FAKE_MSGS.items():
        path = os.path.join(root, name)
        if name.endswith("_mod.py"):
            path = os.path.join(root, name.replace("_mod.py", "msg.py"))
            write(path, body)
            continue
        write(path, body)
    # 把 msg.py 里的两个类重新导出到包级别
    write(os.path.join(root, "mavros_msgs", "msg", "__init__.py"),
          "from .msg import State, ExtendedState, RCOut, ActuatorControl, ESCStatus, ESCTelemetry\n")
    write(os.path.join(root, "std_msgs", "msg", "__init__.py"), "from .msg import String\n")


def run_probe(shim, topics, services, extra=None):
    env = dict(os.environ)
    env["PYTHONPATH"] = shim + os.pathsep + env.get("PYTHONPATH", "")
    result = subprocess.run(
        [sys.executable, os.path.join(TOOL_DIR, "board_probe.py"), "--once",
         "--interval", "0.2", "--topics", topics, "--services", services] + (extra or []),
        cwd=TOOL_DIR, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    lines = [line for line in result.stdout.decode("utf-8", "replace").splitlines()
             if line.strip().startswith("{")]
    if not lines and result.stderr and result.returncode != 2:
        sys.stderr.write("[probe stderr] %s\n" % result.stderr.decode("utf-8", "replace")[-1200:])
    return result, (json.loads(lines[-1]) if lines else None)


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print("%-4s %s%s" % ("PASS" if ok else "FAIL", name, ("  —— %s" % detail) if detail else ""))


def main():
    root = tempfile.mkdtemp(prefix="wb-probe-")
    try:
        shim_down = os.path.join(root, "down")
        make_shim(shim_down, master_up=False)
        result, row = run_probe(shim_down, "/mavros/state,/camera/image_raw", "/legacy/Servo_raw")
        check("无 master 时输出 master=false 且不崩溃",
              row is not None and row.get("master") is False and "error" in row,
              (row or {}).get("error", "无输出"))
        check("无 master 时不输出 state/nodes", row is not None and "state" not in row)

        shim_up = os.path.join(root, "up")
        make_shim(shim_up, master_up=True)
        result, row = run_probe(shim_up, "/mavros/state,/camera/image_raw,/navigation/mission_status",
                                "/legacy/Servo_raw")
        check("有 master 时输出一行 JSON", row is not None and row.get("master") is True,
              (row or {}).get("error", ""))
        row = row or {}
        state = row.get("state") or {}
        check("mavros_msgs/State 取值正确（回归：没有 .data 字段）",
              state.get("connected") is True and state.get("mode") == "OFFBOARD"
              and state.get("armed") is False and state.get("system_status") == 3,
              json.dumps(state, ensure_ascii=False))
        extended = row.get("extended") or {}
        check("ExtendedState 取值正确", extended.get("landed_state") == 1,
              json.dumps(extended, ensure_ascii=False))
        mission = row.get("mission")
        check("std_msgs/String 里的 JSON 被解析",
              isinstance(mission, dict) and mission.get("phase") == "HIGH_VIEW_SEARCH",
              json.dumps(mission, ensure_ascii=False))
        check("只订阅请求的话题 + 三个已知状态话题",
              set(row.get("topics", {})) == {"/mavros/state", "/camera/image_raw",
                                             "/navigation/mission_status",
                                             "/mavros/extended_state",
                                             "/uav_high_view/probe_status",
                                             "/board_trials/auto_land_status"},
              ",".join(sorted(row.get("topics", {}))))
        check("节点列表来自 rosnode", "/patrol_control" in (row.get("nodes") or []),
              ",".join(row.get("nodes") or []))
        check("服务类型读取正确",
              (row.get("services") or {}).get("/legacy/Servo_raw") == "patrol_control/Servo",
              json.dumps(row.get("services"), ensure_ascii=False))
        _, configured = run_probe(shim_up, '/mavros/state', '', [
            '--terminal-hover-topic', '/custom/hover', '--lio-realtime-topic', '/custom/lio'])
        check("可配置悬停状态只读订阅", isinstance(configured.get('terminal_hover'), dict)
              and '/custom/hover' in configured['topics'])
        lio = configured.get('lio_realtime') or []
        check("DiagnosticArray 保留级别与实时诊断值", len(lio) == 1 and lio[0]['level'] == 1
              and lio[0]['values']['output_age_sec'] == '0.23' and lio[0]['values']['lidar_queue'] == '2')
        check("未配置诊断不伪造健康状态", row.get('lio_realtime') is None
              and row.get('terminal_hover') is None)
        observed_topics={key:'/observe/'+key for key in ('fc_pose','lio_pose','ev_pose','setpoint',
            'battery','rc_out','esc_status','esc_telemetry','low_hover')}
        observed_result, observed = run_probe(shim_up, '', '', ['--observe-topics', json.dumps(observed_topics)])
        observation=observed.get('observe',{})
        check('Typed observe topics are subscribed with no camera data',
              observed_result.returncode==0 and set(observation)==set(observed_topics)
              and all(topic in observed['topics'] for topic in observed_topics.values())
              and not any('camera' in topic for topic in observed['topics']))
        for key in ('fc_pose','lio_pose','ev_pose','setpoint'):
            pose=observation.get(key) or {}
            check('Normalized pose/odometry and old source age: '+key,
                  pose.get('frame')=='camera_init' and pose.get('stamp')==96.
                  and pose.get('source_age')==4. and pose.get('x')==1. and pose.get('z')==.3
                  and pose.get('roll_deg')==0. and pose.get('pitch_deg')==0. and pose.get('yaw_deg')==0.)
        check('Battery NaN/Inf are null', observation.get('battery')==dict(voltage=14.8,current=None,percentage=None))
        check('RC output is channels only, never current', observation.get('rc_out')==dict(channels=[1100,1200,1300,1400]))
        status=(observation.get('esc_status') or {}).get('entries') or [{}]
        telemetry=(observation.get('esc_telemetry') or {}).get('entries') or [{}]
        check('ESCStatus keeps native quantities without a invented motor index',
              status[0].get('index') is None and status[0].get('slot')==0
              and status[0].get('rpm')==1234 and status[0].get('current')==2.3
              and status[0].get('temperature') is None)
        check('ESCTelemetry uses native array and finite temperatures',
              telemetry[0].get('index') is None and telemetry[0].get('rpm')==2345
              and telemetry[0].get('temperature')==39. and telemetry[0].get('current') is None)
        check('Low hover status is a JSON dictionary', (observation.get('low_hover') or {}).get('stage')=='HOLD_FOR_PILOT')
        check('Emitted JSON has no nonfinite constants',
              b'NaN' not in observed_result.stdout and b'Infinity' not in observed_result.stdout)
        for ending in ('bad_pose','zero_pose','bad_position','typed_error','fallback'):
            _, invalid = run_probe(shim_up, '', '', ['--observe-topics', json.dumps({'fc_pose':'/observe/'+ending})])
            check('Invalid pose/type stays null with packet count: '+ending,
                  invalid['observe']['fc_pose'] is None and invalid['topics']['/observe/'+ending]['count']==1)
        _, invalid_json=run_probe(shim_up,'','',['--observe-topics',json.dumps({'low_hover':'/observe/bad_json'})])
        _, nan_json=run_probe(shim_up,'','',['--observe-topics',json.dumps({'low_hover':'/observe/nan_json'})])
        check('Invalid JSON rejected and nested NaN sanitized', invalid_json['observe']['low_hover'] is None
              and nan_json['observe']['low_hover']['value'] is None)
        missing= os.path.join(root,'missing_optional')
        make_shim(missing,True)
        write(os.path.join(missing,'mavros_msgs/msg/__init__.py'),'from .msg import State, ExtendedState\n')
        _, optional=run_probe(missing,'','',['--observe-topics',json.dumps({'esc_status':'/observe/esc_status','rc_out':'/observe/rc_out'})])
        check('Unavailable optional message uses AnyMsg counts without typed data',
              optional['observe']==dict(esc_status=None,rc_out=None)
              and optional['topics']['/observe/esc_status']['count']==1
              and optional['topics']['/observe/rc_out']['count']==1)
        broken=os.path.join(root,'broken_optional')
        make_shim(broken,True)
        write(os.path.join(broken,'mavros_msgs/msg/__init__.py'),textwrap.dedent('''
            from .msg import State, ExtendedState, RCOut, ESCTelemetry
            def __getattr__(name):
                if name=='ESCStatus':raise RuntimeError('broken generated ESC decoder')
                raise AttributeError(name)
        '''))
        broken_result, broken_data=run_probe(broken,'','',['--observe-topics',json.dumps({'esc_status':'/observe/esc_status'})])
        check('Broken optional generated type preserves probe and AnyMsg counts',
              broken_result.returncode==0 and broken_data['observe']['esc_status'] is None
              and broken_data['topics']['/observe/esc_status']['count']==1)
        broken_clock=os.path.join(root,'broken_clock')
        make_shim(broken_clock,True)
        clock_rospy=os.path.join(broken_clock,'rospy.py')
        with open(clock_rospy,encoding='utf-8') as handle:clock_source=handle.read()
        write(clock_rospy,clock_source+textwrap.dedent('''
            _clock_reads=0
            _now=Time.now
            def broken_now():
                global _clock_reads
                _clock_reads+=1
                if _clock_reads>1:raise RuntimeError('clock unavailable at emitter')
                return _now()
            Time.now=staticmethod(broken_now)
        '''))
        clock_result, clock_data=run_probe(broken_clock,'','',['--observe-topics',json.dumps({'fc_pose':'/observe/fc_pose'})])
        check('Emitter clock failure clears only source age and keeps pose/probe',
              clock_result.returncode==0 and clock_data['observe']['fc_pose']['source_age'] is None
              and clock_data['observe']['fc_pose']['x']==1.
              and clock_data['observe']['fc_pose']['stamp']==96.)
        stale=os.path.join(root,'stale')
        make_shim(stale,True)
        stale_rospy=os.path.join(stale,'rospy.py')
        with open(stale_rospy,encoding='utf-8') as handle:shim_source=handle.read()
        write(stale_rospy,shim_source+textwrap.dedent('''
            import sys
            _tracker=sys.modules['__main__'].Tracker
            _snapshot=_tracker.snapshot
            def stale_snapshot(self):
                rows=_snapshot(self)
                if '/observe/stale' in rows:rows['/observe/stale']['age']=2.001
                return rows
            _tracker.snapshot=stale_snapshot
        '''))
        _, stale_data=run_probe(stale,'','',['--observe-topics',json.dumps({'fc_pose':'/observe/stale'})])
        check('Receive age above 2s hides pose but preserves age/count for annotation',
              stale_data['observe']['fc_pose'] is None
              and stale_data['topics']['/observe/stale']['age']==2.001
              and stale_data['topics']['/observe/stale']['count']==1)
        _, down_data=run_probe(shim_down,'','',['--observe-topics',json.dumps({'fc_pose':'/observe/fc_pose'})])
        check('No master cannot present configured pose data',
              down_data['master'] is False and down_data['observe']==dict(fc_pose=None))
        for mapping in ('not-json','[]','{"unknown":"/topic"}','{"fc_pose":"relative"}',
                        '{"fc_pose":"/same","lio_pose":"/same"}'):
            invalid_result,_=run_probe(shim_up,'','',['--observe-topics',mapping])
            check('Invalid observe mapping fails before starting: '+mapping,invalid_result.returncode==2)
        spec=importlib.util.spec_from_file_location('probe_unit',os.path.join(TOOL_DIR,'board_probe.py'))
        probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)
        check('Recursive JSON safety keeps nested NaN/Inf finite',
              probe.json_safe({'a':[float('nan'),float('inf'),2.]})=={'a':[None,None,2.]})
        _, actuator = run_probe(shim_up, '', '', ['--observe-topics', json.dumps({'actuator_target':'/mavros/target_actuator_control'})])
        check('Actuator target remains a separate raw control group with finite fields',
              actuator['observe']['actuator_target']==dict(group_mix=0,controls=[.1,.2,.3,.4,None,0.,0.,0.])
              and 'rc_out' not in actuator['observe'])
        diagnostic = probe.observation_diagnostic
        subscribed = dict(subscribed=True, typed=True)
        check('Publisher registration without packets is waiting, not receiving',
              diagnostic('/rc', {'count':0,'age':None}, {'/rc':['/mavros']}, subscribed, None)['status']=='waiting_message')
        check('Missing publisher is distinct from stale or malformed data',
              diagnostic('/rc', {'count':0}, {}, subscribed, None)['status']=='no_publisher'
              and diagnostic('/rc', {'count':1,'age':3}, {'/rc':['/mavros']}, subscribed, {})['status']=='stale'
              and diagnostic('/rc', {'count':1,'age':.1}, {'/rc':['/mavros']}, subscribed, None)['status']=='parse_error')
        check('Missing optional type and failed subscription are explicit',
              diagnostic('/rc', {}, None, dict(subscribed=True,typed=False), None)['status']=='unsupported_type'
              and diagnostic('/rc', {}, None, {}, None)['status']=='subscription_failed')
        late=os.path.join(root,'late_publisher')
        make_shim(late,True)
        with open(os.path.join(late,'rospy.py'),encoding='utf-8') as handle:late_source=handle.read()
        write(os.path.join(late,'rospy.py'),late_source+textwrap.dedent('''
            import time
            _sleeps=0
            _subscriber=Subscriber
            def Subscriber(topic,msg_type,callback,**kwargs):
                if topic=='/mavros/rc/out':
                    assert not any(row[0]==topic for row in Subscribers), 'duplicate healthy subscription'
                    Subscribers.append((topic,msg_type,callback))
                else:_subscriber(topic,msg_type,callback,**kwargs)
            def is_shutdown():return _sleeps>=2
            def sleep(seconds):
                global _sleeps
                _sleeps+=1
                for topic,kind,callback in Subscribers:
                    if topic=='/mavros/rc/out':callback(kind())
            time.sleep=sleep
        '''))
        write(os.path.join(late,'rosgraph.py'),textwrap.dedent('''
            import rospy
            class Master:
                def __init__(self,caller):pass
                def getSystemState(self):
                    return ([['/mavros/rc/out',['/mavros']]] if rospy._sleeps else [],[],[])
        '''))
        late_result=subprocess.run([sys.executable,os.path.join(TOOL_DIR,'board_probe.py'),
                '--nodes-interval','0','--observe-topics',json.dumps({'rc_out':'/mavros/rc/out'})],
                env=dict(os.environ,PYTHONPATH=late),capture_output=True,text=True,timeout=10)
        late_rows=[json.loads(line) for line in late_result.stdout.splitlines() if line.startswith('{')]
        check('Probe subscribed while MAVROS was offline receives a later publisher without duplicate subscriptions',
              late_result.returncode==0 and len(late_rows)==2
              and late_rows[0]['observe_status']['rc_out']['status']=='no_publisher'
              and late_rows[0]['topics']['/mavros/rc/out']['count']==0
              and late_rows[-1]['observe_status']['rc_out']['status']=='receiving'
              and late_rows[-1]['observe']['rc_out']['channels']==[1100,1200,1300,1400],late_result.stderr[-200:])
        source = open(os.path.join(TOOL_DIR, "board_probe.py"), encoding="utf-8").read()
        check("探针不发布、不调用服务（源码边界）",
              "Publisher(" not in source and "ServiceProxy(" not in source
              and "wait_for_service(" not in source and "set_mode" not in source)
    finally:
        shutil.rmtree(root, ignore_errors=True)

    failed = RESULTS.count(False)
    print("\n%d 项检查，%d 项失败" % (len(RESULTS), failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
