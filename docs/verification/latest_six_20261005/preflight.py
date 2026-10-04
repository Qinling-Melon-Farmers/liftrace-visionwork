"""Offline scene, policy, source-overlay and gate checks. No ROS nodes started."""
from pathlib import Path
import importlib.util,json,os,sys,xml.etree.ElementTree as ET
import yaml,rospkg
from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader
import roslaunch.substitution_args
R=Path(__file__).resolve().parents[3];D=Path(__file__).resolve().parent
from uav_mission.high_view_probe import ProbeConfig
from uav_mission.motion_optimization import MotionOptimization
spec=importlib.util.spec_from_file_location('route_gate_check',R/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_vcl06_assertion.py');gate=importlib.util.module_from_spec(spec);sys.modules[spec.name]=gate;spec.loader.exec_module(gate)
for name,expected in [('uav_mission',R/'patrol_uav_ws-patrol_planner/src/uav_mission'),('uav_vision',R/'vision_ws/src/uav_vision'),('uav_high_view',R/'vision_ws/src/uav_high_view')]:assert Path(rospkg.RosPack().get_path(name)).resolve()==expected.resolve()
roslaunch.substitution_args._rospack=rospkg.RosPack(ros_paths=[str(R/'patrol_uav_ws-patrol_planner/src'),str(R/'vision_ws/src'),'/home/xhj/PX4-Autopilot','/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic','/opt/ros/noetic/share'])
rows=[]
for case in json.loads((D/'scenes.json').read_text()):
 print('CHECK',case['seed'],case['variant'],flush=True)
 scene=R/case['scene'];os.environ['SIM_RUN_DIR']=str(R/'logs/preflight_route_speed')
 cfg=ROSLaunchConfig();XmlLoader().load(str(D/'replay.launch'),cfg,verbose=False,argv=[f'scene_dir:={scene}',f'field_seed:={case["seed"]}','target_model_path:=/home/xhj/liftrace/deliverables/liftrace_five_class_20260928_models/flight_5cls_20260928.pt'])
 p={k:v.value for k,v in cfg.params.items()};prefix='/navigation/mission_manager/'
 assert p[prefix+'high_view_probe/config/high_agl']==2.76
 assert p[prefix+'high_view_probe/config/survey_xy']==case['survey']
 assert p['/fast_planner_node/sdf_map/obstacles_inflation']==.25
 assert p['/fast_planner_node/sdf_map/horizontal_avoidance/enabled']
 assert p['/landing_detector/landing_enable_h_stroke_fallback']
 assert p['/target_detector/pause_in_landing_mode']
 assert p['/navigation_vcl06_assertion/field/max_x']-p['/navigation_vcl06_assertion/field/min_x']==10
 assert p['/navigation_vcl06_assertion/field/max_y']-p['/navigation_vcl06_assertion/field/min_y']==10
 for node in cfg.nodes:assert node.package not in ('actuator_pwm','livox_ros_driver2')
 rt=yaml.safe_load((scene/'fast_runtime.yaml').read_text());g=yaml.safe_load((scene/'fast_gate.yaml').read_text())['post_delivery_gate'];m=rt['mission']
 reducer=gate.Vcl06GateReducer(post_delivery_route=m['post_delivery_route'],post_delivery_doors=g['doors'],expected_door_order=g['expected_door_order'],allow_shared_door_route_indices=g.get('allow_shared_route_indices',False))
 enabled=case['variant']!='rectangle_baseline'
 assert p[prefix+'motion_optimization/enabled']==enabled
 assert p[prefix+'planner_line_preference/weight']==2.0
 assert p['/fast_planner_node/search/line_deviation_weight']==2.0
 assert p['/navigation/planner_bridge/execution/odom_twist_frame']=='child'
 assert p['/navigation/planner_bridge/execution/odom_velocity_available']

 if enabled:assert p['/navigation/planner_bridge/target/recovery_height']==p['/uav_vision/recovery_height']
 rows.append(dict(seed=case['seed'],variant=case['variant'],nodes=len(cfg.nodes),gate_doors=len(reducer.post_delivery_doors),status='PASS',motion=enabled))
 (scene/'effective_parameters.json').write_text(json.dumps(p,indent=2))
(D/'preflight.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
