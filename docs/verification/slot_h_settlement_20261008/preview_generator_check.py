from pathlib import Path
import sys,json,copy,tempfile,xml.etree.ElementTree as ET
import yaml
b=Path(__file__).resolve().parents[3]
base=b/'deployment/board_trials_4x4'; mission=b/'patrol_uav_ws-patrol_planner/src/uav_mission'
sys.path[:0]=[str(base/'common/uav_board_trials/scripts'),str(mission/'src'),str(b/'vision_ws/src/uav_high_view/src')]
import trial_config
rig=yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
out=b/'docs/verification/slot_h_settlement_20261008/offline_preview';out.mkdir(parents=True,exist_ok=True)
records=[]
for folder in ['03_h_landing','04_corridor_landing','08_full_mission']:
 original=yaml.safe_load((base/folder/'settings.yaml').read_text());settings=copy.deepcopy(original);fixture=False
 if folder!='03_h_landing':
  fixture=True
  trial_config.apply_site_profile(settings,dict(corridor_waypoints=[dict(x=.6,y=0.,agl=1.),dict(x=1.5,y=.4,agl=1.)],landing_xy=[2.5,0.]))
 for z in [0.,-.05,.09]:
  dest=out/folder/('fc_z_'+str(z).replace('-','minus'))
  ref=trial_config.generate(b,dest,settings,(0.,0.,z),rig)
  c=yaml.safe_load((dest/'control.yaml').read_text());o=yaml.safe_load((dest/'overrides.yaml').read_text())
  expected= {k:v for k,v in settings['landing_posctl'].items() if k not in ['target_agl','trigger_agl']}
  assert abs(c['land_height']-ref['ground_z']-.35)<1e-12
  assert abs(c['external_landing']['auto_land_height']-ref['ground_z']-.37)<1e-12
  assert c['external_landing']['posctl']==expected
  assert c['motion_feedback']==dict(odom_topic='/mavros/local_position/odom',twist_frame='child')
  assert c['drop_system']['compensated_alignment'] is True
  assert c['drop_system']['slot_offset_semantics']=='body_flu_lever_arm'
  assert c['drop_system']['slot_offsets']==[[-.12,0.],[0.,-.12],[0.,.12]]
  assert c['external_landing']['handoff_mode']=='POSCTL'
  records.append(dict(folder=folder,fixture=fixture,fc_local_z=z,ground_z=ref['ground_z'],land_local_z=c['land_height'],trigger_local_z=c['external_landing']['auto_land_height'],posctl=c['external_landing']['posctl'],motion_feedback=c['motion_feedback'],effective_odom_topic='/navigation/local_odom',drop_semantics=c['drop_system']['slot_offset_semantics'],drop_compensated_alignment=c['drop_system']['compensated_alignment'],generated_dir=str(dest.relative_to(b))))
 # Check supplied values, not only defaults, flow all the way into generated files.
 custom=copy.deepcopy(settings)
 custom['landing_posctl'].update(target_agl=.36,trigger_agl=.38,xy_tolerance_m=.045,height_tolerance_m=.018,max_horizontal_speed_mps=.025,max_vertical_speed_mps=.045,stable_duration_sec=.6,max_odom_age_sec=.18,max_sample_gap_sec=.18,min_samples=4)
 with tempfile.TemporaryDirectory(prefix='slot_h_custom_preview_') as d:
  ref=trial_config.generate(b,d,custom,(0.,0.,0.),rig);c=yaml.safe_load((Path(d)/'control.yaml').read_text())
  assert abs(c['land_height']-ref['ground_z']-.36)<1e-12
  assert abs(c['external_landing']['auto_land_height']-ref['ground_z']-.38)<1e-12
  assert c['external_landing']['posctl']=={k:v for k,v in custom['landing_posctl'].items() if k not in ['target_agl','trigger_agl']}
 auto=copy.deepcopy(settings);auto['landing_handoff_mode']='AUTO.LAND'
 with tempfile.TemporaryDirectory(prefix='slot_h_autoland_preview_') as d:
  ref=trial_config.generate(b,d,auto,(0.,0.,0.),rig);c=yaml.safe_load((Path(d)/'control.yaml').read_text())
  assert abs(c['land_height']-ref['ground_z']-.40)<1e-12
  assert abs(c['external_landing']['auto_land_height']-ref['ground_z']-.55)<1e-12
  assert 'posctl' not in c['external_landing']
 assert yaml.safe_load((base/folder/'settings.yaml').read_text())==original
 assert not any(n=='rospy' or n.startswith('roslaunch') for n in sys.modules)
app=ET.parse(base/'common/uav_board_trials/launch/application.launch').getroot()
group=next(g for g in app.findall('group') if any(x.attrib.get('from')=='/mavros/local_position/odom' for x in g.findall('remap')))
assert next(x.attrib['to'] for x in group.findall('remap') if x.attrib.get('from')=='/mavros/local_position/odom')=='/navigation/local_odom'
assert any(x.attrib.get('name')=='waypoint_config' and x.attrib.get('value')=='$(arg generated_dir)/control.yaml' for x in group.findall('include/arg'))
ctrl=ET.parse(b/'patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_control_px4_sim.launch').getroot()
assert any(x.attrib.get('file')=='$(arg waypoint_config)' for x in ctrl.findall('rosparam'))
nodes=ctrl.findall('node');assert any(x.attrib.get('pkg')=='patrol_control' for x in nodes)
vision={}
for name in ['phase_d.launch','phase_d_board.launch']:
 v=ET.parse(b/'vision_ws/src/uav_vision/launch'/name).getroot()
 args={x.attrib['name']:x.attrib.get('default') for x in v.findall('arg')}
 assert args['require_compensated_alignment']=='$(arg require_alignment_context)'
 inc=next(x for x in app.findall('include') if x.attrib['file'].endswith('/'+name))
 assert next(x.attrib['value'] for x in inc.findall('arg') if x.attrib['name']=='require_alignment_context')=='true'
 node=next(n for n in v.findall('node') if n.attrib.get('name')=='drop_aligner')
 assert next(x.attrib['value'] for x in node.findall('param') if x.attrib['name']=='require_compensated_alignment')=='$(arg require_compensated_alignment)'
 assert any(x.attrib.get('file')=='$(find uav_vision)/config/drop_aligner.yaml' for x in node.findall('rosparam'))
 cfg=yaml.safe_load((b/'vision_ws/src/uav_vision/config/drop_aligner.yaml').read_text())
 assert cfg['compensated_observation_cache_size']==16
 vision[name]=dict(require_alignment_context=True,require_compensated_alignment=True,compensated_observation_cache_size=16)
result=dict(status='PASS_OFFLINE_GENERATOR_AND_STATIC_WIRING',ros_started=False,cpp_compiled=False,preview_entry='run_trial.py -> trial_config.generate (offline direct invocation; live preview CLI not run)',synthetic_reference=True,records=records,checks=dict(custom_parameter_consumption=3,auto_land_unchanged=3,settings_preserved=3,vision_launch=vision),dynamic_status='PENDING_MAIN_GAZEBO_RESULT')
(out/'SUMMARY.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
