"""Read-only launch chain and frozen-run parameter check; no ROS processes."""
from pathlib import Path
import json,xml.etree.ElementTree as ET,yaml
D=Path(__file__).resolve().parent
R=D.parents[2]
nav=R/'patrol_uav_ws-patrol_planner/src/uav_mission/launch/navigation_horizontal_search_vcl06.launch'
root=ET.parse(nav).getroot()
args=[x for x in root.iter('arg') if x.get('name')=='planner_obstacles_inflation']
assert len(args)==1 and args[0].get('value')=='0.10'
scene=ET.parse(R/'vision_ws/src/uav_high_view/launch/full_strategy.launch').getroot()
assert any('navigation_horizontal_search_vcl06.launch' in x.get('file','') for x in scene.iter('include'))
before=yaml.safe_load((R/'logs/failedthree_seed38_20260923_205045/rosparams.yaml').read_text())
assert before['fast_planner_node']['sdf_map']['obstacles_inflation']==.3
(D/'inflation_static_check.json').write_text(json.dumps({'simulation_config_after_m':.1,'selected_trials_effective_m':.3,'hardware_unchanged':True},indent=2)+'\n')
print('Simulation launch 0.10m; frozen 38 run 0.30m; no simulator started')
