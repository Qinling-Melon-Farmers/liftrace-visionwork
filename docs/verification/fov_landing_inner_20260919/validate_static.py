from pathlib import Path
import json, xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import roslaunch, yaml
R=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search')
D=R/'docs/verification/fov_landing_inner_20260919'
scene=D/'seed_2672'
model=ET.parse(R/'vision_ws/src/uav_vision_eval/models/iris_mid360_start_fov/model.sdf')
for link in model.findall('.//link'):
    if link.find('.//camera') is not None:
        pose=[float(x) for x in link.findtext('pose').split()]
        actual=Rotation.from_euler('xyz',pose[3:]).as_matrix() @ np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        np.testing.assert_allclose(actual,Rotation.from_quat([0,1,0,0]).as_matrix(),atol=1e-5)
        np.testing.assert_allclose(actual[:2,:2].flatten(),[-1,0,0,1],atol=1e-5)
results={}
for label,fast,alt in [('A','false','2.6'),('B','true','2.6'),('C','true','3.0')]:
    config=roslaunch.config.load_config_default([(str(R/'vision_ws/src/uav_high_view/launch/fov_inner_repair.launch'),[f'scene_dir:={scene}',f'corridor_fast:={fast}',f'high_agl:={alt}','target_model_path:=/home/xhj/liftrace/vision_ws/runs/liftrace_6cls_v5_merged_standard_20260714/weights/best.pt'])],11311,verbose=False)
    values={k:v.value for k,v in config.params.items()}
    (D/f'expanded_{label}.yaml').write_text(yaml.safe_dump(values,sort_keys=True))
    tf=[n.args for n in config.nodes if n.name=='downward_camera_extrinsic']
    assert len(tf)==1 and '0 1 0 0' in tf[0],tf
    assert values['/uav_vision/pixel_to_body_matrix']==[-1.,0.,0.,1.]
    assert values['/fast_planner_node/sdf_map/search_region/enabled'] is True
    assert values['/simulation/landing_phase_active'] is False
    assert values['/navigation_vcl06_assertion/post_delivery_gate/landing_observation_region/max_height']==1.2
    assert values['/simulation/px4_parameters/MPC_LAND_SPEED']==.25
    results[label]={'camera_tf':tf[0],'parameters':len(values),'high_agl':alt,'corridor_fast':fast}
print(json.dumps(results,indent=2))
(D/'static_validation.json').write_text(json.dumps(results,indent=2)+'\n')
