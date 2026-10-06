from pathlib import Path
import os,sys,tempfile,subprocess,json
import roslaunch,rospkg,yaml
from trial_config import TRIAL_FOLDERS,H_MODES,mapping_profile
P=Path(__file__).resolve().parents[1];root=P.parents[3]
roslaunch.substitution_args._rospack=rospkg.RosPack(ros_paths=[str(root/'vision_ws/src'),str(root/'patrol_uav_ws-patrol_planner/src'),'/opt/ros/noetic/share','/home/xhj/PX4-Autopilot','/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic'])
rows=[]
for trial in TRIAL_FOLDERS:
    if trial=='high_speed_capture':continue  # Field capture profile has no 4x4 SITL fixture.
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([sys.executable,str(P/'scripts/prepare_simulation.py'),trial,tmp],check=True,capture_output=True)
        settings=yaml.safe_load((Path(tmp)/'settings.yaml').read_text())
        os.environ['SIM_RUN_DIR']=tmp
        cfg=roslaunch.config.load_config_default([(str(P/'launch/simulation.launch'),[f'generated_dir:={tmp}',f'trial:={trial}',f'mode:={settings["mode"]}','model_path:=/test.pt'])],11311,verbose=False)
        nodes={n.name:n for n in cfg.nodes};params={k:v.value for k,v in cfg.params.items()}
        assert params['/landing_detector/landing_enable_h_stroke_fallback']==(settings['mode'] in H_MODES)
        assert params['/landing_detector/landing_enable_h_structure_check'] is True
        assert params['/landing_detector/default_align_mode']=='disabled'
        assert params['/landing_detector/process_only_in_landing_mode'] is True
        assert nodes['mission_manager'].type=='trial_sim_manager.py'
        assert 'target_detector_rknn' not in nodes
        assert nodes['map_camera_alignment'].type=='static_transform_publisher'
        assert params['/fast_planner_node/sdf_map/virtual_ceil_height']==-.1
        assert params['/fast_planner_node/sdf_map/obstacles_inflation']==.25
        assert params['/navigation/mission_manager/high_view_full/grid/inflation']==.25
        assert params['/navigation/mission_manager/high_view_full/policy/interrupt_refined_classes']==[]
        assert params['/feature_extract_enable'] is True
        assert params['/cube_side_length']==20. and params['/mapping/det_range']==6.
        assert params['/preprocess/lidar_type']==4  # simulator remains PointCloud2
        assert params['/freedom/map/voxel_depth']==2
        distance,top={'low':(6.,2.),'high':(6.,3.2),'corridor':(5.,1.5)}[mapping_profile(settings)]
        assert params['/freedom/sensor/max_range']==distance
        assert params['/freedom/map/raycast_max_range']==distance
        assert params['/freedom/sensor/max_z']==params['/freedom/map/raycast_max_z']==top

        assert 'overview_video_recorder' in nodes and 'trial_recorder' in nodes
        assert not any(n.package=='actuator_pwm' for n in cfg.nodes)
        rows.append(dict(trial=trial,nodes=len(nodes),passed=True))
print(json.dumps(rows,indent=2))
