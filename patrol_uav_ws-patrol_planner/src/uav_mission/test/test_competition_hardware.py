import copy,json,tempfile,unittest,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from uav_mission.competition_config import validate,generate
from uav_mission.hardware_bag import topics_for
from uav_mission.high_view_probe import HighViewProbe,ProbeConfig

ROOT=Path(__file__).resolve().parents[4]
PKG=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'

class CompetitionTests(unittest.TestCase):
    def setUp(self):
        self.s=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        self.rig=yaml.safe_load((PKG/'config/competition/known_rig.yaml').read_text())
        self.s.update(site_confirmed=True,corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],landing_xy=[8.5,-4.2])
    def generate(self,z=0.):
        with tempfile.TemporaryDirectory() as d:
            ref=generate(ROOT,d,self.s,(.02,-.01,z),self.rig)
            docs={n:yaml.safe_load((Path(d)/(n+'.yaml')).read_text()) for n in ('runtime','control','overrides')}
        return ref,docs
    def test_unmeasured_example_can_check_but_cannot_fly(self):
        s=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        validate(s)
        with self.assertRaises(ValueError):validate(s,flight=True)
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):generate(ROOT,d,s,(0,0,0),self.rig)
    def test_missing_corridor_or_h_is_rejected_before_ros(self):
        for patch in (dict(corridor_waypoints=[]),dict(landing_xy=None),dict(site_confirmed=False)):
            with self.assertRaises(ValueError):validate(dict(self.s,**patch),flight=True)
    def test_all_heights_translate_with_ground_reference(self):
        a,ad=self.generate(0.);b,bd=self.generate(.1)
        for name in ('low_z','high_z','drop_z','ground_z','takeoff_z'):self.assertAlmostEqual(b[name]-a[name],.1,places=6)
        for ai,bi in zip(ad['runtime']['mission']['post_delivery_route'],bd['runtime']['mission']['post_delivery_route']):self.assertAlmostEqual(bi[2]-ai[2],.1)
        for k in ('/release_permission_arbiter/min_release_altitude','/release_permission_arbiter/max_release_altitude','/external_planner_max_command_z'):self.assertAlmostEqual(bd['overrides'][k]-ad['overrides'][k],.1)
        self.assertEqual(ad['control']['align_height'],ad['control']['uav_vision']['recovery_height'])
    def test_targets_are_searched_not_injected_and_h_is_final(self):
        ref,d=self.generate();rt=d['runtime'];c=d['control']
        self.assertEqual(rt['runtime']['start_mode'],'full')
        self.assertNotIn('trial',rt)
        self.assertNotIn('manual_waypoints',rt['search'])
        self.assertEqual(rt['mission']['landing_xy'],[8.52,-4.21])
        self.assertEqual(rt['mission']['post_delivery_route'][-1][:2],rt['mission']['landing_xy'])
        self.assertAlmostEqual(rt['mission']['post_delivery_route'][-1][2],c['external_landing']['capture_height'])
        self.assertTrue(c['switch']['auto_land'])
    def test_search_stays_inside_then_corridor_opens(self):
        _,d=self.generate();o=d['overrides'];rt=d['runtime']
        self.assertTrue(o['/fast_planner_node/sdf_map/search_region/enabled'])
        tail=rt['mission']['post_delivery_parameter_stages'][0]['parameters']
        self.assertFalse(tail['/fast_planner_node/sdf_map/search_region/enabled'])
        self.assertEqual(o['/fast_planner_node/sdf_map/virtual_ceil_height'],-.1)
        self.assertEqual(o['/fast_planner_node/sdf_map/obstacles_inflation'],.25)
        self.assertEqual(o['/fast_planner_node/sdf_map/obstacles_inflation_up'],.2)
        self.assertEqual(o['/fast_planner_node/sdf_map/obstacles_inflation_down'],.1)
        self.assertEqual(d['control']['drop_system']['slot_offsets'],self.rig['slot_offsets'])
    def test_outside_survey_and_invalid_altitudes_rejected(self):
        for patch in (dict(survey_xy=[[8.,0.]]),dict(high_agl=3.1),dict(drop_agl=.2),dict(map_size=[10.,10.,4.]),dict(raw_servo_service='/Servo'),dict(actuator_mode='mock'),dict(mode='memory_only'),dict(following_speed_profile=dict(cruise_lead_m=1.4))):
            with self.assertRaises(ValueError):validate(dict(self.s,**patch),flight=True)
    def test_bag_is_local_cloud_and_no_direct_video(self):
        topics=topics_for(self.s)
        self.assertIn('/sdf_map/occupancy_inflate',topics)
        self.assertIn('/competition/run_metadata',topics)
        self.assertNotIn('/freedom/static_pointcloud',topics)
        self.assertNotIn('/livox/lidar',topics)
        self.assertFalse(any(t.startswith('/board_trials/') for t in topics))
    def test_shared_guard_still_rejects_jump(self):
        # Exercise the production guard without constructing an unrelated mission profile.
        import threading
        probe=HighViewProbe.__new__(HighViewProbe)
        probe._lock=threading.RLock()
        from types import SimpleNamespace
        probe.core=SimpleNamespace(config=SimpleNamespace(mission_frame='camera_init'))
        probe.pose=None;probe.pose_stamp=None
        probe.update_pose((0.,0.,2.),10.,'camera_init')
        with self.assertRaisesRegex(ValueError,'pose discontinuity'):probe.update_pose((0.,0.,2.45),10.03,'camera_init')
    def test_app_uses_one_formal_manager_and_guard(self):
        tree=ET.parse(PKG/'launch/competition_application.launch')
        nodes=tree.findall('.//node')
        self.assertEqual([(n.get('pkg'),n.get('type')) for n in nodes],[('uav_mission','navigation_competition_manager.py')])
        source=(PKG/'launch/competition_application.launch').read_text()
        self.assertNotIn('uav_board_trials',source);self.assertNotIn('mock_servo',source)
        guard=[e for e in tree.findall('.//include') if e.get('file','').endswith('/release_guard.launch')][0]
        args={a.get('name'):a.get('value') for a in guard}
        self.assertEqual(args['public_service_name'],'/Servo')
        self.assertEqual(args['require_evidence_context'],'true')
    def test_controller_and_servo_are_entity_sources(self):
        actuator=PKG.parent/'actuator_pwm'
        self.assertTrue((actuator/'src/pwm_node1.cpp').is_file())
        self.assertFalse(actuator.is_symlink())
        launch=ET.parse(actuator/'launch/launch_all.launch')
        self.assertEqual(launch.find('.//remap').get('to'),'/legacy/Servo_raw')

if __name__=='__main__':unittest.main()
