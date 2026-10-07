"""Production-generator and launch contracts; no ROS or C++ execution."""
from pathlib import Path
import copy
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[4]
PKG = ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'
TRIAL = ROOT/'deployment/board_trials_4x4/common/uav_board_trials'
VISION = ROOT/'vision_ws/src/uav_vision'
sys.path[:0] = [str(PKG/'src'), str(TRIAL/'scripts'), str(ROOT/'vision_ws/src/uav_high_view/src')]
from uav_mission.competition_config import generate as competition_generate

class SlotHEntryWiring(unittest.TestCase):
    def controls(self):
        rig = yaml.safe_load((PKG/'config/competition/known_rig.yaml').read_text())
        for rel in ['field.example.yaml','field_20261007_validated.yaml','candidates/rectangle_motion.yaml','candidates/snake_motion.yaml']:
            settings = yaml.safe_load((ROOT/'deployment/competition'/rel).read_text())
            if not settings['corridor_waypoints'] or settings['landing_xy'] is None:
                settings.update(site_confirmed=True, corridor_waypoints=[
                    dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),
                    dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],landing_xy=[8.5,-4.2])
            with tempfile.TemporaryDirectory() as d:
                ref = competition_generate(ROOT,d,settings,(0.,0.,0.),rig)
                control = yaml.safe_load((Path(d)/'control.yaml').read_text())
            yield rel, ref, control
        if TRIAL.is_dir():
            from trial_config import generate, apply_site_profile
            rig = yaml.safe_load((TRIAL/'config/known_rig.yaml').read_text())
            for folder in ['03_h_landing','04_corridor_landing','08_full_mission']:
                settings = yaml.safe_load((ROOT/'deployment/board_trials_4x4'/folder/'settings.yaml').read_text())
                if folder != '03_h_landing':
                    apply_site_profile(settings,dict(corridor_waypoints=[
                        dict(x=.6,y=0.,agl=1.),dict(x=1.5,y=.4,agl=1.)],landing_xy=[2.5,0.]))
                with tempfile.TemporaryDirectory() as d:
                    ref = generate(ROOT,d,settings,(0.,0.,0.),rig)
                    control = yaml.safe_load((Path(d)/'control.yaml').read_text())
                yield folder, ref, control

    def test_actual_generators_emit_feedback_motion_offsets_and_posctl(self):
        feedback = yaml.safe_load((VISION/'config/drop_aligner.yaml').read_text())['drop_alignment_feedback_topic']
        for profile,ref,c in self.controls():
            with self.subTest(profile=profile):
                self.assertEqual(c['drop_system']['alignment_feedback_topic'],feedback)
                self.assertTrue(c['drop_system']['compensated_alignment'])
                self.assertEqual(c['drop_system']['slot_offset_semantics'],'body_flu_lever_arm')
                for key in ['slot_offsets','dynamic_slot_offsets']:
                    self.assertEqual(c['drop_system'][key],[[-.12,0.],[0.,-.12],[0.,.12]])
                self.assertEqual(c['motion_feedback'],dict(odom_topic='/mavros/local_position/odom',twist_frame='child'))
                self.assertEqual(c['external_landing']['handoff_mode'],'POSCTL')
                self.assertAlmostEqual(c['land_height']-ref['ground_z'],.35)
                self.assertAlmostEqual(c['external_landing']['auto_land_height']-ref['ground_z'],.37)
                self.assertEqual(c['external_landing']['posctl'],dict(
                    xy_tolerance_m=.05,height_tolerance_m=.02,capture_height_tolerance_m=.10,
                    max_horizontal_speed_mps=.03,max_vertical_speed_mps=.05,
                    stable_duration_sec=.5,max_odom_age_sec=.2,max_sample_gap_sec=.2,min_samples=3))

    def test_controller_entries_load_generated_config_and_remap_motion_odom(self):
        apps = [TRIAL/'launch/application.launch',PKG/'launch/competition_application.launch']
        present = [p for p in apps if p.is_file()]
        self.assertTrue(present)
        for app in present:
            with self.subTest(entry=app.name):
                tree = ET.parse(app)
                group = next(g for g in tree.findall('./group') if any(
                    x.get('from')=='/mavros/local_position/odom' for x in g.findall('remap')))
                self.assertEqual(next(x.get('to') for x in group.findall('remap')
                    if x.get('from')=='/mavros/local_position/odom'),'/navigation/local_odom')
                inc = next(x for x in group.findall('include')
                    if x.get('file','').endswith('/patrol_control_px4_sim.launch'))
                args = {x.get('name'):x.get('value') for x in inc.findall('arg')}
                self.assertEqual(args['waypoint_config'],'$(arg generated_dir)/control.yaml')
                self.assertEqual(args['external_mission_mode'],'true')
        ctrl = ET.parse(ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_control_px4_sim.launch')
        self.assertTrue(any(x.get('file')=='$(arg waypoint_config)' for x in ctrl.findall('./rosparam')))
        cpp = (ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp').read_text()
        for name in ['motion_feedback/odom_topic','motion_feedback/twist_frame','drop_system/alignment_feedback_topic']:
            self.assertIn('"'+name+'"',cpp)

    def test_formal_visual_entries_enable_matching_feedback_and_cache(self):
        apps = [TRIAL/'launch/application.launch',PKG/'launch/competition_application.launch']
        count = 0
        for app in apps:
            if not app.is_file(): continue
            for inc in ET.parse(app).findall('./include'):
                ref = inc.get('file','')
                if not any(ref.endswith('/'+name) for name in [
                    'phase_d.launch','phase_d_board.launch','competition_vision_interface.launch']):continue
                count += 1
                values = {x.get('name'):x.get('value') for x in inc.findall('arg')}
                self.assertEqual(values['require_alignment_context'],'true')
                package,tail = ref[7:].split(')/',1)
                launch = ({'uav_vision':VISION,'uav_mission':PKG}[package])/tail
                tree = ET.parse(launch)
                args = {x.get('name'):x.get('default') for x in tree.findall('./arg')}
                args.update(values)
                def resolve(value):
                    for _ in range(4):
                        if value and value.startswith('$(arg ') and value.endswith(')'):
                            value = args[value[6:-1]]
                        else: break
                    return value
                node = next(n for n in tree.findall('./node') if n.get('name')=='drop_aligner')
                self.assertTrue(any(x.get('file')=='$(find uav_vision)/config/drop_aligner.yaml'
                    for x in node.findall('rosparam')))
                params = yaml.safe_load((VISION/'config/drop_aligner.yaml').read_text())
                params.update({x.get('name'):resolve(x.get('value')) for x in node.findall('param')})
                self.assertEqual(params['require_compensated_alignment'],'true')
                self.assertEqual(params['require_alignment_context'],'true')
                self.assertEqual(params['drop_alignment_feedback_topic'],'/uav_vision/drop_alignment_feedback')
                self.assertEqual(params['compensated_observation_cache_size'],16)
        self.assertGreater(count,0)
        aligner = (VISION/'scripts/drop_aligner.py').read_text()
        self.assertIn('rospy.get_param(',aligner)
        self.assertIn('"~drop_alignment_feedback_topic"',aligner)
        self.assertIn('DropAlignmentFeedback',aligner)

    def test_high_capture_and_low_handoff_keep_independent_height_tolerances(self):
        cpp = (ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp').read_text()
        self.assertIn('"external_landing/posctl/capture_height_tolerance_m", 0.10',cpp)
        self.assertIn('landing_capture_window_ = LandingHandoffStabilityWindow(landing_capture_config_)',cpp)
        self.assertIn('landing_handoff_window_ = LandingHandoffStabilityWindow(landing_settle_config_)',cpp)
        self.assertIn('const char** rejection = nullptr',
            (ROOT/'patrol_uav_ws-patrol_planner/src/patrol_control/include/patrol_control/patrol_control.h').read_text())

if __name__ == '__main__':
    unittest.main(verbosity=2)
