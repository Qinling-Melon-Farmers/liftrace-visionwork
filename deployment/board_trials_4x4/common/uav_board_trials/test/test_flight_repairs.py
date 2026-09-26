"""Regression for the failures found by comparing the actual flight entry."""
from pathlib import Path
import tempfile
import unittest
import yaml
from trial_config import generate, validate_settings

ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT / 'deployment/board_trials_4x4'


class FlightRepairs(unittest.TestCase):
    def test_recovery_goal_exceeds_handoff_for_every_ground_reference(self):
        rig = yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for folder in ('01_visual_interrupt', '02_high_view_revisit', '03_h_landing'):
            settings = yaml.safe_load((BASE/folder/'settings.yaml').read_text())
            for ground_fc in (-.09, 0., .09):
                with tempfile.TemporaryDirectory() as directory:
                    generate(ROOT, directory, settings, (0.,0.,ground_fc), rig)
                    control = yaml.safe_load((Path(directory)/'control.yaml').read_text())
                    params = yaml.safe_load((Path(directory)/'overrides.yaml').read_text())
                    vision = control['uav_vision']
                    for name in ('standard_recovery_setpoint_height', 'cross_recovery_setpoint_height'):
                        self.assertGreater(vision[name], vision['recovery_height']+.09)
                        self.assertLess(vision[name], params['/external_planner_max_command_z'])
                    self.assertEqual(vision['recovery_height'], params['/navigation/planner_bridge/target/recovery_height'])

    def test_same_geometry_for_low_and_high_but_only_high_extrudes(self):
        rig = yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for folder in ('01_visual_interrupt','02_high_view_revisit','03_h_landing'):
            settings = yaml.safe_load((BASE/folder/'settings.yaml').read_text())
            with tempfile.TemporaryDirectory() as directory:
                ref=generate(ROOT,directory,settings,(0.,0.,0.),rig)
                params=yaml.safe_load((Path(directory)/'overrides.yaml').read_text())
                prefix='/fast_planner_node/sdf_map/'
                self.assertEqual([params[prefix+k] for k in ('obstacles_inflation','obstacles_inflation_up','obstacles_inflation_down')],[.25,.2,.1])
                self.assertEqual(params[prefix+'horizontal_avoidance/enabled'],folder=='02_high_view_revisit')
                self.assertAlmostEqual(params[prefix+'horizontal_avoidance/obstacle_min_z']-ref['ground_z'],.4)

    def test_forward_line_has_intermediate_goals_before_three_metre_endpoint(self):
        settings=yaml.safe_load((BASE/'01_visual_interrupt/settings.yaml').read_text())
        validate_settings(settings)
        self.assertIn(1.8,settings['search_line_x'])
        self.assertLessEqual(max(b-a for a,b in zip(settings['search_line_x'],settings['search_line_x'][1:])),.61)
        for invalid in ([.6,4.0],[1.8,.6],[.6,float('nan')]):
            with self.assertRaises(ValueError):validate_settings(dict(settings,search_line_x=invalid))

    def test_rig_image_left_projects_forward(self):
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        a,b,c,d=rig['pixel_to_body_matrix']
        self.assertEqual((a*-1+b*0,c*-1+d*0),(1.,0.))
        # A 180-degree Y rotation maps optical-right to body-rear, optical-
        # down to body-left and optical-forward to body-down.
        self.assertEqual(rig['camera_quat_xyzw'],[0.,1.,0.,0.])


if __name__ == '__main__':
    unittest.main()
