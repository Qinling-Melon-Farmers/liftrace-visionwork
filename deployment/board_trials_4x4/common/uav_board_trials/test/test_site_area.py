from pathlib import Path
import unittest,copy,tempfile,yaml
from trial_config import generate,validate_settings
from trial_bag import topics_for
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/"deployment/board_trials_4x4"
SITE=yaml.safe_load((ROOT/"deployment/site_20260928/test_area.yaml").read_text())
class SiteArea(unittest.TestCase):
    def test_five_routes_and_all_bounds_share_approved_area(self):
        for folder in ("01_visual_interrupt","05_low_multi","07_memory_only","02_high_view_revisit","06_high_priority"):
            settings=yaml.safe_load((BASE/folder/"settings.yaml").read_text());settings.update(SITE)
            rig=yaml.safe_load((BASE/"common/uav_board_trials/config/known_rig.yaml").read_text())
            with tempfile.TemporaryDirectory() as out:
                generate(ROOT,out,settings,(.1,-.1,0.),rig)
                rt=yaml.safe_load((Path(out)/"runtime.yaml").read_text())
                ov=yaml.safe_load((Path(out)/"overrides.yaml").read_text())
                expected=[-.25,6.1,-1.6,1.4]
                for values in [rt["high_view_full"]["grid"]["bounds"],rt["high_view_full"]["boundary_policy"]["bounds"]]:
                    for a,b in zip(values,expected):self.assertAlmostEqual(a,b)
                self.assertAlmostEqual(ov["/fast_planner_node/sdf_map/search_region/max_x"],6.1)
                self.assertAlmostEqual(rt["trial"]["waypoints"][-1][0],6.1)
                for x,y in rt["high_view_probe"]["config"]["survey_xy"]:
                    self.assertTrue(expected[0]<=x<=expected[1] and expected[2]<=y<=expected[3])
                self.assertEqual(settings["cruise_speed"],.5)
                self.assertEqual(ov["/fast_planner_node/sdf_map/virtual_ceil_height"],-.1)
    def test_high_strategy_rejects_nine_points_before_ros_start(self):
        settings=yaml.safe_load((BASE/'07_memory_only/settings.yaml').read_text())
        settings.update(copy.deepcopy(SITE))
        self.assertEqual(len(settings['flight_area']['survey_xy']),8)
        validate_settings(settings)
        settings['flight_area']['survey_xy'].insert(-1,[.6,0.])
        with self.assertRaisesRegex(ValueError,'invalid probe configuration'):
            validate_settings(settings)
    def test_invalid_or_mismatched_geometry_rejected(self):
        settings=yaml.safe_load((BASE/"01_visual_interrupt/settings.yaml").read_text())
        for change in [dict(map_size=[10.,6.,3.8]),dict(survey_xy=[[.6,0],[7,0],[.6,0]]),
                       dict(target_bounds=[-.35,4,-1.5,1.5]),dict(center_bounds=[0,True,0,1])]:
            cfg=copy.deepcopy(settings);cfg.update(copy.deepcopy(SITE));cfg["flight_area"].update(change)
            with self.assertRaises(ValueError):validate_settings(cfg)
    def test_recording_covers_replay_contract_without_raw_camera_or_lidar(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location("replay",ROOT/"tools/bag_replay/bag_replay.py")
        replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
        topics=topics_for({})
        optional_clouds={"/sdf_map/occupancy_inflate","/sdf_map/occupancy","/freedom/static_pointcloud"}
        self.assertFalse(set(replay.TOPICS.values())-set(topics)-optional_clouds-{replay.TOPICS["camera"]})
        self.assertEqual(optional_clouds.intersection(topics),{"/sdf_map/occupancy_inflate"})
        self.assertEqual(topics[0],"/board_trials/recording/image/compressed")
        self.assertTrue(optional_clouds.issubset(topics_for({"record_map_clouds":True})))
        self.assertIn("/planning/progress",topics)
        self.assertIn("/uav_vision/release_evidence_context",topics)
        self.assertNotIn("/camera/image_raw",topics)
        self.assertNotIn("/livox/lidar",topics)
if __name__=="__main__":unittest.main()
