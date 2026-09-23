import unittest
from test_mission_core import competition_profile,candidate
from uav_mission.mission_core import MissionCore,MissionConfig,GoalSnapshot

class EarlyReturnDisabledTest(unittest.TestCase):
    def core(self):
        core=MissionCore(competition_profile(),MissionConfig(early_return_enabled=False,forced_return_at=420,return_land_reserve=180))
        core.start('no-early-return',100.)
        return core
    def test_search_can_continue_past_420_but_not_600_elapsed(self):
        core=self.core();goal=GoalSnapshot('camera_init',1,2,1.18)
        self.assertIsNone(core.choose(650,(0,0)))
        action=core.dispatch_search_motion('SEARCH',goal,'coverage',650)
        self.assertEqual(action.deadline_at,700.)
        with self.assertRaises(RuntimeError):self.core().dispatch_search_motion('SEARCH',goal,'coverage',700.)
    def test_late_target_has_remaining_total_time_without_return_reservation(self):
        core=self.core();core.ingest([candidate(now=650)],650)
        self.assertFalse(core.should_stop_search(650,(0,0)))
        action=core.choose(650,(0,0))
        self.assertEqual(action.command,'APPROACH')
        self.assertEqual(action.deadline_at,700.)
    def test_hardware_default_and_total_limit_are_preserved(self):
        self.assertTrue(MissionConfig().early_return_enabled)
        with self.assertRaises(ValueError):MissionConfig(early_return_enabled=False,mission_timeout=601)
