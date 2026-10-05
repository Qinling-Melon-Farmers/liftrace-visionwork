import unittest
from dataclasses import replace
import numpy as np
import test_motion_profiles as motion
from test_trials import config
from test_mission_runtime import profile
from trial_runtime import PriorityRevisitRuntime,MemoryOnlyRuntime
from uav_mission.high_view_probe import ProbeConfig
from uav_mission.mission_core import MissionCore
from uav_mission.search_types import Waypoint
from uav_high_view.survey_policy import SurveyPolicy

class TrialResumeTests(unittest.TestCase):
    def test_generator_opt_in_only_priority_and_full_mission(self):
        fixture=motion.MotionProfiles()
        for mode in ('high_priority','full_mission'):
            s=fixture.config(mode)
            self.assertFalse(fixture.generate(s)[0]['high_view_full']['policy']['resume_survey_enabled'])
            s['resume_survey_enabled']=True
            self.assertTrue(fixture.generate(s)[0]['high_view_full']['policy']['resume_survey_enabled'])
        for mode in ('high_view','memory_only','high_speed_capture','landing','low_multi'):
            s=fixture.config(mode);s['resume_survey_enabled']=True
            with self.assertRaises(ValueError):fixture.generate(s)
    def test_fifth_group_uses_resume_before_failing_frozen_manifest(self):
        r=PriorityRevisitRuntime(MissionCore(profile(),config()),ProbeConfig(-.22,((1.,1.),(2.,1.))),
            SurveyPolicy(resume_survey_enabled=True))
        r.start('mission-runtime',100.,(0.,0.))
        r.route.interrupt(r.core.active_action.decision_seq);r.core.active_action=None
        r.ascent_verified=True;r.stage='REACQUIRE';r.pose=(0.,0.,1.18);r.pose_stamp=105.
        r.trial_manifest={'panzer':None};r.top_hints={}
        r.survey_breakpoint=(.7,1.);r.remaining_survey=(Waypoint(1.,1.,2.38),Waypoint(2.,1.,2.38))
        r.grid.update(np.array([[0.,0.,-.22]]),105.,.18,2.8)
        out=r._start_fallback(105.,'low_view_class_disproved')
        self.assertEqual(r.stage,'RESUME_ASCEND')
        self.assertEqual(out.action.command,'SEARCH')
        self.assertEqual(r.core.config.home_xy,(0.,0.))
        self.assertTrue(r.return_to_takeoff_before_land)
    def test_memory_only_ending_does_not_enter_delivery_or_resume(self):
        r=MemoryOnlyRuntime(MissionCore(profile(),config()),ProbeConfig(-.22,((1.,1.),)))
        r.start('mission-runtime',100.,(0.,0.))
        r.route.interrupt(r.core.active_action.decision_seq);r.core.active_action=None
        out=r._next_target(105.)
        self.assertEqual(out.action.command,'LAND')
        self.assertFalse(r.resume_attempted)
        self.assertEqual(r.core.committed_slots,0)
if __name__=='__main__':unittest.main()
