import unittest
from dataclasses import replace
from trial_runtime import PriorityRevisitRuntime
from test_trials import config
from test_mission_runtime import profile,result_for
from uav_mission.mission_core import MissionCore
from uav_mission.high_view_probe import ProbeConfig

class PriorityReturnTests(unittest.TestCase):
    def test_finish_returns_to_home_not_staging_then_lands(self):
        r=PriorityRevisitRuntime(MissionCore(profile(),config()),
            ProbeConfig(-.22,((1.,1.),),staging_xy=(.6,0.)))
        r.start('mission-runtime',100.,(0.,0.))
        r.route.interrupt(r.core.active_action.decision_seq)
        r.core.active_action=None
        r._current_xy=(3.,1.)
        out=r.end_here(101.,'test_complete')
        self.assertEqual(out.action.command,'RETURN_HOME')
        self.assertEqual((out.action.goal.x,out.action.goal.y),(0.,0.))
        self.assertEqual(r.core.config.landing_xy,(0.,0.))
        event=replace(result_for(out.action,1,status='SUCCEEDED',stage='PLANNER',terminal=True),
                      event_stamp_ns=102000000000)
        out=r.apply_result(event,102.,(0.,0.))
        self.assertEqual(out.action.command,'LAND')
