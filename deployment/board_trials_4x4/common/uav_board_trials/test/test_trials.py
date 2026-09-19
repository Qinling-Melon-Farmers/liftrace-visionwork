from pathlib import Path
from dataclasses import replace
import ast,json,tempfile,unittest
import numpy as np,yaml
from trial_config import generate
from trial_runtime import SingleDeliveryRuntime,FullCircleRuntime,OpenTourGrid
from uav_mission.mission_core import MissionCore,MissionConfig,GoalSnapshot,MissionPhase,SlotStatus
from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint
from uav_mission.high_view_probe import ProbeConfig
from test_mission_runtime import profile,candidate,result_for,release_ack

ROOT=Path(__file__).resolve().parents[5]

def config():return MissionConfig(mission_timeout=300.,forced_return_at=240.,return_land_reserve=45.,early_return_enabled=False,home_xy=(0.,0.),post_delivery_route=(GoalSnapshot('camera_init',.6,0.,1.18),),landing_xy=(.6,0.),return_altitude=1.18)

class TrialTests(unittest.TestCase):
    def test_single_release_returns_here_after_recovery(self):
        r=SingleDeliveryRuntime(MissionCore(profile(),config()),CoverageRoute([Waypoint(.6,0,1.18),Waypoint(3,0,1.18)],'test'))
        r.start('mission-runtime',100.,(0.,0.));r.ingest([candidate(class_name='panzer',now=101.,x=2.,y=0.)],101.)
        out=r.tick(101.1,(1.,0.));action=out.action;self.assertEqual(action.command,'APPROACH')
        r.apply_result(replace(release_ack(action,1),event_stamp_ns=102_000_000_000),102.,(2.,0.))
        self.assertEqual(r.core.phase,MissionPhase.EXECUTING)
        out=r.apply_result(replace(result_for(action,2,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=103_000_000_000),103.,(2.02,.04))
        self.assertEqual(out.action.command,'LAND');self.assertEqual(tuple(r.core.config.landing_xy),(2.02,.04));self.assertEqual(r.core.committed_slots,1)
    def test_full_circle_does_not_interrupt_when_top_three_ready(self):
        r=FullCircleRuntime(MissionCore(profile(),config()),ProbeConfig(-.22,((1.,-1.),(3.,-1.),(3.,1.),(1.,1.),(1.,-1.))))
        r.start('mission-runtime',100.,(0.,0.));r.ascent_verified=True
        for t in (101.,101.3,101.6):
            r.update_pose((0,0,2.38),t,'camera_init')
            r.ingest([replace(candidate(target_id=i,class_name=c,now=t,x=1.+i*.5,y=.5),first_seen_ns=99_000_000_000) for i,c in enumerate(('bridge','panzer','red_cross'))],t)
        self.assertEqual(len(r._all_top(101.6)),3)
        seq=r.core.active_action.decision_seq;r.tick(101.7,(0.,0.));self.assertEqual(r.core.active_action.decision_seq,seq);self.assertEqual(r.stage,'SURVEY')
    def test_one_two_three_memories_finish_at_last_target_without_fake_slots(self):
        for n in (1,2,3):
            r=FullCircleRuntime(MissionCore(profile(),config()),ProbeConfig(-.22,((1.,0.),)))
            r.start('mission-runtime',100.,(0.,0.));r.route.interrupt(r.core.active_action.decision_seq);r.core.active_action=None;r.core.phase=MissionPhase.SEARCH;r.stage='DELIVERY';r._current_xy=(2.3,.4)
            names=('bridge','panzer','red_cross')[:n];r.trial_manifest={k:None for k in names};r.core.queue.delivered_classes=set(names)
            for slot in r.core.slots[:n]:slot.status=SlotStatus.COMMITTED
            out=r._start_fallback(101.,'known_hints_exhausted');self.assertEqual(out.action.command,'LAND');self.assertEqual(tuple(r.core.config.landing_xy),(2.3,.4));self.assertEqual(r.core.committed_slots,n);self.assertEqual(r.stage,'TAIL')
    def test_open_tour_does_not_add_return_edge(self):
        grid=OpenTourGrid(bounds=(-.5,4.,-2.,2.),resolution=.1,inflation=0.);grid.stamp=100.
        cost,names=grid.order((0.,0.),{'near':(1.,0.),'far':(3.,0.)},(0.,0.),100.)
        self.assertEqual(names,('near','far'));self.assertAlmostEqual(cost,3.,places=5)
    def test_all_local_heights_derive_from_same_measured_reference(self):
        base=ROOT/'deployment/board_trials_4x4';rig=yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for folder in ('01_visual_interrupt','02_high_view_revisit','03_h_landing'):
            with tempfile.TemporaryDirectory() as path:
                s=yaml.safe_load((base/folder/'settings.yaml').read_text());ref=generate(ROOT,path,s,(.01,-.01,-.05),rig)
                self.assertAlmostEqual(ref['ground_z'],-.27);self.assertAlmostEqual(ref['low_z']-ref['ground_z'],1.4)
                control=yaml.safe_load((Path(path)/'control.yaml').read_text());runtime=yaml.safe_load((Path(path)/'runtime.yaml').read_text())
                self.assertAlmostEqual(control['drop_system']['release_setpoint_height']-ref['ground_z'],.6)
                if folder=='03_h_landing':
                    route=runtime['mission']['post_delivery_route'];self.assertEqual(route[-1][:2],route[-2][:2]);self.assertGreater(route[-1][2],route[-2][2]);self.assertFalse(control['drop_system']['enable_drop'])
                else:self.assertTrue(control['drop_system']['enable_drop'])
    def test_python38_syntax(self):
        for p in (ROOT/'deployment/board_trials_4x4/common/uav_board_trials/scripts').glob('*.py'):ast.parse(p.read_text(),feature_version=8)
    def test_auto_land_requires_matching_successful_trial_context(self):
        from trial_auto_land import trial_ready
        status=dict(phase='LAND',active_command='LAND',mission_failed=False,mission_id='m',active_decision_seq=8)
        context=dict(scope='board_trial_landing_after_mock',frame='camera_init',mode='visual_interrupt',expected=1,committed=1,mission_id='m',decision_seq=8)
        self.assertTrue(trial_ready(status,context,'camera_init'))
        for change in (dict(committed=0),dict(mission_id='old'),dict(decision_seq=7),dict(frame='map'),dict(expected=0)):
            self.assertFalse(trial_ready(status,{**context,**change},'camera_init'))
    def test_no_target_cannot_report_one_delivery(self):
        r=SingleDeliveryRuntime(MissionCore(profile(),config()),CoverageRoute([Waypoint(.6,0,1.18)],'test'))
        r.start('mission-runtime',100.,(0.,0.));action=r.core.active_action
        out=r.apply_result(replace(result_for(action,1,status='SUCCEEDED',terminal=True),event_stamp_ns=101_000_000_000),101.,(.6,0.))
        self.assertEqual(r.core.committed_slots,0);self.assertEqual(out.action.command,'ABORT')

if __name__=='__main__':unittest.main()
