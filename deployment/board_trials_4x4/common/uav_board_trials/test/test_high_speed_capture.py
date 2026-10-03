from pathlib import Path
from dataclasses import replace
import copy,tempfile,unittest,yaml
from trial_config import generate,validate_settings,flight_geometry
from trial_runtime import HighSpeedCaptureRuntime
from trial_auto_land import trial_ready
from trial_result import evaluate
from uav_mission.mission_core import MissionCore
from uav_mission.high_view_probe import ProbeConfig
from test_mission_runtime import profile,result_for,candidate
from test_trials import config
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
class CaptureTests(unittest.TestCase):
    def settings(self):return yaml.safe_load((BASE/'09_high_speed_capture/settings.yaml').read_text())
    def test_existing_modules_still_reject_high_speed(self):
        for name in ('01_visual_interrupt','02_high_view_revisit','07_memory_only'):
            s=yaml.safe_load((BASE/name/'settings.yaml').read_text());s['cruise_speed']=1.
            with self.assertRaises(ValueError):validate_settings(s)
    def test_both_speed_profiles_have_single_no_release_chain_and_limits(self):
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for speed in (.5,1.):
            s=self.settings();s['cruise_speed']=speed
            with tempfile.TemporaryDirectory() as path:
                ref=generate(ROOT,path,s,(.01,-.01,0.),rig)
                rt=yaml.safe_load((Path(path)/'runtime.yaml').read_text());ctrl=yaml.safe_load((Path(path)/'control.yaml').read_text());ov=yaml.safe_load((Path(path)/'overrides.yaml').read_text())
                self.assertFalse(ctrl['drop_system']['enable_drop']);self.assertEqual(rt['trial']['actuator_mode'],'none')
                self.assertEqual(rt['following_speed_profile']['cruise_lead_m'],speed)
                self.assertGreater(ov['/external_planner_start_max_distance'],speed)
                self.assertAlmostEqual(ov['/external_planner_max_command_z']-ref['ground_z'],2.)
                points=rt['high_view_probe']['config']['survey_xy']
                self.assertEqual(points,[[5.21,-.01],[.81,-.01],[5.21,-.01],[.81,-.01]])
                self.assertEqual(ov['/fast_planner_node/sdf_map/virtual_ceil_height'],-.1)
                self.assertFalse(ov['/fast_planner_node/sdf_map/horizontal_avoidance/enabled'])
    def test_bad_route_speed_and_actuator_rejected(self):
        for patch in ({'actuator_mode':'real'},{'cruise_speed':1.1},{'high_agl':2.6},{'terminal_hover_agl':.4},{'capture_round_trips':5},{'capture_line_xy':[[.8,0],[2,0]]},{'capture_line_xy':[[.8,0],[6,0]]},{'capture_line_xy':[[.8,0],[float('nan'),0]]}):
            s=self.settings();s.update(patch)
            with self.assertRaises(ValueError):validate_settings(s)
    def runtime(self):
        s=self.settings();a=flight_geometry(s)
        r=HighSpeedCaptureRuntime(MissionCore(profile(),config()),ProbeConfig(-.22,tuple(tuple(p) for p in a['survey_xy']),high_agl=2.,staging_xy=tuple(a['staging_xy'])))
        r.start('capture',100.,(0.,0.));return r
    def test_empty_scene_finishes_route_descent_then_land_without_approach(self):
        r=self.runtime();now=100.;commands=[]
        for seq in range(16):
            action=r.core.active_action;commands.append(action.command)
            if action.command=='LAND':break
            self.assertEqual(action.command,'SEARCH');self.assertFalse(action.has_target)
            now+=2.
            r.update_pose((action.goal.x,action.goal.y,action.goal.z),now,'camera_init')
            out=r.apply_result(replace(result_for(action,seq+1,status='SUCCEEDED',terminal=True),mission_id='capture',event_stamp_ns=int(now*1e9)),now,(action.goal.x,action.goal.y))
        self.assertEqual(commands[-1],'LAND');self.assertEqual(r.core.committed_slots,0)
        self.assertTrue(r.capture_complete);self.assertEqual(r.trial_manifest,{})
        self.assertEqual(r.core.active_action.reason,'board_high_speed_capture_complete')
    def test_targets_do_not_interrupt_capture(self):
        r=self.runtime();r.ascent_verified=True;r.update_pose((.8,0,1.78),101.,'camera_init')
        r.ingest([candidate(class_name='red_cross',now=101.,x=2.,y=0)],101.)
        seq=r.core.active_action.decision_seq
        out=r.tick(101.1,(.8,0.));self.assertEqual(r.core.active_action.decision_seq,seq)
        self.assertEqual(r.stage,'SURVEY');self.assertEqual(r.core.committed_slots,0)
    def test_incomplete_route_does_not_report_capture_complete(self):
        r=self.runtime();r.ascent_verified=True
        r.route.interrupt(r.core.active_action.decision_seq);r.core.active_action=None
        out=r._retreat(101.);self.assertEqual(out.action.command,'ABORT');self.assertFalse(r.capture_complete)
    def test_hover_requires_capture_completion_and_matching_identity(self):
        status=dict(phase='LAND',active_command='LAND',mission_failed=False,mission_id='m',active_decision_seq=2)
        ctx=dict(scope='board_trial_landing_after_mock',mode='high_speed_capture',frame='camera_init',mission_id='m',decision_seq=2,expected=0,committed=0,capture_complete=True)
        self.assertTrue(trial_ready(status,ctx,'camera_init'))
        for patch in ({'capture_complete':False},{'decision_seq':1},{'committed':1}):
            self.assertFalse(trial_ready(status,{**ctx,**patch},'camera_init'))
    def test_collection_is_not_detection_or_speed_pass(self):
        sup=dict(trial='high_speed_capture',end_reason='landed_after_flight',actuator_mode='none')
        data=dict(mission=dict(mission_failed=False,committed_slots=0),high=dict(capture_complete=True),terminal_hover=dict(stage='PILOT_HANDOFF'))
        out=evaluate(sup,data,self.settings());self.assertEqual(out['status'],'CAPTURED')
        self.assertEqual(out['speed_and_recognition_validation'],'PENDING_OFFLINE')
        data['mission']['mission_failed']=True
        self.assertEqual(evaluate(sup,data,self.settings())['status'],'INCOMPLETE')
if __name__=='__main__':unittest.main()
