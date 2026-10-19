"""Task 2: single-target interrupts, counted releases, measured corridor/H tail."""
from dataclasses import replace
from pathlib import Path
import copy,tempfile,unittest,yaml
from trial_runtime import MultiDeliveryCorridorRuntime
from trial_config import generate,apply_site_profile,validate_settings
from trial_speed_profiles import apply_speed_profile
from uav_mission.mission_core import MissionCore,MissionPhase,GoalSnapshot
from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint
from test_mission_runtime import profile,candidate,result_for,release_ack
from test_trials import config
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
class LowMultiCorridor(unittest.TestCase):
    def test_one_valid_target_interrupts_then_each_release_resumes_until_count(self):
        for count in (1,2,3):
            with self.subTest(count=count):
                cfg=replace(config(),landing_xy=(3.,1.),post_delivery_route=(
                    GoalSnapshot('camera_init',2.,1.,.68),GoalSnapshot('camera_init',3.,1.,.68),
                    GoalSnapshot('camera_init',3.,1.,.98)))
                runtime=MultiDeliveryCorridorRuntime(MissionCore(profile(),cfg),
                    CoverageRoute([Waypoint(.6,0,1.18),Waypoint(3,0,1.18)],'low'),delivery_count=count)
                runtime.start('mission-runtime',100.,(0.,0.))
                seq=0
                for index,cls in enumerate(('red_cross','panzer','bridge')[:count]):
                    now=101.+index*4
                    runtime.ingest([candidate(target_id=index+1,class_name=cls,now=now,x=1.+index*.4,y=0.)],now)
                    action=runtime.tick(now+.1,(.6,0.)).action
                    self.assertEqual(action.command,'APPROACH')
                    self.assertEqual(runtime.core.queue.delivered_classes,set(('red_cross','panzer','bridge')[:index]))
                    seq+=1
                    runtime.apply_result(replace(release_ack(action,seq),event_stamp_ns=int((now+1)*1e9)),now+1,(1.+index*.4,0.))
                    # Committing alone must not skip the recovery stage.
                    self.assertNotEqual(runtime.core.phase,MissionPhase.POST_DELIVERY_ROUTE)
                    seq+=1
                    out=runtime.apply_result(replace(result_for(action,seq,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=int((now+2)*1e9)),now+2,(1.+index*.4,0.))
                    if index<count-1:
                        self.assertEqual(out.action.command,'RESUME')
                        self.assertEqual(runtime.route.current_index,0)
                self.assertEqual(runtime.core.phase,MissionPhase.POST_DELIVERY_ROUTE)
                self.assertEqual(out.action.goal,cfg.post_delivery_route[0])
                self.assertEqual(runtime.core.config.landing_xy,(3.,1.))
                # Complete every corridor leg and H capture leg, then LAND at H.
                now+=3
                for goal in cfg.post_delivery_route:
                    self.assertEqual(out.action.goal,goal);seq+=1
                    event=replace(result_for(out.action,seq,status='SUCCEEDED',stage='PLANNER',terminal=True),event_stamp_ns=int(now*1e9))
                    out=runtime.apply_result(event,now,(goal.x,goal.y));now+=1
                self.assertEqual(out.action.command,'LAND')
                self.assertEqual(runtime.core.committed_slots,count)
    def test_unconfirmed_target_does_not_interrupt(self):
        runtime=MultiDeliveryCorridorRuntime(MissionCore(profile(),config()),CoverageRoute([Waypoint(.6,0,1.18),Waypoint(3,0,1.18)],'low'))
        runtime.start('mission-runtime',100.,(0.,0.))
        runtime.ingest([replace(candidate(class_name='red_cross',now=101.),state=0)],101.)
        self.assertIsNone(runtime.tick(101.1,(0.,0.)).action)
        self.assertEqual(runtime.core.active_action.command,'SEARCH')
    def test_generated_map_and_speed_keep_release_and_low_height(self):
        settings=yaml.safe_load((BASE/'05_low_multi/settings.yaml').read_text(encoding='utf-8'))
        site=yaml.safe_load((BASE/'05_low_multi/site.yaml').read_text(encoding='utf-8'))
        site['search_waypoints_xy']=[[.6,0.],[1.,1.],[2.,-1.]]
        apply_site_profile(settings,site)
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text(encoding='utf-8'))
        for speed in ('limited','competition'):
            with self.subTest(speed=speed),tempfile.TemporaryDirectory() as out:
                active=copy.deepcopy(settings);apply_speed_profile(ROOT,active,'low_multi',speed)
                validate_settings(active);generate(ROOT,out,active,(0.,0.,0.),rig)
                runtime=yaml.safe_load((Path(out)/'runtime.yaml').read_text(encoding='utf-8'))
                control=yaml.safe_load((Path(out)/'control.yaml').read_text(encoding='utf-8'))
                ground=-rig['fc_ground_clearance']
                self.assertTrue(runtime['trial']['corridor_after_delivery'])
                self.assertEqual([v[:2] for v in runtime['trial']['waypoints']],site['search_waypoints_xy'])
                for v in runtime['trial']['waypoints']:self.assertAlmostEqual(v[2]-ground,1.4,places=6)
                self.assertEqual(active['drop_agl'],settings['drop_agl'])
                self.assertNotIn('resume_survey_enabled',active)
                self.assertEqual(active['cruise_speed'],.5 if speed=='limited' else 1.2)
                self.assertEqual(runtime['mission']['landing_xy'],site['landing_xy'])
                self.assertTrue(control['switch']['auto_land'])
                self.assertEqual(control['external_landing']['handoff_mode'],'POSCTL')
                self.assertEqual(runtime['corridor_speed_schedule']['wall_coordinates'],site['corridor_geometry']['wall_coordinates'])
                self.assertEqual(runtime['mission']['post_delivery_route'][-1][:2],site['landing_xy'])
    def test_empty_corridor_rejected_and_low_xy_not_allowed_for_other_group(self):
        settings=yaml.safe_load((BASE/'05_low_multi/settings.yaml').read_text(encoding='utf-8'))
        with self.assertRaises(ValueError):validate_settings(settings)
        other=yaml.safe_load((BASE/'01_visual_interrupt/settings.yaml').read_text(encoding='utf-8'))
        with self.assertRaises(ValueError):apply_site_profile(other,{'search_waypoints_xy':[[.6,0.],[1.,1.]]})
if __name__=='__main__':unittest.main()
