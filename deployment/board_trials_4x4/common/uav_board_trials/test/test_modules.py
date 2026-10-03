from dataclasses import replace
from pathlib import Path
import tempfile,unittest,yaml
from trial_runtime import MultiDeliveryRuntime,MemoryOnlyRuntime,FullCircleRuntime
from trial_config import generate,TRIAL_FOLDERS,HIGH_MODES,NO_DROP_MODES
from trial_result import evaluate
from uav_mission.mission_core import MissionCore,MissionPhase
from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint
from uav_mission.high_view_probe import ProbeConfig
from test_mission_runtime import profile,candidate,result_for,release_ack
from test_trials import config
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'deployment/board_trials_4x4'
class Modules(unittest.TestCase):
    def test_two_release_transactions_do_not_land_after_first(self):
        r=MultiDeliveryRuntime(MissionCore(profile(),config()),CoverageRoute([Waypoint(.6,0,1.18),Waypoint(3,0,1.18)],'test'),delivery_count=2)
        r.start('mission-runtime',100.,(0.,0.))
        r.ingest([candidate(class_name='red_cross',now=101.,x=1.,y=0.)],101.)
        action=r.tick(101.1,(.6,0.)).action
        self.assertEqual(action.command,'APPROACH')
        r.apply_result(replace(release_ack(action,1),event_stamp_ns=102_000_000_000),102.,(1.,0.))
        out=r.apply_result(replace(result_for(action,2,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=103_000_000_000),103.,(1.,0.))
        self.assertEqual(r.core.committed_slots,1);self.assertNotEqual(out.action.command,'LAND')
        r.ingest([candidate(target_id=2,class_name='panzer',now=104.,x=2.7,y=0.)],104.)
        action=r.tick(104.1,(1.,0.)).action
        self.assertEqual(action.command,'APPROACH')
        r.apply_result(replace(release_ack(action,3),event_stamp_ns=105_000_000_000),105.,(2.7,0.))
        out=r.apply_result(replace(result_for(action,4,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=106_000_000_000),106.,(2.7,0.))
        self.assertEqual(out.action.command,'LAND');self.assertEqual(r.core.committed_slots,2)
    def test_memory_only_ends_without_approach(self):
        r=MemoryOnlyRuntime(MissionCore(profile(),config()),ProbeConfig(-.22,((1.,0.),)))
        r.start('m',100.,(0.,0.));r.trial_manifest={'panzer':None};r.stage='DESCEND';r._current_xy=(2.,1.)
        out=r._next_target(110.)
        self.assertEqual(out.action.command,'LAND');self.assertEqual(r.core.committed_slots,0)
        self.assertEqual(out.action.reason,'board_memory_only_complete')
    def test_frozen_manifest_does_not_resurrect_missing_hints(self):
        r=FullCircleRuntime(MissionCore(profile(),config()),ProbeConfig(-.22,((1.,0.),)))
        r.start('m',100.,(0.,0.));r.trial_manifest={'panzer':object()}
        self.assertEqual(r._all_top(101.),{})
    def test_all_modules_preserve_ceiling_and_slot_offsets(self):
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for trial,folder in TRIAL_FOLDERS.items():
            settings=yaml.safe_load((BASE/folder/'settings.yaml').read_text())
            if trial in ('corridor_landing','full_mission'):
                settings.update(corridor_waypoints=[dict(x=.6,y=0),dict(x=2.,y=0)],landing_xy=[2.8,0])
            with tempfile.TemporaryDirectory() as tmp:
                generate(ROOT,tmp,settings,(0,0,0),rig)
                runtime=yaml.safe_load((Path(tmp)/'runtime.yaml').read_text());control=yaml.safe_load((Path(tmp)/'control.yaml').read_text())
                self.assertEqual(control['drop_system']['slot_offsets'],rig['slot_offsets'])
                self.assertEqual(control['drop_system']['enable_drop'],settings['mode'] not in NO_DROP_MODES)
                for v in runtime['high_view_probe']['low_stage_parameters']:
                    if v['name'].endswith('virtual_ceil_height'):self.assertEqual(v['value'],-.1)
                for stage in runtime['mission']['post_delivery_parameter_stages']:
                    self.assertEqual(stage['parameters']['/fast_planner_node/sdf_map/virtual_ceil_height'],-.1)
    def test_failures_and_stale_landing_handoff_never_pass(self):
        supervisor=dict(trial='visual_interrupt',end_reason='landed_after_flight')
        mission=dict(mission_id='new',active_decision_seq=2,committed_slots=1,mission_failed=False)
        handoff=dict(mode_sent=True,mission_id='old',decision_seq=2)
        result=evaluate(supervisor,dict(mission=mission,land_handoff=handoff),{})
        self.assertEqual(result['status'],'INCOMPLETE')
    def test_real_endpoint_is_explicit_and_never_runs_pwm_driver(self):
        import roslaunch,rospkg
        roslaunch.substitution_args._rospack=rospkg.RosPack(ros_paths=[str(ROOT/'vision_ws/src'),str(ROOT/'patrol_uav_ws-patrol_planner/src'),'/opt/ros/noetic/share'])
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        s=yaml.safe_load((BASE/'01_visual_interrupt/settings.yaml').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            generate(ROOT,tmp,s,(0,0,0),rig)
            cfg=roslaunch.config.load_config_default([(str(BASE/'common/uav_board_trials/launch/application.launch'),['enable_control_output:=true','actuator_mode:=real','raw_servo_service:=/legacy/Servo_raw','mode:=visual_interrupt','model_path:=/test.rknn',f'generated_dir:={tmp}','ground_z:=-.22','low_z:=1.18'])],11311,verbose=False)
            nodes={n.name:n for n in cfg.nodes};values={k:v.value for k,v in cfg.params.items()}
            self.assertNotIn('board_mock_servo',nodes)
            self.assertEqual(values['/guarded_servo_proxy/raw_service_name'],'/legacy/Servo_raw')
            self.assertEqual(values['/guarded_servo_proxy/service_name'],'/board_trials/Servo')
            self.assertFalse(any(n.package=='actuator_pwm' for n in cfg.nodes))
    def test_complete_zero_sequence_requires_recorded_landing_identity(self):
        sup=dict(trial='visual_interrupt',end_reason='landed_after_flight')
        mission=dict(mission_id='m',active_decision_seq=0,phase='COMPLETE',committed_slots=1,mission_failed=False)
        handoff=dict(mode_sent=True,mission_id='m',decision_seq=4)
        latest=dict(mission=mission,land_handoff=handoff)
        self.assertEqual(evaluate(sup,latest,{})['status'],'INCOMPLETE')
        latest['landing_command']=dict(mission_id='m',active_decision_seq=4,active_command='LAND')
        self.assertEqual(evaluate(sup,latest,{})['status'],'PASS')
        latest['landing_command']['active_decision_seq']=3
        self.assertEqual(evaluate(sup,latest,{})['status'],'INCOMPLETE')
if __name__=='__main__':unittest.main()
