"""离线检查比赛参数保护、CLI覆盖、预算和硬件session；不启动ROS。"""
import contextlib,copy,importlib.util,io,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import yaml
from uav_mission.competition_config import validate,generate,apply_overrides

ROOT=Path(__file__).resolve().parents[4]
PKG=ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission'
REPORT=ROOT/'docs/verification/competition_release_20261008'

class ReleaseEntryTests(unittest.TestCase):
    def setUp(self):
        self.s=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text())
        self.rig=yaml.safe_load((PKG/'config/competition/known_rig.yaml').read_text())

    def generate(self,s):
        with tempfile.TemporaryDirectory() as d:
            generate(ROOT,d,s,(0.,0.,0.),self.rig)
            return {n:yaml.safe_load((Path(d)/(n+'.yaml')).read_text()) for n in ('runtime','control','overrides')}

    def fixture(self):
        s=copy.deepcopy(self.s)
        s.update(site_confirmed=True,corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),dict(x=6.7,y=4.,agl=.9),dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],landing_xy=[8.5,-4.2],
            corridor_speed_schedule=dict(axis=1,wall_coordinates=[-1.6,1.6],entry_waypoints=1))
        return s

    def test_hardware_runtime_uses_fixed_session_and_full_mission_budget(self):
        from test_mission_runtime import profile
        from uav_mission.mission_core import MissionCore,MissionConfig
        from uav_mission.high_view_probe import ProbeConfig
        spec=importlib.util.spec_from_file_location('hardware_runtime_checked',PKG/'scripts/navigation_competition_manager.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        runtime=module.HardwareFullRuntime(MissionCore(profile(),MissionConfig(early_return_enabled=False)),ProbeConfig(-.22,((1.,1.),(2.,1.))))
        outcome=runtime.start('hardware-mission',100.,(0.,0.))
        self.assertTrue(outcome.accepted)
        self.assertEqual(runtime.catalog.epoch.localization,'fixed-board-session')
        self.assertEqual(runtime.core.started_at,100.)
        self.assertEqual(runtime.survey_until,100.+runtime.core.config.mission_timeout)
        self.assertEqual(runtime.core.committed_slots,0)

    def test_existing_competition_values_preserved(self):
        original=json.loads((REPORT/'original_competition_parameters.json').read_text())
        for name,data in original.items():
            current=yaml.safe_load((ROOT/name).read_text())
            if 'field_20261007_validated' not in name:
                data=copy.deepcopy(data)
                data.setdefault('motion_optimization',{})['enabled']=True
                data['survey_policy']['resume_survey_enabled']=True
                # 仅推广本轮授权高度/前视与显式恢复关闭，其余原检测值仍逐项比较。
                data.update(drop_agl=.35,max_agl=3.2,navigation_recovery={'enabled':False})
                data['following_speed_profile']['corridor_lead_m']=.4
                schedule=data.get('corridor_speed_schedule') or dict(
                    axis=1,wall_coordinates=[-1.6,1.6],enter_distance_m=.75,
                    exit_distance_m=.95,landing_radius_m=.8)
                data['corridor_speed_schedule']={**schedule,'open_lead_m':.6,'door_lead_m':.4}
            # Snapshot predates the separately tested POSCTL settlement block.
            if 'landing_posctl' not in data:current.pop('landing_posctl',None)
            self.assertEqual(current,data,name)

    def test_no_cli_switch_changes_yaml(self):
        self.assertEqual(apply_overrides(self.s),self.s)

    def test_explicit_switches_do_not_mutate_source(self):
        s=self.fixture();before=copy.deepcopy(s)
        enabled=apply_overrides(s,motion='on',columns='off')
        self.assertEqual(s,before)
        d=self.generate(enabled)
        self.assertTrue(d['runtime']['motion_optimization']['enabled'])
        self.assertFalse(d['overrides']['/fast_planner_node/sdf_map/horizontal_avoidance/enabled'])
        disabled=apply_overrides(enabled,motion='off',columns='on')
        d=self.generate(disabled)
        self.assertFalse(d['runtime']['motion_optimization']['enabled'])
        self.assertFalse(d['overrides']['/navigation/planner_bridge/motion_optimization']['enabled'])
        self.assertTrue(d['overrides']['/fast_planner_node/sdf_map/horizontal_avoidance/enabled'])
        self.assertEqual(d['control']['uav_vision']['recovery_height'],d['control']['align_height'])

    def test_default_motion_and_resume_are_independent_and_generated(self):
        self.assertIs(self.s['motion_optimization']['enabled'],True)
        self.assertIs(self.s['survey_policy']['resume_survey_enabled'],True)
        for motion in ('on','off'):
            for resume in ('on','off'):
                with self.subTest(motion=motion,resume=resume):
                    s=apply_overrides(self.fixture(),motion=motion,resume=resume)
                    docs=self.generate(s)
                    self.assertIs(docs['runtime']['motion_optimization']['enabled'],motion=='on')
                    self.assertIs(docs['overrides']['/navigation/planner_bridge/motion_optimization']['enabled'],motion=='on')
                    self.assertIs(docs['runtime']['high_view_full']['policy']['resume_survey_enabled'],resume=='on')
        historical=yaml.safe_load((ROOT/'deployment/competition/field_20261007_validated.yaml').read_text())
        self.assertIs(historical['survey_policy']['resume_survey_enabled'],False)
        # An inherited historic off remains off until explicitly overridden.
        self.assertIs(self.generate(historical)['runtime']['high_view_full']['policy']['resume_survey_enabled'],False)

    def test_default_budget_and_explicit_timeout(self):
        s=self.fixture();d=self.generate(s)
        self.assertEqual(d['runtime']['mission']['motion_action_timeout'],90.)
        self.assertEqual(d['runtime']['mission']['target_action_timeout'],120.)
        d=self.generate(apply_overrides(s,motion_timeout=60.))
        self.assertEqual(d['runtime']['mission']['motion_action_timeout'],60.)
        self.assertEqual(d['runtime']['mission']['target_action_timeout'],120.)

    def test_bad_budgets_and_non_boolean_resume_refused(self):
        for value in (True,0.,-1.,601.,float('nan'),float('inf'),'60'):
            for key in ('motion_action_timeout','target_action_timeout'):
                with self.subTest(value=value,key=key),self.assertRaises(ValueError):
                    validate(dict(self.s,**{key:value}))
        s=copy.deepcopy(self.s);s['survey_policy']['resume_survey_enabled']=True
        validate(s)
        for invalid in ('true',1,None):
            s['survey_policy']['resume_survey_enabled']=invalid
            with self.assertRaisesRegex(ValueError,'must be boolean'):validate(s)

    def test_validated_test_is_explicit_and_retains_updated_door_points(self):
        s=yaml.safe_load((ROOT/'deployment/competition/field_20261007_validated.yaml').read_text())
        validate(s,flight=True)
        self.assertEqual([p['y'] for p in s['corridor_waypoints'][2:6]],[1.,-.7,-1.1,-2.5])
        self.assertEqual((s['high_agl'],s['drop_agl'],s['landing_capture_agl'],s['cruise_speed'],s['cruise_acceleration']),(2.,.35,1.2,.5,.35))
        self.assertEqual(self.generate(s)['runtime']['mission']['motion_action_timeout'],60.)

    def test_check_and_generation_never_calls_session(self):
        spec=importlib.util.spec_from_file_location('checked_supervisor',PKG/'scripts/competition_supervisor.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        with tempfile.TemporaryDirectory() as d:
            argv=['competition_supervisor.py','preview','--root',str(ROOT),'--site-config',str(ROOT/'deployment/competition/field_20261007_validated.yaml'),'--check-config','--output-dir',d,'--fc-reference','0','0','0','--motion-optimization','off','--obstacle-columns','off']
            with patch('sys.argv',argv),patch.object(cli,'run_session') as session,contextlib.redirect_stdout(io.StringIO()):
                cli.main()
            session.assert_not_called()
            self.assertFalse(yaml.safe_load((Path(d)/'overrides.yaml').read_text())['/navigation/planner_bridge/motion_optimization']['enabled'])

    def test_unconfirmed_template_flight_is_refused_before_session(self):
        spec=importlib.util.spec_from_file_location('checked_supervisor2',PKG/'scripts/competition_supervisor.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        argv=['competition_supervisor.py','flight','--root',str(ROOT),'--site-config',str(ROOT/'deployment/competition/field.example.yaml'),'--check-config']
        with patch('sys.argv',argv),patch.object(cli,'run_session') as session,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            cli.main()
        session.assert_not_called()

if __name__=='__main__':unittest.main()
