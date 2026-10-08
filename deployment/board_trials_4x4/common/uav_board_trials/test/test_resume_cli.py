"""Exercise board CLI resume overrides before any ROS import or child launch."""
import copy
import io
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
SCRIPTS=BASE/'common/uav_board_trials/scripts'
sys.path[:0]=[str(SCRIPTS),str(ROOT/'vision_ws/src/uav_high_view/src'),
              str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src')]
import run_trial
from trial_config import generate,TRIAL_FOLDERS


class ResumeCLI(unittest.TestCase):
    def settings(self,trial):
        value=yaml.safe_load((BASE/TRIAL_FOLDERS[trial]/'settings.yaml').read_text())
        if trial in ('full_mission','corridor_landing'):
            value.update(corridor_waypoints=[dict(x=x,y=0.,agl=.9) for x in (.6,1.2,1.8)],landing_xy=[2.8,0.])
        return value

    def invoke(self,trial,settings,option=None):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);base=root/'deployment/board_trials_4x4'
            config=base/'common/uav_board_trials/config';config.mkdir(parents=True)
            for name in ('known_rig.yaml','mapping_startup.yaml'):
                shutil.copyfile(BASE/'common/uav_board_trials/config'/name,config/name)
            folder=base/TRIAL_FOLDERS[trial];folder.mkdir()
            settings_path=folder/'settings.yaml'
            settings_path.write_text(yaml.safe_dump(settings))
            original=settings_path.read_text()
            args=['run_trial.py',trial,'preview','--root',str(root),'--check-config']
            if option is not None:args+=['--resume-survey',option]
            seen=[]
            validate=run_trial.validate_settings
            def inspect(value):
                seen.append(copy.deepcopy(value));validate(value)
            stdout=io.StringIO();stderr=io.StringIO();code=0
            with patch.object(sys,'argv',args),patch.object(run_trial,'validate_settings',side_effect=inspect),redirect_stdout(stdout),redirect_stderr(stderr):
                try:run_trial.main()
                except SystemExit as error:code=error.code
            self.assertEqual(settings_path.read_text(),original)
            return code,stdout.getvalue(),stderr.getvalue(),seen

    def test_on_off_reach_existing_runtime_policy_for_both_groups(self):
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for trial in ('high_priority','full_mission'):
            for option in ('on','off'):
                with self.subTest(trial=trial,option=option):
                    settings=self.settings(trial);settings['resume_survey_enabled']=option!='on'
                    code,stdout,stderr,seen=self.invoke(trial,settings,option)
                    self.assertEqual(code,0,stderr)
                    self.assertIn('CONFIG_VALID; no ROS nodes started',stdout)
                    self.assertIs(seen[0]['resume_survey_enabled'],option=='on')
                    with tempfile.TemporaryDirectory() as directory:
                        generate(ROOT,directory,seen[0],(0.,0.,0.),rig)
                        runtime=yaml.safe_load((Path(directory)/'runtime.yaml').read_text())
                        self.assertIs(runtime['high_view_full']['policy']['resume_survey_enabled'],option=='on')

    def test_omission_preserves_inherited_value_and_defaults(self):
        for inherited in (None,False,True):
            with self.subTest(inherited=inherited):
                settings=self.settings('high_priority')
                if inherited is not None:settings['resume_survey_enabled']=inherited
                code,stdout,stderr,seen=self.invoke('high_priority',settings)
                self.assertEqual(code,0,stderr)
                if inherited is None:self.assertIs(seen[0]['resume_survey_enabled'],True)
                else:self.assertIs(seen[0]['resume_survey_enabled'],inherited)

    def test_all_other_groups_reject_explicit_on_and_off_in_check_config(self):
        for trial in set(TRIAL_FOLDERS)-{'high_priority','full_mission'}:
            for option in ('on','off'):
                with self.subTest(trial=trial,option=option):
                    code,stdout,stderr,seen=self.invoke(trial,self.settings(trial),option)
                    self.assertEqual(code,2)
                    self.assertIn('--resume-survey is only valid',stderr)
                    self.assertNotIn('CONFIG_VALID',stdout)

    def test_invalid_choice_and_inherited_settings_fail_before_ready(self):
        cases=[('high_priority',{},'true','invalid choice'),
               ('high_priority',{'resume_survey_enabled':'on'},None,'must be boolean'),
               ('memory_only',{'resume_survey_enabled':True},None,'only available')]
        for trial,extra,option,message in cases:
            with self.subTest(trial=trial,extra=extra,option=option):
                settings=self.settings(trial);settings.update(extra)
                code,stdout,stderr,seen=self.invoke(trial,settings,option)
                self.assertEqual(code,2)
                self.assertIn(message,stderr)
                self.assertNotIn('CONFIG_VALID',stdout)


if __name__=='__main__':unittest.main(verbosity=2)
