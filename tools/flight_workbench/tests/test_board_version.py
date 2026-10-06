"""Mock preflight checks; fallback fixtures do not prove this checkout can fly."""
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from contextlib import redirect_stdout
from unittest.mock import Mock,patch

TOOL=Path(__file__).resolve().parents[1]
ROOT=TOOL.parents[1]
sys.path.insert(0,str(TOOL))
import wb_board


# Tool-only/EV repos may lack production message sources. These test-only latest
# contracts populate the TEMPORARY mock board, never the repository or hardware.
# Existing repository schemas remain the preferred integration fixture source.
PERMISSION_FIXTURE = '''Header header
bool permitted
uint8 payload_slot
string align_mode
uint32 target_id
string target_class
time evidence_stamp
time valid_until
string reason
string mission_id
uint32 decision_seq
uint16 attempt
time target_first_seen
string permission_epoch
uint64 permission_revision
'''
INTERFACE_FIXTURES = {
    'ReleaseAuthorization': PERMISSION_FIXTURE,
    'ReleasePermission': PERMISSION_FIXTURE,
    'ServoAction': '''uint64 request_id
uint8 payload_slot
string mission_id
uint32 decision_seq
uint16 attempt
uint32 target_id
time target_first_seen
string target_class
string align_mode
string permission_epoch
uint64 permission_revision
---
uint8 EXECUTION_UNKNOWN=0
uint8 NOT_STARTED=1
uint8 RAW_CALL_STARTED=2
uint8 COMPLETED=3
uint64 request_id
uint8 payload_slot
bool res
uint8 execution_state
bool terminal
string reason
''',
}


def interface_source(source_root, relative, name):
    path=source_root/relative
    return path.read_text(encoding='utf-8') if path.is_file() else INTERFACE_FIXTURES[name]


class BoardVersion(unittest.TestCase):
    def setUp(self):
        self.config=copy.deepcopy(wb_board.load_config())
        self.config['connection']['site_dir']='deployment/custom_site'
        self.config['connection']['env_script']='deployment/custom_site/environment.sh'
        self.config['connection']['board_python']='/configured/python3'
        self.board=wb_board.BoardClient(self.config,None)

    def payload(self):
        with patch.object(self.board,'run',return_value=(0,'')) as run:
            self.board.preflight()
        command=run.call_args.args[0]
        self.assertIn('deployment/custom_site/test_area.yaml',command)
        self.assertIn('source ',command)
        self.assertIn('/configured/python3',command)
        return command.split("<<'WBVERSION'\n",1)[1].split('\nWBVERSION',1)[0]

    def execute_payload(self,change=None,source_root=ROOT):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            entry=root/'deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py'
            entry.parent.mkdir(parents=True)
            entry.write_text("parser.add_argument('--resume-survey')\n")
            modules={'patrol_control.msg':SimpleNamespace(),'uav_mission.msg':SimpleNamespace(),
                     'patrol_control.srv':SimpleNamespace()}
            def generated(text):
                names=[];kinds=[]
                for line in text.splitlines():
                    tokens=line.partition('#')[0].split()
                    if len(tokens)==2 and '=' not in tokens[1]:
                        kinds.append('std_msgs/Header' if tokens[0]=='Header' else tokens[0]);names.append(tokens[1])
                return SimpleNamespace(__slots__=names,_slot_types=kinds)
            for package,name,kind in (('patrol_control','ReleaseAuthorization','msg'),
                                      ('uav_mission','ReleasePermission','msg'),
                                      ('patrol_control','ServoAction','srv')):
                relative=Path('patrol_uav_ws-patrol_planner/src')/package/kind/(name+'.'+kind)
                text=interface_source(source_root,relative,name)
                path=root/relative;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text,encoding='utf-8')
                if kind=='srv':
                    request,response=text.split('---')
                    value=SimpleNamespace(_request_class=generated(request),_response_class=generated(response))
                else:value=generated(text)
                value._type=package+'/'+name
                setattr(modules[package+'.'+kind],name,value)
            if change:change(root,modules)
            stdout=io.StringIO()
            with patch.dict(sys.modules,modules),patch.object(sys,'argv',['-',str(root)]), \
                    patch('subprocess.check_output',side_effect=['abc1234\n',' M source.py\n']) as git,redirect_stdout(stdout):
                exec(compile(self.payload(),'<board-version>','exec'),{})
            self.assertTrue(all(call.args[0][0]=='git' for call in git.call_args_list))
            self.assertEqual([c.args[0][-2:] for c in git.call_args_list],[['--short','HEAD'],['status','--porcelain']])
            return json.loads(stdout.getvalue().partition('VERSION|')[2])

    def preflight(self,report=None,missing_settings=False,no_output=False):
        rows=['SITECFG=OK','DISK|4194304']
        rows+=['PROC|%s|0'%name for name in self.config['checks']['process_names']]
        rows+=['GROUP|%s|start=1|real=1|settings=%s'%(g['folder'],'0' if missing_settings else '1') for g in self.config['groups']]
        if report is not None:rows.append('VERSION|'+json.dumps(report))
        with patch.object(self.board,'run',return_value=(1 if no_output else 0,'' if no_output else '\n'.join(rows))):
            return self.board.preflight()

    def test_latest_source_and_generated_types_match_without_running_claim(self):
        report=self.execute_payload()
        self.assertEqual(report['source'],{'head':'abc1234','dirty':True})
        self.assertTrue(report['resume_cli'])
        self.assertTrue(all(row['ok'] for row in report['interfaces'].values()))
        result=self.preflight(report)
        revision=next(c for c in result['checks'] if c['name'].startswith('源码版本'))
        self.assertIn('非运行版本',revision['name'])
        self.assertIn('未核实运行程序版本',revision['detail'])

    def test_missing_repository_schemas_use_only_temporary_mock_fixtures(self):
        with tempfile.TemporaryDirectory() as directory:
            source_root=Path(directory)
            report=self.execute_payload(source_root=source_root)
            self.assertTrue(all(row['ok'] for row in report['interfaces'].values()))
            self.assertEqual(list(source_root.iterdir()),[])

    def test_existing_source_definition_takes_precedence_over_fallback(self):
        relative=Path('patrol_uav_ws-patrol_planner/src/patrol_control/msg/ReleaseAuthorization.msg')
        with tempfile.TemporaryDirectory() as directory:
            source_root=Path(directory);path=source_root/relative
            path.parent.mkdir(parents=True)
            actual=PERMISSION_FIXTURE+'uint8 integration_fixture_marker\n'
            path.write_text(actual)
            self.assertEqual(interface_source(source_root,relative,'ReleaseAuthorization'),actual)

    def test_stale_generated_slots_and_types_fail_specific_interface(self):
        for name,module in (('ReleaseAuthorization','patrol_control.msg'),('ReleasePermission','uav_mission.msg'),('ServoAction','patrol_control.srv')):
            for mutation in ('slots','types'):
                def change(root,modules):
                    value=getattr(modules[module],name)
                    if name=='ServoAction':value=value._request_class
                    if mutation=='slots':value.__slots__=value.__slots__[:-1];value._slot_types=value._slot_types[:-1]
                    else:value._slot_types[-1]='uint32'
                with self.subTest(name=name,mutation=mutation):
                    report=self.execute_payload(change)
                    self.assertFalse(report['interfaces'][name]['ok'])
                    check=next(c for c in self.preflight(report)['checks'] if c['name']==name+' 生成接口')
                    self.assertFalse(check['ok'])
                    self.assertIn('不能只同步 Python',check['detail'])

    def test_source_and_import_unavailable_are_not_pass(self):
        def missing_source(root,modules):
            (root/'patrol_uav_ws-patrol_planner/src/patrol_control/msg/ReleaseAuthorization.msg').unlink()
            delattr(modules['uav_mission.msg'],'ReleasePermission')
        report=self.execute_payload(missing_source)
        self.assertFalse(report['interfaces']['ReleaseAuthorization']['ok'])
        self.assertFalse(report['interfaces']['ReleasePermission']['ok'])
        self.assertTrue(report['interfaces']['ServoAction']['ok'])

    def test_both_old_source_and_old_generated_cannot_pass_latest_contract(self):
        def old(root,modules):
            path=root/'patrol_uav_ws-patrol_planner/src/patrol_control/msg/ReleaseAuthorization.msg'
            path.write_text(path.read_text(encoding='utf-8').replace('uint64 permission_revision',''))
            value=modules['patrol_control.msg'].ReleaseAuthorization
            index=value.__slots__.index('permission_revision')
            del value.__slots__[index];del value._slot_types[index]
        self.assertFalse(self.execute_payload(old)['interfaces']['ReleaseAuthorization']['ok'])

    def test_missing_cli_and_generated_report_fail(self):
        def old_entry(root,modules):
            (root/'deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py').write_text('# old source\n')
        report=self.execute_payload(old_entry)
        self.assertFalse(report['resume_cli'])
        checks=self.preflight()['checks']
        self.assertTrue(all(not c['ok'] for c in checks if c['name'].endswith('生成接口')))
        revision=next(c for c in self.preflight({'source':{'head':'abc1234'}})['checks'] if c['name'].startswith('源码版本'))
        self.assertFalse(revision['ok'])
        self.assertIn('unknown',revision['detail'])

    def test_missing_settings_or_all_output_cannot_claim_all_entries_exist(self):
        for result in (self.preflight(missing_settings=True),self.preflight(no_output=True)):
            checks={c['name']:c for c in result['checks']}
            self.assertFalse(checks['任务组入口']['ok'])
            self.assertNotIn('均存在',checks['任务组入口']['detail'])
        no_output={c['name']:c for c in self.preflight(no_output=True)['checks']}
        self.assertFalse(no_output['现场范围配置']['ok'])
        self.assertFalse(no_output['本机残留进程']['ok'])


if __name__=='__main__':unittest.main(verbosity=2)
