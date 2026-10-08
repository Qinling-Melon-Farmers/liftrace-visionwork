"""Read-only cross-worktree CLI check, never starts ROS or imports a live profile."""
import argparse
import itertools
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import sys

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
try:
    import pexpect
except ImportError:
    sys.path.append('/usr/lib/python3/dist-packages')  # Pure Python pexpect, after conda packages.
import yaml
import wb_board


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--competition-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    root = a.competition_root.resolve()
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join([str(root / 'patrol_uav_ws-patrol_planner/src/uav_mission/src'),
        str(root / 'vision_ws/src/uav_high_view/src'), env.get('PYTHONPATH', '')])
    entry = root / 'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/competition_supervisor.py'
    config = wb_board.load_config()
    group = next(g for g in config['groups'] if g['id'] == 'competition')
    settings = yaml.safe_load((root / group['site_config']).read_text(encoding='utf-8'))
    assert settings['site_confirmed'] is False
    assert settings['corridor_waypoints'] == [] and settings['landing_xy'] is None
    assert (settings['high_agl'], settings['cruise_speed'], settings['cruise_acceleration']) == (2.6, 1.2, 1.0)
    assert [settings['following_speed_profile'][k] for k in ('cruise_lead_m','precision_lead_m','corridor_lead_m')] == [1., .4, .15]
    report = dict(source_root=str(root), template=group['site_config'], board_connected=False,
        competition_values=dict(high_agl=2.6, cruise_speed=1.2, cruise_acceleration=1., following=[1., .4, .15]), cases=[])
    for site in (group['site_config'], 'deployment/competition/field_20261007_validated.yaml'):
      site_settings = yaml.safe_load((root / site).read_text(encoding='utf-8'))
      for motion, columns, resume in itertools.product((None, 'on', 'off'), repeat=3):
        command, _ = wb_board.build_group_command(config, group, 'preview', check_config=True,
            motion_optimization=motion, obstacle_columns=columns, resume_survey=resume, competition_config=site)
        # Exercise the real Python CLI with exactly the workbench-generated
        # arguments. The shell wrapper only supplies root and board ROS env.
        argv = shlex.split(command)
        result = subprocess.run([sys.executable, str(entry), *argv[2:], '--root', str(root)],
                                cwd=root, env=env, capture_output=True, text=True, timeout=20)
        assert result.returncode == 0, result.stdout + result.stderr
        effective = json.loads(result.stdout.strip())
        assert effective['ros_started'] is False and effective['site_confirmed'] == site_settings['site_confirmed']
        expected_motion = site_settings.get('motion_optimization', {}).get('enabled', False) if motion is None else motion == 'on'
        expected_columns = site_settings['obstacle_columns_enabled'] if columns is None else columns == 'on'
        expected_resume=site_settings.get('survey_policy',{}).get('resume_survey_enabled',False) if resume is None else resume=='on'
        assert effective['resume_survey'] == expected_resume
        assert effective['motion_optimization'] == expected_motion
        assert effective['obstacle_columns'] == expected_columns
        if site != group['site_config']:
            with tempfile.TemporaryDirectory(prefix='wb-competition-expand-') as tmp:
                generated = subprocess.run([sys.executable, str(entry), *argv[2:], '--root', str(root),
                    '--output-dir', tmp, '--fc-reference', '0', '0', '0.22'],
                    cwd=root, env=env, capture_output=True, text=True, timeout=20)
                assert generated.returncode == 0, generated.stdout + generated.stderr
                runtime = yaml.safe_load((Path(tmp) / 'runtime.yaml').read_text())
                overrides = yaml.safe_load((Path(tmp) / 'overrides.yaml').read_text())
                assert runtime['high_view_full']['policy']['resume_survey_enabled'] == expected_resume
                reference=json.loads((Path(tmp)/'ground_reference.json').read_text())
                assert reference['effective_config']['resume_survey']==expected_resume
                assert reference['effective_config']['motion_optimization']==expected_motion
                assert runtime['motion_optimization']['enabled'] == expected_motion
                assert overrides['/navigation/planner_bridge/motion_optimization']['enabled'] == expected_motion
                assert overrides['/fast_planner_node/sdf_map/horizontal_avoidance/enabled'] == expected_columns
        report['cases'].append(dict(command=command, motion=motion or 'inherit', columns=columns or 'inherit', resume=resume or 'inherit', effective=effective))
    result = subprocess.run([sys.executable, str(entry), 'flight', '--root', str(root),
        '--site-config', group['site_config'], '--check-config'], cwd=root, env=env,
        capture_output=True, text=True, timeout=20)
    assert result.returncode != 0, 'Unconfirmed competition template accepted as flight'
    report['unconfirmed_flight_rejected'] = True
    report['result'] = 'PASS'
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('PASS 54 actual CLI combinations, 27 generated-runtime expansions; motion/resume independent; unconfirmed flight rejected; ROS not started')


if __name__ == '__main__':
    main()
