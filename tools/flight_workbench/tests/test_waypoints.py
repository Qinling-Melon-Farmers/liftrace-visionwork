"""Flight-before-edit contracts; no board or flight operations."""
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wb_board
import wb_waypoints as waypoints


class WaypointTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.profile = patch.object(wb_board, 'DEFAULT_PROFILE_DIR', self.directory.name)
        self.profile.start()
        config = wb_board.load_config(str(Path(__file__).resolve().parents[1]/'workbench.yaml'))
        self.wb = SimpleNamespace(config=config, target=SimpleNamespace(host='test-host', transport='local'),
            connection={'state':'unknown'}, sessions=SimpleNamespace(get=lambda key: None))
        self.wb.board = wb_board.BoardClient(config, self.wb.target)

    def tearDown(self):
        self.profile.stop();self.directory.cleanup()

    def save(self, site=None):
        context = waypoints.context(self.wb, {'group_id':'mod08'})
        value = copy.deepcopy(site or context['site'])
        value['corridor_waypoints'][3]['y'] = -.85
        return waypoints.save(self.wb, dict(group_id='mod08',scope=context['scope'],site=value))

    def test_persistence_and_return_to_original_without_deleting_draft(self):
        data = self.save()
        self.assertEqual(waypoints.draft(self.wb,{'group_id':'mod08'})['draft']['patch']['corridor_waypoints'][3]['y'],-.85)
        waypoints.disable(self.wb, dict(group_id='mod08',scope=data['scope']))
        context = waypoints.context(self.wb, {'group_id':'mod08'})
        self.assertFalse(context['draft']['enabled'])
        self.assertEqual(context['site']['corridor_waypoints'][3]['y'],-.9)

    def test_board_root_and_group_isolation(self):
        self.save()
        self.wb.target.host = 'other-host'
        self.assertIsNone(waypoints.draft(self.wb, {'group_id':'mod08'})['draft'])
        self.wb.target.host = 'test-host'
        self.assertIsNone(waypoints.draft(self.wb, {'group_id':'mod04'})['draft'])
        self.wb.config['connection']['board_root'] = '/other/project'
        self.assertIsNone(waypoints.draft(self.wb, {'group_id':'mod08'})['draft'])

    def test_active_task_and_stale_source_rejected(self):
        context = waypoints.context(self.wb, {'group_id':'mod08'})
        body = dict(group_id='mod08',scope=context['scope'],site=context['site'])
        self.wb.sessions = SimpleNamespace(get=lambda key:SimpleNamespace(state='running'))
        with self.assertRaises(ValueError):waypoints.save(self.wb, body)
        self.wb.sessions = SimpleNamespace(get=lambda key:None)
        self.wb.target.host = 'other-host'
        with self.assertRaises(ValueError):waypoints.save(self.wb, body)

    def test_preserves_other_business_and_builds_the_actual_start_overlay(self):
        data = self.save()
        self.assertEqual(set(data['patch']),set(waypoints.FIELDS))
        group = next(g for g in self.wb.config['groups'] if g['id']=='mod08')
        for mode,check in [('preview',True),('preview',False),('flight',False)]:
            command,_ = wb_board.build_group_command(self.wb.config,group,mode,route='module',check_config=check,
                site_geometry=data['patch'],geometry_revision=data['revision'])
            self.assertIn('site_overlays/08_full_mission_',command)
            self.assertNotIn('--survey-pattern',command)
            self.assertIn(' && bash deployment/board_trials_4x4/08_full_mission/start.sh',command)

    def test_source_module_accepts_adjusted_geometry(self):
        repository=Path(__file__).resolve().parents[3]
        if not repository.exists():self.skipTest('source repository not available')
        for path in ['deployment/board_trials_4x4/common/uav_board_trials/scripts',
                     'patrol_uav_ws-patrol_planner/src/uav_mission/src','vision_ws/src/uav_high_view/src']:
            sys.path.insert(0,str(repository/path))
        from trial_config import apply_site_profile,validate_settings
        settings=yaml.safe_load((repository/'deployment/board_trials_4x4/08_full_mission/settings.yaml').read_text(encoding='utf-8'))
        base=yaml.safe_load((repository/'deployment/board_trials_4x4/08_full_mission/site_20261007_221730.yaml').read_text(encoding='utf-8'))
        merged=waypoints.merge(base,self.save()['patch'])
        apply_site_profile(settings,merged);validate_settings(settings)
        self.assertEqual(merged['corridor_waypoints'][3]['y'], -.85)


if __name__=='__main__':unittest.main()
