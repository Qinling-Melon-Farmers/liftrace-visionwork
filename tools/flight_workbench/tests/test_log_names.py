"""Offline naming checks; no SSH, ROS, recorder or flight operations."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from urllib.parse import unquote

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wb_log_names import REMOTE_INFO, attachment_header, download_name


class LogNameTests(unittest.TestCase):
    def test_modes_and_chinese_http_header(self):
        info = dict(timestamp='20261013153001', settings=dict(
            drop_horizontal_compensation_mode=2, drop_descent_mode=1,
            landing_alignment_mode='vh', landing_descent_mode=2))
        name = download_name('run/flight_debug_0.bag', info)
        self.assertEqual(name, '1013-1530-投递H2D1-H降落vhD2.bag')
        header = attachment_header(name)
        header.encode('ascii')
        self.assertEqual(unquote(header.split("filename*=UTF-8''")[1]), name)

    def test_unknown_modes_and_active_extension(self):
        name = download_name('run/flight.bag.active', dict(timestamp='20261013153001'))
        self.assertEqual(name, '1013-1530-投递H未知D未知-H降落未知D未知.bag.active')
        self.assertEqual(download_name('run/flight.bag', {}), 'flight.bag')
        self.assertEqual(download_name('run/run_metadata.json', {}), 'run_metadata.json')

    def test_epoch_uses_china_time_and_old_alignment_alias(self):
        info = dict(started_at=0, settings=dict(landing_alignment_mode='low'))
        self.assertIn('0101-0800-', download_name('flight.bag', info))
        self.assertIn('H降落non_vhD未知', download_name('flight.bag', info))

    def test_reads_only_own_run_ancestors_and_run_timestamp(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root/'board_full_mission_20261013_153001'
            recording = run/'recording'; recording.mkdir(parents=True)
            path = recording/'flight_debug_0.bag';path.write_bytes(b'fixture')
            (run/'run_metadata.json').write_text(json.dumps(dict(settings=dict(
                drop_horizontal_compensation_mode=1, drop_descent_mode=4,
                landing_alignment_mode='vh', landing_descent_mode=3))))
            def inside(base, value):
                value = value.resolve()
                if base != value and base not in value.parents:
                    raise ValueError('outside root')
                return value
            def read_json(value):
                try: return json.loads(value.read_text())
                except (OSError, ValueError): return {}
            scope = dict(json=json, inside=inside, read_json=read_json)
            exec(REMOTE_INFO, scope)
            info = scope['download_info'](root, path)
            self.assertEqual(download_name(path.name, info), '1013-1530-投递H1D4-H降落vhD3.bag')
            other = root/'bag_20261013_160000_abcdefgh';other.mkdir()
            active = other/'flight.bag.active';active.write_bytes(b'fixture')
            info = scope['download_info'](root, active)
            self.assertEqual(info['settings'], {})
            self.assertEqual(info['timestamp'], '20261013160000')


if __name__ == '__main__':
    unittest.main()
