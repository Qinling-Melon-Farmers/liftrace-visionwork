import csv
import json
import math
import stat
import sys
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace
from zipfile import ZipFile, ZipInfo

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis import Analysis, Options, Stream, at, euler, export, main, normalize
from safe_zip import inspect_zip, extract_sources
import app


def dataset(name, times, instance=0, **fields):
    return SimpleNamespace(name=name, multi_id=instance,
                           data={'timestamp': np.asarray(times)*1e6+1e6,
                                 **{k: np.asarray(v) for k, v in fields.items()}})


def analyze(*datasets, options=None, dropouts=()):
    return Analysis(Path('synthetic.ulg'), SimpleNamespace(start_timestamp=1000000,
                    data_list=list(datasets), dropouts=list(dropouts),
                    msg_info_dict={}, initial_parameters={}), options)


def fixture():
    times = np.arange(0, 12.01, .1)
    states = np.arange(13)
    return analyze(
        dataset('actuator_motors', times, **{'control[0]': np.where(times < 2, .99, .5),
                                            'control[1]': np.where(times > 9, 1, .6)}),
        dataset('actuator_armed', states, armed=np.ones(13), lockdown=np.zeros(13),
                manual_lockdown=np.zeros(13), force_failsafe=np.zeros(13)),
        dataset('vehicle_land_detected', states, landed=(states < 2) | (states >= 10),
                ground_contact=(states < 2) | (states >= 10), maybe_landed=np.zeros(13)),
        dataset('manual_control_switches', states, kill_switch=np.where(states >= 9, 2, 1)),
        options=Options(guard_sec=.5))


class TimeAndWindowTests(unittest.TestCase):
    def test_causal_age_and_no_future_backfill(self):
        s = Stream('s', 0, np.array([2., 4.]), {'v': np.array([7., 9.])})
        actual = at(s, 'v', np.array([1., 2., 3., 3.01, 4.]), 1)
        np.testing.assert_allclose(actual, [np.nan, 7, 7, np.nan, 9], equal_nan=True)

    def test_duplicate_last_wins_and_rewinds_reported(self):
        s = normalize('s', 0, {'timestamp': [3e6, 2e6, 2e6], 'v': [3, 1, 2]}, 1e6)
        np.testing.assert_equal(s.time, [1, 2])
        np.testing.assert_equal(s.data['v'], [2, 3])
        self.assertEqual(s.rewinds, 1)

    def test_ground_kill_transition_excluded_from_comparison(self):
        a = fixture()
        _, windows = a.context(np.array([1., 2., 3., 8.9, 9.7, 10., 11.]))
        self.assertEqual(list(windows), ['ground', 'transition_guard', 'motor_airborne',
                         'transition_guard', 'transition_guard', 'transition_guard', 'ground'])
        rows = [r for r in a.motor_stats() if r['window'] == 'motor_airborne']
        self.assertTrue(rows)
        self.assertAlmostEqual(next(r['mean'] for r in rows if r['motor'] == 'control[0]'), .5)
        self.assertAlmostEqual(next(r['mean'] for r in rows if r['motor'] == 'control[1]'), .6)

    def test_missing_land_state_does_not_invent_airborne(self):
        a = analyze(dataset('actuator_motors', [0, .1], **{'control[0]': [.5, .6]}),
                    dataset('actuator_armed', [0], armed=[1]))
        self.assertEqual(list(a.context(np.array([0., .1]))[1]), ['unknown', 'unknown'])
        self.assertFalse(any(r['window'] == 'motor_airborne' for r in a.motor_stats()))

    def test_aged_states_are_unknown(self):
        a = analyze(dataset('actuator_armed', [0], armed=[1], lockdown=[0],
                            manual_lockdown=[0], force_failsafe=[0]),
                    dataset('vehicle_land_detected', [0], landed=[0], ground_contact=[0]))
        self.assertEqual(a.context(np.array([3.]))[1][0], 'unknown')

    def test_spike_guard_is_not_a_collision_diagnosis(self):
        a = analyze(dataset('vehicle_local_position', [1, 2, 3], ax=[0, 20, 0],
                            ay=[0, 0, 0], az=[0, 0, 0]))
        self.assertEqual(a.context(np.array([2.]))[1][0], 'suspected_impact')

    def test_gap_no_held_duration_credit(self):
        a = analyze(dataset('actuator_motors', [0, .1, 10, 10.1],
                            **{'control[0]': [1, 1, 1, 1]}))
        self.assertEqual(a.context(np.array([5.]))[1][0], 'logging_gap')
        self.assertTrue(all(r.get('held_command_s_ge_0999', 0) == 0 for r in a.motor_stats()
                            if r['window'] == 'logging_gap'))

    def test_logger_dropout_exclusion(self):
        a = analyze(dropouts=[SimpleNamespace(timestamp=3e6, duration=500)])
        self.assertEqual(a.context(np.array([2.2]))[1][0], 'logging_gap')

    def test_nan_population_comparison_is_simultaneous(self):
        a = fixture()
        s = a.get('actuator_motors')
        s.data['control[1]'][::2] = np.nan
        spread = next(r for r in a.motor_stats() if r['motor'] == 'simultaneous_max_minus_min')
        self.assertAlmostEqual(spread['mean'], .1)
        m1 = next(r for r in a.motor_stats() if r['motor'] == 'control[0]' and r['window'] == 'motor_airborne')
        self.assertLess(spread['samples'], m1['samples'])


class SourceMeaningTests(unittest.TestCase):
    def test_no_display_automatically_uses_headless_without_tk(self):
        with patch.dict('os.environ', {}, clear=True), patch.object(sys, 'argv', ['app.py', 'one.ulg']), \
                patch.object(app, 'batch', return_value=0) as batch:
            self.assertEqual(app.main(), 0)
            batch.assert_called_once_with(['one.ulg'], default_output=True)

    def test_cli_partial_failure_keeps_good_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'batch'
            def reader(path, options):
                if path.name == 'bad.ulg':
                    raise ValueError('test invalid header')
                return fixture()
            with patch('analysis.read_log', side_effect=reader):
                self.assertEqual(main(['bad.ulg', 'good.ulg', '--no-plots', '--output', str(output)]), 1)
            self.assertTrue((output/'002_good/summary.json').exists())
            self.assertIn('test invalid header', (output/'batch_errors.csv').read_text(encoding='utf-8-sig'))

    def test_reset_wrap_initial_not_event_delta_is_latest(self):
        a = analyze(dataset('vehicle_local_position', [0, 1, 2],
                            z_reset_counter=[254, 255, 0], delta_z=[0, .3, -.1]))
        events = a.report()['reset_events']
        self.assertEqual(len(events), 2)
        self.assertEqual(events[-1]['counter_step_mod256'], 1)
        self.assertAlmostEqual(events[-1]['delta'], -.1)

    def test_estimator_selection_not_instance_zero_assumption(self):
        a = analyze(dataset('estimator_selector_status', [0, 2], primary_instance=[0, 1]),
                    dataset('estimator_status_flags', [0, 1, 2, 3], cs_ev_pos=[0, 0, 0, 0]),
                    dataset('estimator_status_flags', [0, 1, 2, 3], instance=1, cs_ev_pos=[1, 1, 1, 1]))
        np.testing.assert_equal(a.selected('estimator_status_flags', 'cs_ev_pos', np.array([1., 2., 3.])), [0, 1, 1])
        del a.streams[('estimator_selector_status', 0)]
        self.assertTrue(np.all(np.isnan(a.selected('estimator_status_flags', 'cs_ev_pos', np.array([1.])))))

    def test_quaternion_normalization_and_invalid(self):
        angle = np.pi/4
        result = euler({'q[0]': [2*np.cos(angle), 0], 'q[1]': [0, 0],
                        'q[2]': [0, 0], 'q[3]': [2*np.sin(angle), 0]})
        self.assertAlmostEqual(result[0, 2], 90)
        self.assertTrue(np.all(np.isnan(result[1])))

    def test_raw_output_fallback_is_not_normalized_motor(self):
        a = analyze(dataset('actuator_outputs', [0], **{'output[0]': [1500]}))
        self.assertFalse(a.motor_stats())
        self.assertTrue(any('No actuator_motors' in w for w in a.report()['warnings']))

    def test_csv_estimated_height_not_truth_and_no_nan_json(self):
        a = analyze(dataset('vehicle_local_position', [0, 1], z=[-.4, np.nan], z_valid=[1, 0]))
        with tempfile.TemporaryDirectory() as tmp:
            export(a, Path(tmp)/'out', plots=False)
            text = (Path(tmp)/'out/summary.json').read_text()
            self.assertNotIn('NaN', text)
            with (Path(tmp)/'out/vehicle_local_position_instance0.csv').open(encoding='utf-8-sig') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(float(rows[0]['estimated_local_up_m']), .4)
            self.assertEqual(rows[1]['estimated_local_up_m'], '')

    def test_empty_and_missing_topics_can_export_plot(self):
        with tempfile.TemporaryDirectory() as tmp:
            export(analyze(), Path(tmp)/'out')
            self.assertGreater((Path(tmp)/'out/overview.png').stat().st_size, 1000)


class ZipTests(unittest.TestCase):
    def archive(self, tmp, names):
        path = Path(tmp)/'input.zip'
        with ZipFile(path, 'w') as z:
            for name in names:
                z.writestr(name, 'source')
        return path

    def test_reject_paths_before_any_extraction(self):
        for name in ('../escape.py', '/tmp/escape.py', 'C:/escape.py', '..\\escape.py',
                     'folder/../../escape.py'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                path = self.archive(tmp, ['safe.py', name])
                with self.assertRaises(ValueError):
                    extract_sources(path, Path(tmp)/'out')
                self.assertFalse((Path(tmp)/'out/safe.py').exists())

    def test_reject_symlink_duplicate_and_size(self):
        with tempfile.TemporaryDirectory() as tmp:
            info = ZipInfo('link')
            info.create_system = 3
            info.external_attr = (stat.S_IFLNK | 0o777) << 16
            path = self.archive(tmp, [info])
            with self.assertRaises(ValueError):
                inspect_zip(path)
            path = self.archive(tmp, ['a.py', 'A.py'])
            with self.assertRaises(ValueError):
                inspect_zip(path)
            path = self.archive(tmp, ['safe.py'])
            with self.assertRaises(ValueError):
                inspect_zip(path, max_bytes=1)

    def test_safe_extract_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self.archive(tmp, ['tool/app.py'])
            extract_sources(path, Path(tmp)/'out')
            self.assertEqual((Path(tmp)/'out/tool/app.py').read_text(), 'source')
            with self.assertRaises(FileExistsError):
                extract_sources(path, Path(tmp)/'out')


if __name__ == '__main__':
    unittest.main()
