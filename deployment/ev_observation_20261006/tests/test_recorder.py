import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from ev_suite.bootstrap import HERE
import record_shadow


class RecorderTests(unittest.TestCase):
    def test_extra_shadow_config_accepts_snapshot_and_exact_raw_imu(self):
        module = record_shadow.recorder_module()
        cfg = record_shadow.config_check(module, HERE/'config/recording_shadow.yaml')
        self.assertEqual({x['name'] for x in cfg['topics']}, set(record_shadow.TOPICS))
        self.assertEqual(cfg['queue_bytes'], 16777216)
        self.assertEqual(cfg['max_duration_s'], 600)

    def test_arbitrary_livox_image_cloud_or_custom_type_rejected(self):
        module = record_shadow.recorder_module()
        for name, types in (('/livox/lidar', ['sensor_msgs/PointCloud2']),
                            ('/livox/other', ['sensor_msgs/Imu']),
                            ('/camera/image_raw', ['sensor_msgs/Image']),
                            ('/laserMapping/prediction_state', ['std_msgs/String'])):
            with self.assertRaises(ValueError): module.validate_topic(name, types)

    def test_preview_does_not_create_output_or_need_master(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp)/'recording'
            result = subprocess.run([sys.executable, str(HERE/'record_shadow.py'),
                '--output', str(target)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            data = json.loads(result.stdout)
            self.assertTrue(data['preview'])
            self.assertEqual(data['publishers'], [])
            self.assertFalse(target.exists())

    def test_extra_config_does_not_relax_original_recorder(self):
        import importlib.util
        path = record_shadow.ROOT/'deployment/low_hover_observation/record_diagnostics.py'
        spec = importlib.util.spec_from_file_location('unchanged_low_recorder', str(path))
        original = importlib.util.module_from_spec(spec); spec.loader.exec_module(original)
        self.assertNotIn('fast_lio/PredictionState', original.SAFE_TYPES)
        with self.assertRaises(ValueError): original.validate_topic('/livox/imu', ['sensor_msgs/Imu'])


if __name__ == '__main__': unittest.main()
