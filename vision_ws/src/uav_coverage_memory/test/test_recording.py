import tempfile
import unittest
from pathlib import Path
import json
import numpy as np
from uav_coverage_memory.recording import Recorder
from uav_coverage_memory.memory import Camera, Config


class RecordingTests(unittest.TestCase):
    def test_bounded_source_matched_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=Recorder(directory, interval=6, limit=2)
            camera=Camera(4,4,(2.,0,2.,0,2.,2.,0,0,1),(),'optical')
            config=Config(-1,1,-1,1,.1,0,'map')
            def sample(t,g=1):
                return recorder.sample({'image_stamp':t,'generation':g},np.ones((4,4),np.uint8),
                    camera,np.eye(4),None,None,np.zeros((2,2)),config)
            self.assertTrue(sample(1))
            self.assertFalse(sample(2))
            self.assertTrue(sample(7))
            self.assertFalse(sample(20))
            recorder.status({'accepted':True});recorder.close()
            meta=json.loads((Path(directory)/'0000.json').read_text())
            self.assertIsNone(meta['map_stamp'])
            self.assertEqual(np.load(Path(directory)/'0000.npz')['map_points'].shape,(0,3))
            self.assertEqual(len(list(Path(directory).glob('*.png'))),2)

    def test_recording_restart_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=Recorder(directory);recorder.close()
            with self.assertRaises(FileExistsError):Recorder(directory)

    def test_invalid_bounds(self):
        with self.assertRaises(ValueError):Recorder('relative')
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):Recorder(directory,limit=0)
