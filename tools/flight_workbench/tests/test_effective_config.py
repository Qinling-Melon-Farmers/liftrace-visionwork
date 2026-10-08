"""Runtime evidence is parsed from supervisor output, never from selections."""
import json,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from wb_status import StageTracker
class EffectiveConfigTests(unittest.TestCase):
    def test_check_and_runtime_sources_remain_distinct(self):
        tracker=StageTracker()
        self.assertIsNone(tracker.snapshot()['effective_config'])
        checked=dict(status='CONFIG_VALID',source='validated_settings',motion_optimization=True,resume_survey=False,generation_ready=False)
        tracker.feed(json.dumps(checked)+'\n')
        self.assertEqual(tracker.snapshot()['effective_config'],checked)
        actual=dict(source='generated_runtime',motion_optimization=False,resume_survey=True,runtime_path='/logs/run/runtime.yaml')
        tracker.feed('CONFIG_EFFECTIVE '+json.dumps(actual)+'\nREADY: /logs/run\n')
        self.assertEqual(tracker.snapshot()['effective_config'],actual)
        tracker.reset()
        self.assertIsNone(tracker.snapshot()['effective_config'])
    def test_missing_or_wrong_types_never_claim_effective_values(self):
        tracker=StageTracker()
        for data in ({'status':'CONFIG_VALID','motion_optimization':True},
                     {'status':'CONFIG_VALID','motion_optimization':'true','resume_survey':True},
                     {'motion_optimization':True,'resume_survey':True}):
            tracker.feed(json.dumps(data)+'\n')
        tracker.feed('CONFIG_VALID; no ROS nodes started\n')
        self.assertIsNone(tracker.snapshot()['effective_config'])
if __name__=='__main__':unittest.main()
