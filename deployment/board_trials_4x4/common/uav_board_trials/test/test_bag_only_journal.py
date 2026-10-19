from pathlib import Path
import json,tempfile,unittest,subprocess,sys
from unittest.mock import patch
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trial_journal import Journal

class BagOnlyJournalTest(unittest.TestCase):
    def test_hardware_journal_has_no_camera_subscription_and_keeps_finish_state(self):
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)
            with patch.object(rospy,'get_param',side_effect=lambda k,d=None:directory if k=='~directory' else d),patch.object(rospy,'Subscriber') as subscriber,patch.object(rospy,'on_shutdown'),patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.)):
                j=Journal()
                self.assertTrue(all('Image' not in c.args[1].__name__ for c in subscriber.call_args_list))
                self.assertTrue(any(c.args[0]=='/board_trials/terminal_hover_status' for c in subscriber.call_args_list))
                for key,data in [('mission',dict(phase='LAND',mission_failed=False,committed_slots=0)),('high',dict(capture_complete=True)),('terminal_hover',dict(stage='PILOT_HANDOFF'))]:
                    j.text(key,String(data=json.dumps(data)))
                p=PoseStamped();p.header.stamp=rospy.Time.from_sec(10.);p.header.frame_id='camera_init';p.pose.position.x=1.
                j.pose(p);j.close();j.close()
            self.assertFalse((out/'camera_frames.csv').exists())
            self.assertEqual(json.loads((out/'recording.json').read_text())['camera_storage'],'rosbag_only')
            self.assertEqual(len((out/'navigation_pose.csv').read_text().splitlines()),2)
            (out/'supervisor_result.json').write_text(json.dumps(dict(trial='high_speed_capture',end_reason='landed_after_flight',actuator_mode='none')))
            (out/'bag_recording.json').write_text(json.dumps(dict(status='PASS',bags=['flight_debug_0.bag'])))
            script=Path(__file__).resolve().parents[1]/'scripts/finish_trial.py'
            subprocess.run([sys.executable,str(script),str(out)],check=True,stdout=subprocess.DEVNULL)
            self.assertEqual(json.loads((out/'result.json').read_text())['status'],'CAPTURED')
            self.assertIn('flight_debug_0.bag',(out/'index.html').read_text())
            self.assertEqual(list(out.glob('*.mp4')),[])
if __name__=='__main__':unittest.main()