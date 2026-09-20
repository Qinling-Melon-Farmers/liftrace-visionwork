from pathlib import Path
import copy,tempfile,unittest,yaml,json,subprocess,sys,struct
from trial_config import generate,validate_settings
R=Path(__file__).resolve().parents[5]
class CorridorConfigTest(unittest.TestCase):
    def setUp(self):
        self.settings=yaml.safe_load((R/'deployment/board_trials_4x4/04_corridor_landing/settings.yaml').read_text())
        self.rig=yaml.safe_load((R/'deployment/board_trials_4x4/common/uav_board_trials/config/known_rig.yaml').read_text())
    def filled(self):
        s=copy.deepcopy(self.settings);s['corridor_waypoints']=[dict(x=.6,y=0),dict(x=1.5,y=.4,agl=1.2)];s['landing_xy']=[2.5,0];return s
    def test_unfilled_configuration_cannot_generate_route(self):
        with tempfile.TemporaryDirectory() as out:
            with self.assertRaisesRegex(ValueError,'corridor_waypoints'):generate(R,out,self.settings,(0,0,0),self.rig)
    def test_generated_route_preserves_order_and_converts_agl(self):
        with tempfile.TemporaryDirectory() as out:
            generate(R,out,self.filled(),(0,0,0),self.rig)
            runtime=yaml.safe_load((Path(out)/'runtime.yaml').read_text());control=yaml.safe_load((Path(out)/'control.yaml').read_text())
            route=runtime['mission']['post_delivery_route'];self.assertEqual(len(route),4)
            self.assertEqual(route[1][:2],[1.5,.4]);self.assertAlmostEqual(route[1][2],.98)
            self.assertEqual(route[-1][:2],[2.5,0]);self.assertAlmostEqual(route[-1][2],1.58)
            self.assertEqual(runtime['runtime']['start_mode'],'post_delivery');self.assertFalse(control['drop_system']['enable_drop'])
            s=self.filled();s['corridor_waypoints'][-1]=dict(x=2.5,y=0)
            generate(R,out,s,(0,0,0),self.rig)
            runtime=yaml.safe_load((Path(out)/'runtime.yaml').read_text())
            self.assertEqual(len(runtime['mission']['post_delivery_route']),3)
    def test_invalid_goal_or_missing_h_rejected(self):
        for key,value in [('landing_xy',None),('landing_xy',[4.,0]),('corridor_waypoints',[dict(x=.6,y=0),dict(x=2,y=0,agl=float('nan'))])]:
            s=self.filled();s[key]=value
            with self.assertRaises(ValueError):validate_settings(s)
    def test_landing_result_does_not_require_mock_deliveries(self):
        script=R/'deployment/board_trials_4x4/common/uav_board_trials/scripts/finish_recording.py'
        for phase,expected in [('COMPLETE','PASS'),('RETURNING','INCOMPLETE')]:
            with tempfile.TemporaryDirectory() as tmp:
                out=Path(tmp)
                (out/'supervisor_result.json').write_text(json.dumps(dict(trial='corridor_landing',end_reason='landed_after_flight')))
                (out/'vision_events.jsonl').write_text(json.dumps(dict(kind='mission',data=dict(phase=phase,committed_slots=0)))+'\n')
                subprocess.run([sys.executable,str(script),str(out)],check=True,stdout=subprocess.DEVNULL)
                result=json.loads((out/'result.json').read_text());self.assertEqual(result['status'],expected);self.assertEqual(result['expected_mock_deliveries'],0)
    def test_actual_ground_offset_does_not_trip_float_double_height_guard(self):
        for fc_z in [-.061408067122101784,-.05,0.,.1]:
            with tempfile.TemporaryDirectory() as tmp:
                generate(R,tmp,self.filled(),(0,0,fc_z),self.rig)
                c=yaml.safe_load((Path(tmp)/'control.yaml').read_text())
                cpp_align=struct.unpack('f',struct.pack('f',c['align_height']))[0]
                self.assertLessEqual(c['uav_vision']['recovery_height'],cpp_align)
if __name__=='__main__':unittest.main()
