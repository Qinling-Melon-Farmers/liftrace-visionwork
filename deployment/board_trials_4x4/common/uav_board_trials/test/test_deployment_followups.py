"""Offline checks of real generators, CLI parsing and child command arguments."""
from pathlib import Path
import copy,io,json,os,shutil,subprocess,sys,tempfile,unittest
from contextlib import redirect_stdout
from unittest.mock import Mock,patch
import yaml

ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'deployment/board_trials_4x4'
SCRIPTS=BASE/'common/uav_board_trials/scripts'
sys.path[:0]=[str(ROOT/'vision_ws/src/uav_high_view/src'),str(SCRIPTS),str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src')]
import trial_config,trial_bag,run_trial
HAS_MOTION=(SCRIPTS/'trial_motion.py').is_file()
if HAS_MOTION:import trial_motion

class DeploymentFollowups(unittest.TestCase):
    def settings(self,kind='full_mission'):
        s=yaml.safe_load((BASE/trial_config.TRIAL_FOLDERS[kind]/'settings.yaml').read_text())
        if kind=='full_mission':
            s.update(corridor_waypoints=[dict(x=x,y=0,agl=.9) for x in (.6,1.2,1.8,2.4)],landing_xy=[2.8,0])
        return s

    def generated(self,s,fc=(0,0,0)):
        rig=yaml.safe_load((BASE/'common/uav_board_trials/config/known_rig.yaml').read_text())
        with tempfile.TemporaryDirectory() as out:
            trial_config.generate(ROOT,out,s,fc,rig)
            return [yaml.safe_load((Path(out)/n).read_text()) for n in ('runtime.yaml','control.yaml','overrides.yaml')]

    @unittest.skipUnless(HAS_MOTION,'No motion CLI/generator in this checkout')
    def test_wall_planes_follow_takeoff_offset_on_each_axis(self):
        for axis in (0,1):
            with self.subTest(axis=axis):
                s=self.settings()
                s.update(motion_optimization={'enabled':True},
                    corridor_geometry=dict(wall_axis=axis,wall_coordinates=[1.4,2.1],entry_waypoints=1))
                original=copy.deepcopy(s)
                helper=trial_motion.optimize_post_route
                with patch.object(trial_motion,'optimize_post_route',wraps=helper) as optimize:
                    r,c,o=self.generated(s,(.2,-.15,0))
                expected=[w+(.2,-.15)[axis] for w in (1.4,2.1)]
                self.assertEqual(optimize.call_args.args[4],expected)
                self.assertEqual(r['corridor_speed_schedule']['wall_coordinates'],expected)
                self.assertEqual(r['corridor_speed_schedule']['axis'],axis)
                baseline=self.generated(s)[0]
                for shifted,zero in zip(r['mission']['post_delivery_route'],baseline['mission']['post_delivery_route']):
                    self.assertAlmostEqual(shifted[0]-zero[0],.2)
                    self.assertAlmostEqual(shifted[1]-zero[1],-.15)
                    self.assertEqual(shifted[2],zero[2])
                self.assertEqual(s,original)

    @unittest.skipUnless(HAS_MOTION,'No motion CLI/generator in this checkout')
    def test_wall_axis_rejected_before_using_reference(self):
        s=self.settings()
        for axis in (-1,2,True):
            s.update(motion_optimization={'enabled':True},
                corridor_geometry=dict(wall_axis=axis,wall_coordinates=[1.4,2.1],entry_waypoints=1))
            with self.subTest(axis=axis),self.assertRaises(ValueError):
                self.generated(s)

    def cli_root(self,directory,settings,kind='high_priority'):
        root=Path(directory);base=root/'deployment/board_trials_4x4'
        source=BASE/'common/uav_board_trials/config'
        dest=base/'common/uav_board_trials/config';dest.mkdir(parents=True)
        for name in ('known_rig.yaml','mapping_startup.yaml'):shutil.copyfile(source/name,dest/name)
        folder=base/trial_config.TRIAL_FOLDERS[kind];folder.mkdir()
        (folder/'settings.yaml').write_text(yaml.safe_dump(settings))
        return root

    @unittest.skipUnless(HAS_MOTION,'No motion CLI/generator in this checkout')
    def test_cli_enables_without_discarding_policy_then_generates(self):
        policy=dict(enabled=False,moving_recovery=False,recovery_handoff_agl=.85,
                    recovery_max_odom_age=.15,dynamic_boundary=False)
        s=self.settings('high_priority');s['motion_optimization']=policy.copy()
        seen=[]
        validate=run_trial.validate_settings
        def observe(value):seen.append(copy.deepcopy(value));validate(value)
        with tempfile.TemporaryDirectory() as d:
            root=self.cli_root(d,s)
            args=['run_trial.py','high_priority','preview','--root',str(root),'--motion-optimized','--check-config']
            with patch.object(sys,'argv',args),patch.object(run_trial,'validate_settings',side_effect=observe),redirect_stdout(io.StringIO()):
                run_trial.main()
        self.assertEqual(seen[0]['motion_optimization'],dict(policy,enabled=True))
        self.assertEqual(policy['enabled'],False)
        r,c,o=self.generated(seen[0])
        for k,v in dict(policy,enabled=True).items():
            self.assertEqual(r['motion_optimization'][k],v)
            self.assertEqual(o['/navigation/planner_bridge/motion_optimization'][k],v)

    @unittest.skipUnless(HAS_MOTION,'No motion CLI/generator in this checkout')
    def test_cli_rejects_malformed_policy_instead_of_overwriting(self):
        s=self.settings('high_priority');s['motion_optimization']=None
        with tempfile.TemporaryDirectory() as d:
            root=self.cli_root(d,s)
            args=['run_trial.py','high_priority','preview','--root',str(root),'--motion-optimized','--check-config']
            with patch.object(sys,'argv',args),patch('sys.stderr',new_callable=io.StringIO),self.assertRaises(SystemExit) as exc:
                run_trial.main()
            self.assertEqual(exc.exception.code,2)

    def test_light_topics_preserve_cloud_switches(self):
        for map_clouds in (False,True):
            for inflated in (False,True):
                settings=dict(record_map_clouds=map_clouds,record_inflated_cloud=inflated)
                topics=trial_bag.topics_for(settings)
                with self.subTest(settings=settings):
                    self.assertEqual(topics.count('/laserMapping/realtime'),1)
                    self.assertEqual('/freedom/static_pointcloud' in topics,map_clouds)
                    self.assertEqual('/sdf_map/occupancy' in topics,map_clouds)
                    self.assertEqual('/sdf_map/occupancy_inflate' in topics,map_clouds or inflated)
                    self.assertNotIn('/livox/lidar',topics)
                    self.assertNotIn('/cloud_registered_body',topics)

    def test_actual_record_command_includes_diagnostic_and_throttled_camera(self):
        module=sys.modules[trial_bag.TrialBag.start.__module__]
        with tempfile.TemporaryDirectory() as d:
            child=Mock();child.poll.return_value=None
            with patch.object(module.subprocess,'Popen',return_value=child) as launch,patch.object(module.time,'sleep'):
                bag=trial_bag.TrialBag(d,{},dict(os.environ))
                try:
                    bag.start()
                    self.assertEqual(launch.call_count,2)
                    relay,record=[call.args[0] for call in launch.call_args_list]
                    self.assertEqual(relay[:4],['rosrun','topic_tools','throttle','messages'])
                    self.assertEqual(relay[5],'5.0')
                    self.assertEqual(record[:3],['rosbag','record','--lz4'])
                    self.assertIn('/laserMapping/realtime',record)
                    self.assertIn('/sdf_map/occupancy_inflate',record)
                    self.assertNotIn('/freedom/static_pointcloud',record)
                    self.assertNotIn('/sdf_map/occupancy',record)
                    self.assertNotIn('/livox/lidar',record)
                    metadata=json.loads((Path(d)/'bag_topics.json').read_text())
                    self.assertEqual(metadata['command'],record)
                    self.assertFalse(metadata['raw_lidar_recorded'])
                finally:
                    if bag.stream:bag.stream.close()

    @unittest.skipUnless(hasattr(run_trial,'run_session'),'Only the shared-session entry uses this default')
    def test_shared_session_and_competition_use_same_default_recorder(self):
        from uav_mission import hardware_session,hardware_bag
        self.assertIs(trial_bag.TrialBag,hardware_bag.TrialBag)
        self.assertIs(hardware_session.TrialBag,hardware_bag.TrialBag)
        original=hardware_session.TrialBag
        with tempfile.TemporaryDirectory() as d:
            root=self.cli_root(d,self.settings('high_priority'))
            args=['run_trial.py','high_priority','flight','--root',str(root)]
            def observe(*args,**kwargs):
                self.assertIs(hardware_session.TrialBag,original)
                self.assertIn('/laserMapping/realtime',original(d,{},{}).topics)
            with patch.object(sys,'argv',args),patch.object(run_trial,'run_session',side_effect=observe) as session:
                run_trial.main()
            session.assert_called_once()
            self.assertIs(hardware_session.TrialBag,original)
            # Exercise the real default recorder with competition settings too.
            settings=dict(competition_recording=True,
                bag_image_topic='/competition/recording/image/compressed',
                metadata_topic='/competition/run_metadata')
            child=Mock();child.poll.return_value=None
            with patch.object(hardware_bag.subprocess,'Popen',return_value=child) as launch,patch.object(hardware_bag.time,'sleep'):
                bag=original(d,settings,dict(os.environ))
                try:
                    bag.start()
                    command=launch.call_args.args[0]
                    self.assertEqual(command[:3],['rosbag','record','--lz4'])
                    self.assertIn('/laserMapping/realtime',command)
                    self.assertIn('/sdf_map/occupancy_inflate',command)
                    self.assertFalse(any(t.startswith('/board_trials/') for t in bag.topics))
                    self.assertNotIn('/livox/lidar',command)
                finally:
                    if bag.stream:bag.stream.close()

    @unittest.skipUnless((ROOT/'deployment/competition/build.sh').is_file(),'No competition build script in this checkout')
    def test_executed_build_arguments_default_override_and_invalid(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for name in ('deployment/competition','vision_ws/devel','patrol_uav_ws-patrol_planner','bin'):
                (root/name).mkdir(parents=True,exist_ok=True)
            script=root/'deployment/competition/build.sh'
            shutil.copyfile(ROOT/'deployment/competition/build.sh',script)
            (root/'vision_ws/devel/setup.bash').write_text(':\n')
            stub=root/'bin/catkin_make'
            stub.write_text('#!/usr/bin/python3\nimport json,os,sys\nwith open(os.environ["BUILD_CAPTURE"],"a") as f:f.write(json.dumps(dict(argv=sys.argv[1:],cwd=os.getcwd()))+"\\n")\n')
            stub.chmod(0o755)
            capture=root/'commands.jsonl'
            shell_env=root/'stub_env.sh'
            shell_env.write_text('catkin_make() { '+str(stub)+' "$@"; }'+chr(10))
            for threads in (None,'1','4','0','9','bad'):
                capture.unlink(missing_ok=True)
                env=dict(os.environ,PATH=str(root/'bin')+':'+os.environ['PATH'],
                         BUILD_CAPTURE=str(capture),BUILD_JOBS='2',BASH_ENV=str(shell_env))
                env.pop('FAST_LIO_MATCH_THREADS',None)
                if threads is not None:env['FAST_LIO_MATCH_THREADS']=threads
                result=subprocess.run(['bash',str(script)],env=env,capture_output=True,text=True)
                with self.subTest(threads=threads):
                    if threads in ('9','bad'):
                        self.assertEqual(result.returncode,2,result.stderr)
                        self.assertFalse(capture.exists())
                    else:
                        self.assertEqual(result.returncode,0,result.stderr)
                        calls=[json.loads(l) for l in capture.read_text().splitlines()]
                        self.assertEqual(len(calls),2)
                        self.assertFalse(any('FAST_LIO_MATCH_THREADS' in a for a in calls[0]['argv']))
                        self.assertIn('-DFAST_LIO_MATCH_THREADS='+('3' if threads is None else threads),calls[1]['argv'])
                        self.assertIn('fast_lio',calls[1]['argv'])
                        self.assertIn('-j2',calls[1]['argv'])
                        self.assertTrue(calls[1]['cwd'].endswith('patrol_uav_ws-patrol_planner'))

if __name__=='__main__':unittest.main()
