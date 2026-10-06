import math
from pathlib import Path
import unittest
import yaml
from observation_core import Observation,route,validate

class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.c=yaml.safe_load((Path(__file__).parent/'profiles.yaml').read_text())
        self.origin=(1.,2.,-.05,0.)

    def engine(self,profile='hover'):
        o=Observation(self.c,profile,self.origin,0.)
        o.step(0.,self.origin,10.,True,False,'POSCTL')
        return o

    def launch(self,o):
        o.step(2.1,self.origin,12.1,True,True,'OFFBOARD')
        self.assertEqual(o.stage,'RUN')

    def test_fc_agl_not_relative_climb_or_camera_height(self):
        points=route(self.c,'forward',self.origin)
        self.assertAlmostEqual(points[0][2],.33)
        self.assertAlmostEqual(points[0][2]-self.origin[2],.38)
        self.assertAlmostEqual(points[0][2]-(self.origin[2]-.22),.6)

    def test_route_follows_initial_heading_and_square_returns(self):
        start=(0.,0.,0.,math.pi/2)
        points=route(self.c,'forward',start)
        self.assertAlmostEqual(points[-1][0],0.)
        self.assertAlmostEqual(points[-1][1],1.5)
        self.assertEqual(route(self.c,'square',start)[-1],route(self.c,'square',start)[0])

    def test_arming_alone_and_inherited_offboard_do_not_start(self):
        o=self.engine()
        o.step(3.,self.origin,13.,True,True,'POSCTL')
        self.assertEqual(o.stage,'READY')
        o=Observation(self.c,'hover',self.origin,0.)
        o.step(0.,self.origin,10.,True,False,'OFFBOARD')
        o.step(3.,self.origin,13.,True,True,'OFFBOARD')
        self.assertEqual(o.stage,'READY')
        o.step(3.1,self.origin,13.1,True,True,'POSCTL')
        o.step(3.2,self.origin,13.2,True,True,'OFFBOARD')
        self.assertEqual(o.stage,'RUN')

    def test_ground_reference_change_does_not_silently_move_route(self):
        o=self.engine()
        moved=(1.2,2.,-.05,0.)
        o.step(3.,moved,13.,True,True,'OFFBOARD')
        self.assertEqual(o.stage,'HOLD_FOR_PILOT')
        self.assertEqual(o.target,moved)

    def test_pilot_takeover_never_resumes(self):
        o=self.engine();self.launch(o)
        self.assertIsNone(o.step(2.2,self.origin,12.2,True,True,'POSCTL'))
        self.assertIsNone(o.step(2.3,self.origin,12.3,True,True,'OFFBOARD'))
        self.assertEqual(o.stage,'TAKEN_OVER')

    def test_stale_and_stop_request_hold_instead_of_advancing(self):
        for reason in ('stale','stop'):
            o=self.engine();self.launch(o)
            target=o.target
            output=o.step(2.2,self.origin,12.2,reason!='stale',True,'OFFBOARD',stop_requested=reason=='stop')
            self.assertEqual(o.stage,'HOLD_FOR_PILOT')
            self.assertEqual(output,target)
            self.assertEqual(o.step(2.4,self.origin,12.4,True,True,'OFFBOARD'),target)

    def test_yaw_and_height_discontinuity_cancel_route(self):
        for pose in ((1,2,.5,0.),(1,2,-.05,math.pi/2)):
            o=self.engine();self.launch(o)
            output=o.step(2.2,pose,12.2,True,True,'OFFBOARD')
            self.assertEqual(o.stage,'HOLD_FOR_PILOT')
            self.assertEqual(output,pose)
            self.assertEqual(o.reason,'pose_discontinuity_take_over')

    def test_setpoint_cannot_run_far_ahead_of_stationary_aircraft(self):
        o=self.engine();self.launch(o)
        for i in range(1,201):
            now=2.1+i*.033
            o.step(now,self.origin,10+now,True,True,'OFFBOARD')
            self.assertLessEqual(math.dist(o.target[:3],self.origin[:3]),.15+1e-9)

    def test_three_profiles_finish_in_hover_without_land_command(self):
        for profile in ('hover','forward','square'):
            o=self.engine(profile);self.launch(o);pose=self.origin;previous=o.target
            for i in range(1,4500):
                now=2.1+i/30
                target=o.step(now,pose,10+now,True,True,'OFFBOARD',speed=0.)
                self.assertLessEqual(math.dist(target[:3],previous[:3]),.2/30+1e-8)
                pose=target;previous=target
                if o.stage=='FINISHED_HOVER':break
            self.assertEqual(o.stage,'FINISHED_HOVER',profile)
            self.assertEqual(o.target,o.goals[-1])
            self.assertEqual(o.step(now+.033,pose,10+now+.033,True,True,'OFFBOARD'),pose)

    def test_timeout_preserves_hold_output(self):
        o=self.engine();self.launch(o)
        out=o.step(153.,self.origin,163.,True,True,'OFFBOARD')
        self.assertEqual(o.stage,'HOLD_FOR_PILOT')
        self.assertIsNotNone(out)

    def test_preview_imports_no_hardware_and_does_not_start_processes(self):
        import subprocess,sys,json
        output=subprocess.check_output([sys.executable,str(Path(__file__).parent/'flight.py'),'preview','--profile','square'],text=True)
        preview=json.loads(output)
        self.assertFalse(preview['auto_arm']);self.assertFalse(preview['auto_mode'])
        self.assertFalse(preview['camera']);self.assertFalse(preview['servo'])

    def test_rejects_high_or_fast_profiles(self):
        for key,value in [('height_agl',2.),('horizontal_speed',1.),('height_agl',math.nan)]:
            bad=dict(self.c,**{key:value})
            with self.assertRaises(ValueError):validate(bad,'hover')

    def test_invalid_velocity_blocks_start_and_running_motion(self):
        for value in (math.inf,math.nan):
            o=self.engine()
            o.step(2.1,self.origin,12.1,True,True,'OFFBOARD',speed=value)
            self.assertEqual(o.stage,'HOLD_FOR_PILOT')
            o=self.engine();self.launch(o);target=o.target
            o.step(2.2,self.origin,12.2,True,True,'OFFBOARD',speed=value)
            self.assertEqual(o.stage,'HOLD_FOR_PILOT');self.assertEqual(o.target,target)

    def test_stop_latches_first_hold_position(self):
        o=self.engine();self.launch(o)
        first=(1.,2.,.1,0.)
        second=(1.03,2.,.13,0.)
        o.step(2.2,first,12.2,True,True,'OFFBOARD',stop_requested=True)
        o.step(2.3,second,12.3,True,True,'OFFBOARD',stop_requested=True)
        self.assertEqual(o.target,first)

    def test_lead_clipping_cannot_create_unbounded_run_setpoint_jump(self):
        o=self.engine();self.launch(o);before=o.target
        moved=(self.origin[0]+.18,self.origin[1],self.origin[2],self.origin[3])
        out=o.step(2.1+1/30,moved,12.1+1/30,True,True,'OFFBOARD')
        self.assertEqual(o.stage,'HOLD_FOR_PILOT')
        self.assertEqual(out,before)

    def test_ready_gap_requires_restart_not_late_auto_launch(self):
        o=self.engine()
        o.step(3.,None,0.,False,False,'POSCTL',connected=False)
        o.step(603.,self.origin,613.,True,True,'OFFBOARD')
        self.assertEqual(o.stage,'HOLD_FOR_PILOT')
        self.assertFalse(o.ever_started)

    def test_runtime_pose_and_speed_validate_frame_content_and_age(self):
        from types import SimpleNamespace as N
        from flight import valid_pose,valid_speed
        def header(frame='camera_init',stamp=10.):return N(frame_id=frame,stamp=N(to_sec=lambda:stamp))
        p=N(header=header(),pose=N(position=N(x=0.,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.)))
        self.assertTrue(valid_pose(p,'camera_init',10.,.3,0.))
        p.header.frame_id='wrong';self.assertFalse(valid_pose(p,'camera_init',10.,.3,0.))
        p.header=header();p.pose.position.x=math.nan
        self.assertFalse(valid_pose(p,'camera_init',10.,.3,0.))
        m=N(header=header('map'),twist=N(twist=N(linear=N(x=0.,y=0.,z=.1))))
        self.assertEqual(valid_speed(m,'map',10.,.3,0.),.1)
        self.assertTrue(math.isinf(valid_speed(m,'map',10.4,.3,0.)))
        m.twist.twist.linear.x=math.nan
        self.assertTrue(math.isinf(valid_speed(m,'map',10.,.3,0.)))

class ConflictTests(unittest.TestCase):
    def test_mavros_feedback_is_not_an_input_publisher(self):
        from flight import conflicts
        from types import SimpleNamespace
        pubs=[('/mavros/camera/image_captured',['/mavros']),
              ('/mavros/setpoint_trajectory/desired',['/mavros']),
              ('/mavros/setpoint_raw/target_local',['/mavros'])]
        self.assertEqual(conflicts(SimpleNamespace(getSystemState=lambda:(pubs,[],[])),'/test'),[])
    def test_real_command_and_camera_publishers_still_conflict(self):
        from flight import conflicts
        from types import SimpleNamespace
        pubs=[('/mavros/setpoint_position/local',['/other']),('/camera/image_raw',['/camera_sdk']),
              ('/mavros/setpoint_trajectory/desired',['/other'])]
        self.assertEqual(len(conflicts(SimpleNamespace(getSystemState=lambda:(pubs,[],[])),'/test')),3)

if __name__=='__main__':unittest.main()
