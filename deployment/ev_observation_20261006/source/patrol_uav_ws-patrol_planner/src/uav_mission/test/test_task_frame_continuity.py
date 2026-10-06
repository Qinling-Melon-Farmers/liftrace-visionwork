import math
import unittest
from dataclasses import replace

from uav_mission.task_frame_continuity import (
    BoundaryRejected, FcReset, FcState, LioHealth, Pose, TaskFrameBoundary,
    Transform)

I = Transform((0.,0.,0.), (0.,0.,0.,1.))


def boundary(**kw):
    params = dict(task_frame='task', fc_frame='map', lio_frame='lio',
                  fc_epoch='fc-boot-1', lio_epoch='lio-boot-1',
                  task_from_fc=I, task_from_lio=I,
                  body_to_camera=Transform((0.,0.,-.16), (0.,1.,0.,0.)),
                  ground_z=0., calibration_verified=True, initial_reset_counter=0)
    params.update(kw)
    return TaskFrameBoundary(**params)


def pair(b, t, z=.6, fc_z=None, armed=True, mode='OFFBOARD', health=None):
    fc = Pose(t, 'map', (0.,0., z if fc_z is None else fc_z), (0.,0.,0.,1.))
    lio = Pose(t, 'lio', (0.,0.,z), (0.,0.,0.,1.))
    h = health or LioHealth(t, 'lio-boot-1', 'lio', True, True)
    return b.observe(fc, lio, h, FcState(t, True, armed, mode), t)


def ready():
    b = boundary()
    for t in (1.,1.06,1.12):
        pair(b, t, armed=False, mode='POSCTL')
    pair(b, 1.18)
    assert b.ready
    return b


def reset(t=1.2, delta=.524, **kw):
    args = dict(stamp=t, epoch='fc-boot-1', previous_counter=0, counter=1,
                frame='map', new_from_previous=Transform((0.,0.,delta), (0.,0.,0.,1.)),
                authoritative=True)
    args.update(kw)
    return FcReset(**args)


class ContinuityTests(unittest.TestCase):
    def test_complete_point524_reset_path(self):
        b = ready()
        before = b.snapshot(1.18)
        point = b.project_pixel(100., 0., (500.,500.,0.,0.), 1.18)
        self.assertAlmostEqual(point[0], -.088)
        self.assertTrue(b.apply_reset(reset(), 1.2))
        self.assertEqual(b.generation, 1)
        self.assertFalse(b.release_allowed(permission_stamp=1.2, valid_until=2., now=1.2, generation=1))
        # One new consistent pair can produce a transformed HOLD request,
        # but cannot authorize descent, projection or release.
        pair(b, 1.21, fc_z=1.124)
        self.assertAlmostEqual(b.hold_request(1.21).xyz[2], 1.124)
        with self.assertRaises(BoundaryRejected):
            b.command(Pose(1.21, 'task', (0.,0.,.4), (0.,0.,0.,1.)), 1.21)
        with self.assertRaises(BoundaryRejected):
            b.snapshot(1.21)
        pair(b, 1.27, fc_z=1.124)
        pair(b, 1.33, fc_z=1.124)
        after = b.snapshot(1.33)
        self.assertAlmostEqual(after['body_agl'], .6)
        self.assertAlmostEqual(after['camera_agl'], .44)
        self.assertEqual(before['ground_z'], after['ground_z'])
        for a,c in zip(before['body'].xyz, after['body'].xyz): self.assertAlmostEqual(a,c)
        for a,c in zip(b.project_pixel(100.,0.,(500.,500.,0.,0.),1.33), point): self.assertAlmostEqual(a,c)
        cmd = b.command(Pose(1.33, 'task', (0.,0.,.6), (0.,0.,0.,1.)), 1.33)
        self.assertAlmostEqual(cmd.xyz[2], 1.124)
        # FC sees zero physical altitude error instead of a -0.524m descent.
        self.assertAlmostEqual(cmd.xyz[2]-b.fc.xyz[2], 0.)
        self.assertTrue(b.release_allowed(permission_stamp=1.33, valid_until=2., now=1.33, generation=1))
        self.assertFalse(b.release_allowed(permission_stamp=1.18, valid_until=2., now=1.33, generation=0))

    def test_true_ascent_is_not_reset_or_smoothed(self):
        b = ready()
        for i in range(1,11):
            t=1.18+.05*i; z=.6+.05*i
            self.assertTrue(pair(b, t, z=z))
            self.assertAlmostEqual(b.snapshot(t)['body_agl'], z)
            self.assertAlmostEqual(b.snapshot(t)['camera_agl'], z-.16)
        self.assertEqual(b.generation, 0)
        self.assertEqual(b.task_from_fc, I)

    def test_residual_without_reset_inhibits_no_remap(self):
        b=ready(); pair(b,1.2,fc_z=1.124)
        self.assertFalse(b.ready)
        self.assertEqual(b.reason, 'unexplained_fc_lio_disagreement')
        self.assertEqual(b.task_from_fc,I)
        self.assertIsNone(b.hold_request(1.2))
        with self.assertRaises(BoundaryRejected): b.command(b.task_pose,1.2)
        self.assertFalse(b.release_allowed(permission_stamp=1.2,valid_until=2.,now=1.2,generation=0))
        # A shortly delayed verified reset can resolve this soft inhibition.
        b.apply_reset(reset(),1.21)
        for t in (1.22,1.28,1.34): pair(b,t,fc_z=1.124)
        self.assertTrue(b.ready)

    def test_stale_sources_inhibit_projection_descent_release(self):
        b=ready()
        with self.assertRaises(BoundaryRejected): b.snapshot(1.5)
        with self.assertRaises(BoundaryRejected):
            b.command(Pose(1.5,'task',(0.,0.,.3),(0.,0.,0.,1.)),1.5)
        self.assertFalse(b.release_allowed(permission_stamp=1.5,valid_until=2.,now=1.5,generation=0))
        self.assertIsNone(b.hold_request(1.5))
        for t in (1.5,1.56,1.62): pair(b,t)
        self.assertTrue(b.ready)

    def test_lio_health_failure_latched(self):
        b=ready(); pair(b,1.2,health=LioHealth(1.2,'lio-boot-1','lio',False,True))
        self.assertTrue(b.fault)
        for t in (1.3,1.36,1.42): pair(b,t)
        self.assertFalse(b.ready)
        self.assertFalse(b.apply_reset(reset(t=1.42),1.42))

    def test_lio_epoch_change_not_compensated_as_fc_reset(self):
        b=ready(); pair(b,1.2,health=LioHealth(1.2,'lio-boot-2','lio',True,True))
        self.assertIn('lio_epoch_changed',b.fault)
        b.apply_reset(reset(),1.2)
        self.assertEqual(b.generation,0)
        self.assertEqual(b.task_from_fc,I)

    def test_lio_pose_jump_inhibits_without_health_epoch_change(self):
        b=ready(); pair(b,1.2,z=1.124,fc_z=.6)
        self.assertFalse(b.ready)
        self.assertEqual(b.task_from_fc,I)
        self.assertIsNone(b.hold_request(1.2))

    def test_mode_takeover_latches_and_does_not_auto_reclaim(self):
        b=ready(); pair(b,1.2,mode='POSCTL')
        for t in (1.26,1.32,1.38): pair(b,t)
        self.assertIn('manual_takeover',b.fault)
        self.assertFalse(b.ready)

    def test_fc_disconnect_latches(self):
        b=ready(); b.update_state(FcState(1.2,False,True,'OFFBOARD'))
        pair(b,1.26)
        self.assertFalse(b.ready)
        self.assertIn('disconnected',b.fault)

    def test_three_independent_samples_required_after_reset(self):
        b=ready(); b.apply_reset(reset(),1.2)
        pair(b,1.21,fc_z=1.124)
        for _ in range(10): pair(b,1.21,fc_z=1.124)
        self.assertEqual(len(b.samples),1)
        pair(b,1.37,fc_z=1.124)
        self.assertFalse(b.ready)  # sufficient span, only two samples
        pair(b,1.43,fc_z=1.124)
        self.assertTrue(b.ready)

    def test_wrong_counter_epoch_stale_event_rejected(self):
        for event in (reset(counter=2),reset(epoch='new-fc-boot'),reset(t=.7),
                      reset(authoritative=False)):
            b=ready(); b.apply_reset(event,1.2)
            self.assertEqual(b.generation,0)
            self.assertTrue(b.fault)

    def test_reset_replay_idempotent_and_conflict_rejected(self):
        b=ready(); e=reset(); b.apply_reset(e,1.2)
        tf=b.task_from_fc
        b.apply_reset(e,1.21)
        self.assertEqual(b.task_from_fc,tf)
        self.assertEqual(b.generation,1)
        b.apply_reset(replace(e,new_from_previous=I),1.21)
        self.assertEqual(b.fault,'conflicting_reset_replay')

    def test_pre_reset_measurement_cannot_use_new_reference(self):
        b=ready(); b.apply_reset(reset(),1.2)
        pair(b,1.19)
        self.assertFalse(b.ready)
        self.assertIsNone(b.hold_request(1.2))

    def test_full_rotation_translation_inverse_and_camera_mount(self):
        yaw=.4; tilt=.2
        tf=Transform((1.,-2.,.1),(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))
        mount=Transform((.05,0.,-.16),(0.,1.,0.,0.))
        b=boundary(task_from_fc=tf,body_to_camera=mount)
        for t in (1.,1.06,1.12):
            fc=Pose(t,'map',(.2,.3,.8),(0.,math.sin(tilt/2),0.,math.cos(tilt/2)))
            lio=fc.transformed(tf,'lio')
            b.observe(fc,lio,LioHealth(t,'lio-boot-1','lio',True,True),FcState(t,True,False,'POSCTL'),t)
        snap=b.snapshot(1.12)
        cmd=b.command(snap['body'],1.12)
        for a,c in zip(cmd.xyz,fc.xyz): self.assertAlmostEqual(a,c)
        expected=Transform(snap['body'].xyz,snap['body'].xyzw).compose(mount)
        self.assertEqual(snap['camera'].xyz,expected.xyz)
        self.assertAlmostEqual(snap['camera_agl'],expected.xyz[2])

    def test_reset_rotation_and_translation_compose_in_correct_order(self):
        b=ready(); rot=Transform((.1,-.2,.524),(0.,0.,math.sin(.1),math.cos(.1)))
        b.apply_reset(reset(new_from_previous=rot),1.2)
        for t in (1.21,1.27,1.33):
            old=Pose(t,'map',(0.,0.,.6),(0.,0.,0.,1.))
            new=old.transformed(rot,'map')
            b.observe(new,Pose(t,'lio',old.xyz,old.xyzw),
                      LioHealth(t,'lio-boot-1','lio',True,True),FcState(t,True,True,'OFFBOARD'),t)
        self.assertTrue(b.ready)
        cmd=b.command(b.snapshot(1.33)['body'],1.33)
        for a,c in zip(cmd.xyz,new.xyz): self.assertAlmostEqual(a,c)

    def test_calibration_default_cannot_enable_remap(self):
        b=boundary(calibration_verified=False)
        for t in (1.,1.06,1.12): pair(b,t,armed=False)
        self.assertFalse(b.ready)
        b.apply_reset(reset(),1.2)
        self.assertEqual(b.generation,0)

    def test_no_in_air_initialization(self):
        b=boundary()
        for t in (1.,1.06,1.12): pair(b,t)
        self.assertFalse(b.initialized)

    def test_timestamp_regression_latched(self):
        b=ready(); pair(b,1.17)
        self.assertEqual(b.fault,'source_time_regression')

    def test_future_unsynchronized_sources_do_not_count(self):
        b=ready()
        self.assertFalse(b.observe(Pose(2.,'map',(0.,0.,.6),(0.,0.,0.,1.)),
            Pose(1.2,'lio',(0.,0.,.6),(0.,0.,0.,1.)),
            LioHealth(1.2,'lio-boot-1','lio',True,True),FcState(1.2,True,True,'OFFBOARD'),1.2))
        self.assertFalse(b.ready)

    def test_disconnected_startup_cannot_initialize(self):
        b=boundary()
        for t in (1.,1.06,1.12):
            p=Pose(t,'map',(0.,0.,.6),(0.,0.,0.,1.))
            b.observe(p,Pose(t,'lio',p.xyz,p.xyzw),
                      LioHealth(t,'lio-boot-1','lio',True,True),
                      FcState(t,False,False,'POSCTL'),t)
        self.assertFalse(b.initialized)
        for t in (1.2,1.26,1.32): pair(b,t,armed=False,mode='POSCTL')
        self.assertTrue(b.initialized)

    def test_pre_reset_visual_evidence_cannot_create_new_permission(self):
        b=ready(); b.apply_reset(reset(),1.2)
        for t in (1.21,1.27,1.33): pair(b,t,fc_z=1.124)
        self.assertFalse(b.release_allowed(permission_stamp=1.33,evidence_stamp=1.18,
                         valid_until=2.,now=1.33,generation=1))
        self.assertTrue(b.release_allowed(permission_stamp=1.33,evidence_stamp=1.32,
                         valid_until=2.,now=1.33,generation=1))

    def test_autoland_feedback_retained_no_release_or_mode_request(self):
        b=ready(); pair(b,1.2,mode='AUTO.LAND')
        self.assertTrue(b.ready)
        self.assertFalse(b.release_allowed(permission_stamp=1.2,valid_until=2.,now=1.2,generation=0))
        with self.assertRaises(BoundaryRejected): b.command(b.task_pose,1.2)


if __name__ == '__main__': unittest.main()
