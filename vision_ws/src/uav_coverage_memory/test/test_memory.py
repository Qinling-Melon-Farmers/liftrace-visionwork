import unittest
from dataclasses import replace
import numpy as np
from uav_coverage_memory.memory import Camera, Config, Memory, State


class MemoryTests(unittest.TestCase):
    def setUp(self):
        self.config = Config(-1,1,-1,1,.1,0,'map', min_dwell=.5, image_margin_px=8)
        self.memory = Memory(self.config)
        self.camera = Camera(160,120,(100.,0,80.,0,100.,60.,0,0,1),(), 'optical')
        self.tf = np.diag([1.,-1.,-1.,1.])
        self.tf[2,3] = 2
        self.image = np.random.RandomState(4).randint(30,225,(120,160),dtype=np.uint8)
        # Sparse occupied-map fixture outside the image; explicitly only an estimate.
        self.points = np.array([[10.,10.,1.]])

    def observe(self, stamp=1., **kwargs):
        args = dict(image=self.image, camera=self.camera, camera_to_map=self.tf,
                    stamp=stamp, now=stamp+.01, tf_stamp=stamp, camera_stamp=stamp,
                    image_frame='optical', map_points=self.points, map_stamp=stamp, map_frame='map')
        args.update(kwargs)
        return self.memory.observe(**args)

    def qualify(self):
        for t in [1.,1.25,1.5]:
            result = self.observe(t)
        return result

    def test_repeated_images_required(self):
        self.assertEqual(self.observe()['estimated_seen_area_m2'],0)
        self.assertEqual(self.observe(1.25)['estimated_seen_area_m2'],0)
        self.assertGreater(self.observe(1.5)['estimated_seen_area_m2'],0)

    def test_revisit_is_not_new_area(self):
        first = self.qualify()
        again = self.observe(1.75)
        self.assertGreater(first['newly_estimated_area_m2'],0)
        self.assertEqual(again['newly_estimated_area_m2'],0)
        self.assertGreater(again['revisited_estimated_area_m2'],0)

    def test_duplicate_and_reordered_cannot_add_credit(self):
        self.observe(1.)
        for t in [1.,.9]:
            self.assertEqual(self.observe(t,now=1.11)['reason'],'duplicate_or_out_of_order_image')
        self.assertEqual(self.observe(1.25)['estimated_seen_area_m2'],0)

    def test_temporal_gap_breaks_streak(self):
        self.observe(1.);self.observe(1.25)
        self.assertEqual(self.observe(3.)['estimated_seen_area_m2'],0)

    def test_high_frame_rate_not_independent_observations(self):
        for t in [1.,1.01,1.02,1.03]:
            self.assertEqual(self.observe(t)['estimated_seen_area_m2'],0)

    def test_stale_future_and_zero_image_rejected(self):
        for stamp,now in [(1,2),(2,1),(0,1)]:
            self.memory.reset('test')
            self.assertFalse(self.observe(stamp,now=now)['accepted'])

    def test_bad_calibration_and_tf_times_rejected(self):
        for changes in [dict(camera_stamp=.1,stamp=3.,now=3.),dict(camera_stamp=2),dict(tf_stamp=.5)]:
            self.memory.reset('test')
            self.assertFalse(self.observe(**changes)['accepted'])

    def test_static_transform_stamp_zero_allowed(self):
        self.assertTrue(self.observe(tf_stamp=0)['accepted'])

    def test_frame_mismatch_rejected(self):
        self.assertEqual(self.observe(image_frame='other')['reason'],'camera_frame_mismatch')

    def test_unknown_map_never_becomes_seen(self):
        for t in [1.,1.25,1.5,1.75]:
            row = self.observe(t,map_points=None,map_stamp=None)
            self.assertEqual(row['estimated_seen_area_m2'],0)
        self.assertGreater(row['state_area_m2']['VISIBILITY_UNVERIFIED'],0)

    def test_bad_maps_are_unverified(self):
        variants=[dict(map_points=np.empty((0,3))),dict(map_stamp=0),dict(map_stamp=.1),
                  dict(map_stamp=4),dict(map_frame='other'),dict(map_points=np.array([[np.nan,0,0]])),
                  dict(map_points=np.ones((100001,3)))]
        for changes in variants:
            self.memory.reset('test')
            result=self.observe(3.,**changes)
            self.assertFalse(result['map_fresh'])
            self.assertEqual(result['estimated_seen_area_m2'],0)

    def test_map_staleness_clears_active_estimates(self):
        self.qualify()
        result=self.observe(1.75,map_stamp=None)
        self.assertEqual(result['estimated_seen_area_m2'],0)

    def test_obstacle_blocks_and_invalidates_previous_credit(self):
        self.qualify()
        before=np.isfinite(self.memory.last_seen).sum()
        result=self.observe(1.75,map_points=np.array([[0.,0.,1.]]))
        self.assertGreater(result['state_area_m2']['OCCLUDED'],0)
        self.assertLess(np.isfinite(self.memory.last_seen).sum(),before)

    def test_points_behind_camera_do_not_occlude(self):
        result=self.observe(map_points=np.array([[0.,0.,3.]]))
        self.assertEqual(result['state_area_m2']['OCCLUDED'],0)

    def test_ground_points_not_false_occluders(self):
        result=self.observe(map_points=np.array([[0.,0.,0.]]))
        self.assertEqual(result['state_area_m2']['OCCLUDED'],0)

    def test_blur_black_and_glare_do_not_qualify(self):
        for level in [0,127,255]:
            self.memory.reset('test')
            for t in [1.,1.25,1.5]:result=self.observe(t,image=np.full_like(self.image,level))
            self.assertEqual(result['estimated_seen_area_m2'],0)
            self.assertGreater(result['state_area_m2']['LOW_QUALITY'],0)

    def test_local_glare_does_not_get_credit_from_sharp_background(self):
        image=self.image.copy();image[30:90,50:110]=255
        result=self.observe(image=image)
        self.assertGreater(result['state_area_m2']['LOW_QUALITY'],0)
        self.assertGreater(result['state_area_m2']['VISIBLE_PENDING'],0)

    def test_camera_upward_never_projects_ground(self):
        matrix=np.eye(4);matrix[2,3]=2
        self.assertEqual(self.observe(camera_to_map=matrix)['footprint_area_m2'],0)

    def test_invalid_pose_rejected(self):
        for matrix in [np.zeros((4,4)),np.eye(4),np.full((4,4),np.nan)]:
            self.memory.reset('test')
            self.assertFalse(self.observe(camera_to_map=matrix)['accepted'])

    def test_distortion_changes_footprint(self):
        plain=self.observe()['footprint_area_m2']
        camera=replace(self.camera,d=(.8,0.,0.,0.,0.))
        warped=self.observe(1.25,camera=camera)['footprint_area_m2']
        self.assertLess(warped,plain)

    def test_distortion_cannot_fold_far_rays_back_into_image(self):
        camera=replace(self.camera,d=(.00586686,.0179105,-.00100641,.00147156,-.0264851))
        points=np.array([[3.8,0.,0.],[0.,3.8,0.],[0.,0.,0.]])
        uv,_=Memory._project(points,camera,self.tf)
        self.assertTrue((uv[:2] < 0).all())
        self.assertTrue((uv[2] > 0).all())

    def test_calibration_change_resets_history(self):
        self.qualify()
        result=self.observe(1.75,camera=replace(self.camera,k=(110.,0,80.,0,110.,60.,0,0,1)))
        self.assertEqual(result['reset_reason'],'calibration_changed')
        self.assertEqual(result['estimated_seen_area_m2'],0)

    def test_clock_rollback_resets_history(self):
        self.qualify()
        result=self.observe(.5)
        self.assertEqual(result['reset_reason'],'clock_rollback')
        self.assertEqual(result['estimated_seen_area_m2'],0)

    def test_epoch_reset_clears_all_layers(self):
        self.qualify();self.memory.reset('new_scene','round2')
        self.assertEqual(self.memory.summary(2)['estimated_seen_area_m2'],0)
        self.assertEqual(self.memory.epoch,'round2')
        self.assertTrue((self.memory.state==State.UNSEEN).all())

    def test_expiry_without_new_images(self):
        self.qualify()
        self.assertEqual(self.memory.summary(70)['estimated_seen_area_m2'],0)

    def test_invalid_frame_breaks_pending_streak(self):
        self.observe();self.observe(1.25,image_frame='other')
        self.assertEqual(self.observe(1.5)['estimated_seen_area_m2'],0)

    def test_partial_edge_cells_have_exact_area(self):
        m=Memory(replace(self.config,max_x=1.05,max_y=1.03))
        self.assertAlmostEqual(m.area.sum(),2.05*2.03)
        self.assertTrue((m.samples[:,:,0] < 1.05).all())

    def test_config_and_camera_validation(self):
        for kwargs in [dict(resolution=0),dict(resolution=.0001),dict(memory_ttl=-1),dict(min_observations=1),dict(quality_window_px=2),dict(max_dark_fraction=2)]:
            with self.assertRaises(ValueError):Memory(replace(self.config,**kwargs))
        for kwargs in [dict(k=(0,)*9),dict(d=(1.,2.)),dict(distortion_model='equidistant')]:
            with self.assertRaises(ValueError):replace(self.camera,**kwargs)


if __name__=='__main__':unittest.main()
