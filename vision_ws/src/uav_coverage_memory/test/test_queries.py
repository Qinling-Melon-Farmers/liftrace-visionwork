import unittest
from dataclasses import replace
import numpy as np
from uav_coverage_memory.memory import Config
from uav_coverage_memory.queries import Region,Snapshot,MissionLedger


class QueryTests(unittest.TestCase):
    def setUp(self):
        self.config=Config(-1,1,-1,1,.1,0,'map')
        self.grid=np.zeros((20,20),np.int8);self.grid[10:,:]=100
        self.snapshot=Snapshot(self.config,self.grid,10.,'round1',2)
        self.identity=dict(now=10.2,frame_id='map',epoch='round1',generation=2)

    def test_unseen_south_ranks_above_already_seen_north(self):
        rows=self.snapshot.rank([Region('north',(-1,1,0,1),5),Region('south',(-1,1,-1,0),5)],**self.identity)
        self.assertEqual(rows[0]['name'],'south')
        self.assertEqual(rows[1]['seen_estimate_fraction'],1.)
        self.assertEqual(rows[1]['observation_score'],0.)
        self.assertTrue(all(r['requires_reachability_validation'] for r in rows))

    def test_stale_frame_and_generation_fail_closed(self):
        for changes in [dict(now=12),dict(now=9),dict(frame_id='other'),dict(epoch='round2'),dict(generation=3)]:
            row=self.snapshot.query(Region('region',(-1,1,-1,1),0),**{**self.identity,**changes})
            self.assertFalse(row['accepted'])
            self.assertNotIn('seen_estimate_fraction',row)

    def test_outside_bounds_not_ranked_as_new_area(self):
        row=self.snapshot.rank([Region('outside',(-2,2,-2,2),0)],**self.identity)[0]
        self.assertFalse(row['eligible_for_ranking'])
        self.assertAlmostEqual(row['outside_grid_area_m2'],12.)

    def test_partial_cells_are_exact_and_snapshot_immutable(self):
        row=self.snapshot.query(Region('half',(-.95,.95,-.95,.95),0),**self.identity)
        self.assertAlmostEqual(row['in_grid_area_m2'],1.9**2)
        self.assertAlmostEqual(row['seen_estimate_fraction'],.5)
        self.grid.fill(100)
        row=self.snapshot.query(Region('south',(-1,1,-1,0),0),**self.identity)
        self.assertEqual(row['seen_estimate_fraction'],0.)

    def test_occlusion_is_a_view_change_hint_not_navigation_clearance(self):
        self.grid[:10]=60
        snap=Snapshot(self.config,self.grid,10.,'round1',2)
        row=snap.rank([Region('occluded',(-1,1,-1,0),0)],**self.identity)[0]
        self.assertTrue(row['needs_view_change'])
        self.assertTrue(row['requires_reachability_validation'])
        self.assertNotIn('goal',row)

    def test_invalid_schema_and_duplicate_regions(self):
        with self.assertRaises(ValueError):Snapshot(self.config,np.ones((20,20)),10.,'round1',2)
        with self.assertRaises(ValueError):Region('bad',(1,-1,0,1),0)
        region=Region('dup',(-1,1,-1,1),0)
        with self.assertRaises(ValueError):self.snapshot.rank([region,region],**self.identity)

    def test_ttl_expiry_keeps_prior_observation_separate_from_current_estimate(self):
        ledger=MissionLedger();ledger.update(self.snapshot,10.1)
        expired=Snapshot(self.config,np.zeros((20,20)),80.,'round1',2)
        view=ledger.update(expired,80.1)
        rows=view.rank([Region('north',(-1,1,0,1),5),Region('south',(-1,1,-1,0),5)],
                       now=80.1,frame_id='map',epoch='round1',generation=2)
        self.assertEqual(rows[0]['name'],'south')
        self.assertEqual(rows[1]['seen_estimate_fraction'],0)
        self.assertAlmostEqual(rows[1]['historical_estimate_area_m2'],2)
        self.assertGreater(rows[1]['observation_score'],0)

    def test_epoch_invalidates_but_view_occlusion_does_not_erase_history(self):
        for epoch,generation in [('round2',2),('round1',3)]:
            ledger=MissionLedger();ledger.update(self.snapshot,10.1)
            view=ledger.update(Snapshot(self.config,np.zeros((20,20)),11.,epoch,generation),11.1)
            self.assertFalse(view.historical_seen.any())
        ledger=MissionLedger();ledger.update(self.snapshot,10.1)
        view=ledger.update(Snapshot(self.config,np.full((20,20),60),11.,'round1',2),11.1)
        self.assertEqual(int(view.historical_seen.sum()),200)
        self.assertFalse((view.states==100).any())

    def test_ledger_does_not_accept_duplicate_or_stale_samples(self):
        ledger=MissionLedger();ledger.update(self.snapshot,10.1)
        with self.assertRaises(ValueError):ledger.update(self.snapshot,10.1)
        with self.assertRaises(ValueError):ledger.update(Snapshot(self.config,np.zeros((20,20)),9.9,'round1',2),10.2)
        with self.assertRaises(ValueError):ledger.update(Snapshot(self.config,np.zeros((20,20)),11.,'round1',2),20.)
        view=ledger.update(Snapshot(self.config,np.zeros((20,20)),5.,'round1',2),5.1)
        self.assertFalse(view.historical_seen.any())

    def test_ground_plane_change_resets_history(self):
        ledger=MissionLedger();ledger.update(self.snapshot,10.1)
        moved=Snapshot(replace(self.config,ground_z=.2),np.zeros((20,20)),11.,'round1',2)
        self.assertFalse(ledger.update(moved,11.1).historical_seen.any())


if __name__=='__main__':unittest.main()
