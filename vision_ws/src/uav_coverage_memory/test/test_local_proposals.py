import unittest
from dataclasses import replace
import numpy as np
from uav_coverage_memory.memory import Config
from uav_coverage_memory.queries import Snapshot
from uav_coverage_memory.local_proposals import ProposalConfig,Context,LocalProposer


class LocalProposalTests(unittest.TestCase):
    def setUp(self):
        self.cfg=Config(-5,5,-1,8,.1,0,'map')
        self.grid=np.zeros((90,100),dtype=np.int8)
        self.grid[30:]=100  # northern half has already been observed
        self.snap=Snapshot(self.cfg,self.grid,10.,'mission',2,self.grid==100)
        self.context=Context('mission',1,'SEARCH','map',(-3.,2.,1.2),10.,(4.,2.,1.2),50.)
        self.proposer=LocalProposer(ProposalConfig((-4,4,-.5,7)))

    def reply(self,batch,**changes):
        value=dict(accepted=True,request_id=batch.request_id,frame_id='map',stamp=10.12,map_stamp=10.,
                   planner_session='p1',map_revision=3,status=[0]*len(batch.segments),
                   known_free_proven=False,requires_trajectory_validation=True)
        value.update(changes);return value

    def finish(self,batch,**changes):
        return self.proposer.finish(batch,self.reply(batch,**changes),self.context,2,10.2)

    def test_seen_north_prefers_unseen_south_but_never_authorizes_flight(self):
        batch=self.proposer.prepare(self.snap,self.context,10.1)
        result=self.finish(batch)
        self.assertTrue(result['accepted']);self.assertLess(result['selected']['entry'][1],2.)
        self.assertEqual(result['nominal_goal'],list(self.context.nominal_goal))
        self.assertFalse(result['flight_authorized']);self.assertTrue(result['requires_trajectory_validation'])
        self.assertNotIn('coverage_complete',result)

    def test_blocked_south_not_selected_despite_high_gain(self):
        batch=self.proposer.prepare(self.snap,self.context,10.1)
        codes=[0]*len(batch.segments)
        for c in batch.candidates:
            if c.entry[1]<2:
                for i in c.segment_indices:codes[i]=1
        result=self.finish(batch,status=codes)
        self.assertTrue(result['accepted']);self.assertGreaterEqual(result['selected']['entry'][1],2.)

    def test_fully_blocked_or_unchecked_falls_back_without_nominal_completion(self):
        batch=self.proposer.prepare(self.snap,self.context,10.1)
        for code in [1,2,3,4]:
            result=self.finish(batch,status=[code]*len(batch.segments))
            self.assertFalse(result['accepted']);self.assertIsNone(result['selected'])

    def test_pending_delivery_return_or_height_transition_has_no_proposal(self):
        for context in [replace(self.context,command='APPROACH'),replace(self.context,command='RETURN_HOME'),
                        replace(self.context,nominal_goal=(4.,2.,.2)),replace(self.context,deadline=9.)]:
            with self.assertRaises(ValueError):self.proposer.prepare(self.snap,context,10.1)

    def test_stale_or_wrong_epoch_observation_rejected(self):
        with self.assertRaises(ValueError):self.proposer.prepare(self.snap,replace(self.context,mission_id='other'),10.1)
        with self.assertRaises(ValueError):self.proposer.prepare(self.snap,replace(self.context,pose_stamp=12),12.1)

    def test_late_or_mismatched_reply_rejected(self):
        batch=self.proposer.prepare(self.snap,self.context,10.1)
        for changes in [dict(request_id=99),dict(frame_id='other'),dict(map_stamp=8),dict(stamp=11),
                        dict(map_revision=0),dict(planner_session=''),dict(status=[0]),dict(known_free_proven=True),
                        dict(requires_trajectory_validation=False),dict(accepted=False,reason='query_budget_exhausted')]:
            self.assertFalse(self.finish(batch,**changes)['accepted'])

    def test_context_change_pose_drift_and_expiry_reject_old_reply(self):
        batch=self.proposer.prepare(self.snap,self.context,10.1)
        for context,generation,now in [(replace(self.context,decision_seq=2),2,10.2),
                (self.context,3,10.2),(replace(self.context,pose=(-2.,2.,1.2)),2,10.2),
                (replace(self.context,pose_stamp=10.6),2,10.7)]:
            row=self.proposer.finish(batch,self.reply(batch),context,generation,now)
            self.assertFalse(row['accepted'])

    def test_bounded_progress_and_segments(self):
        batch=self.proposer.prepare(self.snap,self.context,10.1)
        self.assertLessEqual(len(batch.candidates),12);self.assertLessEqual(len(batch.segments),24)
        for candidate in batch.candidates:
            self.assertLess(np.linalg.norm(np.asarray(candidate.entry[:2])-self.context.nominal_goal[:2]),7)
        for start,end in batch.segments:self.assertLessEqual(np.linalg.norm(np.asarray(end)-start),4.)

    def test_nearby_nominal_goal_remains_an_option(self):
        context=replace(self.context,nominal_goal=(-2.,1.,1.2))
        batch=self.proposer.prepare(self.snap,context,10.1)
        self.assertIn('nominal',[c.name for c in batch.candidates])

    def test_outside_bounds_not_clipped_into_a_different_goal(self):
        with self.assertRaises(ValueError):
            self.proposer.prepare(self.snap,replace(self.context,nominal_goal=(8.,2.,1.2)),10.1)

    def test_bad_or_oversized_configuration_rejected(self):
        for changes in [dict(max_candidates=3),dict(speed=0),dict(lookaheads=(1.2,)*30),dict(switch_gain_ratio=.9)]:
            with self.assertRaises(ValueError):replace(self.proposer.config,**changes)

    def test_small_score_changes_do_not_flip_selected_side(self):
        batch=self.proposer.prepare(self.snap,self.context,10.1)
        for item in batch.candidates:
            item.observation['potential_observation_area_m2']=1.
            item.observation['observation_score']=1.
        first=self.finish(batch)['selected']['name']
        alternative=next(c for c in batch.candidates if c.name!=first)
        alternative.observation['observation_score']=1.1
        self.assertEqual(self.finish(batch)['selected']['name'],first)
        alternative.observation['observation_score']=1.3
        self.assertEqual(self.finish(batch)['selected']['name'],alternative.name)

    def test_gain_is_for_entry_only_and_packet_carries_transport_context(self):
        blank=Snapshot(self.cfg,np.zeros_like(self.grid),10.,'mission',2)
        batch=self.proposer.prepare(blank,self.context,10.1)
        result=self.finish(batch)
        self.assertEqual(result['schema_version'],1);self.assertEqual(result['frame_id'],'map')
        self.assertEqual(result['command'],'SEARCH');self.assertEqual(result['gain_scope'],'ENTRY_VIEW_PROXY')
        self.assertAlmostEqual(result['selected']['entry_gain_m2'],.9*.8)


if __name__=='__main__':unittest.main()
