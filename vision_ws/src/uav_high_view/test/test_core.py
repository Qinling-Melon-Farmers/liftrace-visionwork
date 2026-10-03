from dataclasses import replace
import importlib.util
import io
import json
import math
from pathlib import Path
import unittest
from uav_high_view.core import *

EPOCH = Epoch('mission1', 'lio1', 'camera1')
WEIGHTS = dict(tent=1., pillbox=1.5, bridge=2., panzer=2.5, red_cross=10.)


def obs(i=1, cls='red_cross', t=10*NS, xy=None, **changes):
    return replace(Observation(EPOCH, Key(i, NS), t, 'camera_init', cls,
                   xy or (float(i), 0.), .9, .9, .1, 2.6), **changes)


def fill(c, i=1, cls='red_cross', start=10*NS, xy=None):
    for dt in [0, NS//4, NS//2]:
        value = obs(i, cls, start+dt, xy)
        assert c.observe(value, start+dt) == 'accepted'


class CatalogTests(unittest.TestCase):
    def setUp(self):
        self.c = Catalog(Config(), WEIGHTS); self.c.reset(EPOCH)

    def test_no_context(self):
        c = Catalog(Config(), WEIGHTS)
        self.assertEqual(c.observe(obs(), 10*NS), 'context_unavailable')

    def test_three_independent_hits(self):
        fill(self.c)
        h = self.c.hints(11*NS)[0]
        self.assertEqual((h.role, h.last_seen_ns), ('REVISIT_HINT_ONLY', 10500000000))
        self.assertEqual(self.c.hints(20*NS)[0].last_seen_ns, h.last_seen_ns)

    def test_duplicate_frames_do_not_confirm(self):
        for _ in range(10): self.c.observe(obs(), 10*NS)
        self.assertFalse(self.c.hints(10*NS))

    def test_temporally_close_not_independent(self):
        for dt in [0, 100000000, 200000000]: self.c.observe(obs(t=10*NS+dt), 10*NS+dt)
        self.assertFalse(self.c.hints(11*NS))

    def test_large_gap_requires_new_confirmation(self):
        fill(self.c); self.c.observe(obs(t=20*NS), 20*NS)
        self.assertFalse(self.c.hints(20*NS))

    def test_expired_input(self):
        self.assertEqual(self.c.observe(obs(), 11*NS), 'stale_or_future')

    def test_future_input(self):
        self.assertEqual(self.c.observe(obs(t=11*NS), 10*NS), 'stale_or_future')

    def test_stale_transform(self):
        self.assertEqual(self.c.observe(obs(transform_age_ns=NS), 10*NS), 'stale_or_future')

    def test_invalid_identity(self):
        self.assertEqual(self.c.observe(obs(key=Key(1, 20*NS)), 10*NS), 'invalid_identity')

    def test_ros_zero_target_id_is_valid(self):
        fill(self.c,i=0)
        self.assertEqual(self.c.hints(11*NS)[0].key.target_id,0)

    def test_target_id_uint32_bounds(self):
        for value in [-1,2**32,True]:
            self.assertEqual(self.c.observe(obs(key=Key(value,NS)),10*NS),'invalid_identity')

    def test_wrong_frame(self):
        self.assertEqual(self.c.observe(obs(frame='map'), 10*NS), 'frame_mismatch')

    def test_wrong_epoch(self):
        self.assertEqual(self.c.observe(obs(epoch=Epoch('other','lio1','camera1')),10*NS),'epoch_mismatch')

    def test_no_map_no_association(self):
        for key in ['map_valid', 'association_valid']:
            self.assertEqual(self.c.observe(obs(**{key:False}),10*NS),'quality_rejected')

    def test_bad_quality(self):
        for changes in [dict(quality=.1),dict(confidence=1.1),dict(uncertainty_m=0),dict(uncertainty_m=.3)]:
            self.assertEqual(self.c.observe(obs(**changes),10*NS),'quality_rejected')

    def test_nonfinite(self):
        self.assertEqual(self.c.observe(obs(xy=(math.nan,0)),10*NS),'nonfinite')

    def test_height_and_profile(self):
        self.assertEqual(self.c.observe(obs(fc_agl=1.4),10*NS),'outside_survey_height')
        self.assertEqual(self.c.observe(obs(cls='tank'),10*NS),'unsupported_class')

    def test_class_ambiguity(self):
        for dt,cls in [(0,'bridge'),(NS//4,'panzer'),(NS//2,'bridge')]:
            self.c.observe(obs(cls=cls,t=10*NS+dt),10*NS+dt)
        self.assertFalse(self.c.hints(11*NS))

    def test_spatial_spread_not_averaged_away(self):
        for dt,x in [(0,0),(NS//4,.4),(NS//2,.8)]:
            self.c.observe(obs(t=10*NS+dt,xy=(x,0)),10*NS+dt)
        self.assertFalse(self.c.hints(11*NS))

    def test_cross_id_overlap_is_ambiguous(self):
        for dt in [0,NS//4,NS//2]:
            for i in [1,2]: self.c.observe(obs(i,t=10*NS+dt,xy=(0,0)),10*NS+dt)
        self.assertFalse(self.c.hints(11*NS))

    def test_capacity_and_sample_bound(self):
        for i in range(1,17): self.c.observe(obs(i),10*NS)
        self.assertEqual(self.c.observe(obs(17),10*NS),'capacity_reached')
        for i in range(1,40): self.c.observe(obs(t=10*NS+i*NS//4),10*NS+i*NS//4)
        self.assertEqual(len(self.c.entries[Key(1,NS)].samples),8)

    def test_ttl_and_tombstone_bounded(self):
        fill(self.c); self.assertFalse(self.c.hints(80*NS))
        self.assertEqual(self.c.observe(obs(t=80*NS),80*NS),'retired_or_delivered')

    def test_reset_preserves_committed_slots(self):
        self.c.record_delivery(1,'red_cross'); fill(self.c,cls='bridge')
        self.c.reset(Epoch('mission1','lio2','camera1'))
        self.assertFalse(self.c.entries); self.assertEqual(self.c.delivered,{1:'red_cross'})

    def test_rewind_requires_new_context_and_preserves_slots(self):
        self.c.record_delivery(1,'bridge'); fill(self.c)
        self.assertFalse(self.c.tick(5*NS)); self.assertIsNone(self.c.epoch)
        self.c.reset(Epoch('mission1','lio2','camera1'))
        self.assertEqual(self.c.delivered,{1:'bridge'})

    def test_new_mission_clears_commits(self):
        self.c.record_delivery(1,'bridge'); self.c.reset(Epoch('mission2','lio2','camera1'))
        self.assertFalse(self.c.delivered)

    def test_rewind_cannot_reuse_old_epoch(self):
        fill(self.c); self.c.tick(5*NS); self.c.tick(4*NS)
        with self.assertRaises(ValueError): self.c.reset(EPOCH)

    def test_delivery_idempotence_and_no_duplicate_class(self):
        self.assertTrue(self.c.record_delivery(1,'bridge'))
        self.assertTrue(self.c.record_delivery(1,'bridge'))
        self.assertFalse(self.c.record_delivery(1,'red_cross'))
        self.assertFalse(self.c.record_delivery(2,'bridge'))

    def test_revisit_limit(self):
        fill(self.c)
        self.assertTrue(self.c.begin_revisit(Key(1,NS),11*NS))
        self.assertTrue(self.c.begin_revisit(Key(1,NS),11*NS))
        self.assertFalse(self.c.begin_revisit(Key(1,NS),11*NS))

    def test_out_of_order_slot_not_committed(self):
        self.assertFalse(self.c.record_delivery(2,'bridge'))
        self.assertFalse(self.c.delivered)

    def test_invalid_config(self):
        for change in [dict(capacity=100),dict(observations=9),dict(min_hits=1),dict(hint_ttl_ns=-1),dict(vote_fraction=math.nan)]:
            with self.assertRaises(ValueError): Config(**change)


class RouteTests(unittest.TestCase):
    def setUp(self):
        self.h = [Hint(EPOCH,Key(i,NS),cls,(i,0),.1,10*NS,1,3)
                  for i,cls in enumerate(WEIGHTS,1)]
        names=['@start']+[label(h.key) for h in self.h]+['@exit']
        self.edges=[Edge(a,b,1.,10*NS,EPOCH,True) for a in names for b in names if a!=b]
        self.costs={(label(h.key),s):2. for h in self.h for s in [1,2,3]}

    def rank(self,**kw):
        args=dict(hints=self.h,weights=WEIGHTS,edges=self.edges,epoch=EPOCH,now_ns=11*NS,
                  slots=3,service_seconds=self.costs,remaining_seconds=100.,reserve_seconds=20.)
        args.update(kw);return rank_routes(**args)

    def test_score_before_distance_and_bounded_enumeration(self):
        p=self.rank();self.assertEqual(p.weight_sum,14.5);self.assertEqual(p.permutations_checked,60)
        self.assertEqual(p.seconds,30.)

    def test_unknown_edges_not_straight_line(self):
        self.assertIsNone(self.rank(edges=[]))

    def test_blocked_and_wrong_epoch(self):
        self.assertIsNone(self.rank(edges=[replace(e,traversable=False) for e in self.edges]))
        self.assertIsNone(self.rank(epoch=Epoch('m2','l2','c2')))

    def test_stale_edges_and_hints(self):
        self.assertIsNone(self.rank(now_ns=20*NS))
        self.assertIsNone(self.rank(hints=[replace(h,last_seen_ns=NS) for h in self.h],now_ns=90*NS))

    def test_budget_accounts_for_tail_and_descent(self):
        self.assertIsNone(self.rank(remaining_seconds=29.))
        self.assertIsNone(self.rank(remaining_seconds=30.,fixed_seconds=1.))

    def test_slot_specific_service_cost(self):
        costs={**self.costs};costs[(label(self.h[-1].key),1)]=50.
        p=self.rank(service_seconds=costs)
        self.assertNotEqual(p.classes[0],'red_cross');self.assertEqual(p.weight_sum,14.5)

    def test_directed_terminal_cost_changes_order(self):
        edges=[replace(e,seconds=50.) if e.source==label(self.h[-1].key) and e.target=='@exit' else e for e in self.edges]
        p=self.rank(edges=edges);self.assertNotEqual(p.classes[-1],'red_cross')

    def test_duplicate_class_not_arbitrarily_chosen(self):
        extra=replace(self.h[-1],key=Key(16,NS),xy=(10,0))
        p=self.rank(hints=self.h+[extra]);self.assertNotIn('red_cross',p.classes)

    def test_invalid_costs_fail_closed(self):
        self.assertIsNone(self.rank(service_seconds={}))
        self.assertIsNone(self.rank(edges=[replace(e,seconds=math.nan) for e in self.edges]))

    def test_partial_slots(self):
        p=self.rank(slots=1);self.assertEqual(p.classes,('red_cross',))

    def test_duplicate_physical_key_rejected(self):
        with self.assertRaises(ValueError):
            self.rank(hints=self.h+[replace(self.h[-1],class_name='tent')])


class DecisionTests(unittest.TestCase):
    def decide(self,**kw):
        args=dict(elapsed_seconds=45.,budget_seconds=45.,proposal=None,desired_weight=14.5,
                  epoch=EPOCH,now_ns=10*NS,p0_passed=True)
        args.update(kw);return survey_decision(**args)

    def test_p0_is_default_block(self):
        self.assertEqual(self.decide(p0_passed=False),'OBSERVATION_ONLY_P0_PENDING')

    def test_budget_no_proof_no_blind_descent(self):
        self.assertEqual(self.decide(),'NO_DESCENT_EVIDENCE_HOLD_FOR_EXISTING_SUPERVISOR')

    def test_empty_pointcloud_not_descent_proof(self):
        d=DescentEvidence(EPOCH,10*NS,'empty_pointcloud',True)
        self.assertEqual(self.decide(descent=d),'NO_DESCENT_EVIDENCE_HOLD_FOR_EXISTING_SUPERVISOR')

    def test_return_verified_corridor(self):
        d=DescentEvidence(EPOCH,10*NS,'verified_transit',True)
        self.assertEqual(self.decide(return_descent=d),'PROPOSE_RETURN_VERIFIED_DESCENT')

    def test_low_search_when_no_targets(self):
        d=DescentEvidence(EPOCH,10*NS,'sensor_swept_volume',True)
        self.assertEqual(self.decide(descent=d),'PROPOSE_DESCENT_AND_LOW_SEARCH')

    def test_stale_or_wrong_epoch_proof(self):
        d=DescentEvidence(EPOCH,NS,'verified_transit',True)
        self.assertFalse(d.valid(EPOCH,10*NS))
        self.assertFalse(replace(d,stamp_ns=10*NS).valid(Epoch('m2','l2','c2'),10*NS))

    def test_continue_and_early_exit(self):
        self.assertEqual(self.decide(elapsed_seconds=20),'CONTINUE_SURVEY')
        p=RouteProposal((),(),14.5,30,1)
        d=DescentEvidence(EPOCH,10*NS,'verified_transit',True)
        self.assertEqual(self.decide(elapsed_seconds=20,proposal=p,descent=d),'PROPOSE_DESCENT_AND_REVISIT')


class ReplayTests(unittest.TestCase):
    def test_cli_cannot_promote_p0(self):
        root=Path(__file__).resolve().parents[1]
        spec=importlib.util.spec_from_file_location('replay',str(root/'scripts/high_view_replay.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        config=json.loads((root/'config/offline.json').read_text())
        stream=io.StringIO(json.dumps(dict(type='context',epoch=EPOCH.__dict__))+'\n'+json.dumps(
            dict(type='snapshot',now_ns=10*NS,remaining_seconds=600,reserve_seconds=180,
                 elapsed_seconds=45,desired_weight=14.5,p0_passed=True))+'\n')
        output=io.StringIO();module.replay(config,stream,output)
        self.assertEqual(json.loads(output.getvalue().splitlines()[-1])['decision'],'OBSERVATION_ONLY_P0_PENDING')


class EvaluationTests(unittest.TestCase):
    def segment(self):
        return dict(layout_id='one',segment_id='one',fc_agl=2.6,height_verified=True,
                    calibration_id='K1',model_revision='rknn1',
                    truth=[dict(instance_id='a',class_name='red_cross',xy=[1,2],visible=True)],
                    confirmed_predictions=[dict(matched_instance_id='a',class_name='red_cross',xy=[1.1,2])])

    def test_no_data_no_pass(self):
        from uav_high_view.evaluation import evaluate_segments
        r=evaluate_segments([],WEIGHTS)
        self.assertEqual(r['p0_status'],'NOT_DEMONSTRATED')
        self.assertIsNone(r['position_p95_m'])

    def test_confident_result_not_enough_data(self):
        from uav_high_view.evaluation import evaluate_segments
        r=evaluate_segments([self.segment()],WEIGHTS)
        self.assertEqual(r['classes']['red_cross']['visible_recall'],1)
        self.assertFalse(r['sufficient_sample_counts'])

    def test_wrong_class_and_miss(self):
        from uav_high_view.evaluation import evaluate_segments
        s=self.segment();s['confirmed_predictions'][0]['class_name']='bridge'
        r=evaluate_segments([s],WEIGHTS)
        self.assertEqual(r['wrong_class'],1)
        self.assertEqual(r['classes']['red_cross']['visible_recall'],0)

    def test_occlusion_stays_in_all_recall(self):
        from uav_high_view.evaluation import evaluate_segments
        s=self.segment();s['truth'][0]['visible']=False;s['confirmed_predictions']=[]
        r=evaluate_segments([s],WEIGHTS)['classes']['red_cross']
        self.assertEqual(r['all_recall'],0);self.assertIsNone(r['visible_recall'])

    def test_adjacent_frame_duplication_rejected(self):
        from uav_high_view.evaluation import evaluate_segments
        with self.assertRaises(ValueError):evaluate_segments([self.segment(),self.segment()],WEIGHTS)

    def test_missing_height_rejected(self):
        from uav_high_view.evaluation import evaluate_segments
        s=self.segment();s['height_verified']=False
        with self.assertRaises(ValueError):evaluate_segments([s],WEIGHTS)

    def test_wrong_height_rejected(self):
        from uav_high_view.evaluation import evaluate_segments
        s=self.segment();s['fc_agl']=1.4
        with self.assertRaises(ValueError):evaluate_segments([s],WEIGHTS)

    def test_duplicate_confirmations_do_not_dilute_errors(self):
        from uav_high_view.evaluation import evaluate_segments
        s=self.segment();s['confirmed_predictions']*=2
        with self.assertRaises(ValueError):evaluate_segments([s],WEIGHTS)

    def test_wilson_interval_not_point_estimate(self):
        from uav_high_view.evaluation import wilson
        lower,upper=wilson(20,20)
        self.assertLess(lower,.95);self.assertAlmostEqual(upper,1.)


class GeometryTests(unittest.TestCase):
    def test_bounds_and_closed_loop(self):
        from uav_high_view.routes import route_hypotheses
        routes=route_hypotheses((-4.3,4.3,0,7.1),1.3,1.2,1.8,2.6)
        self.assertEqual(routes[0].xy[0],routes[0].xy[-1])
        self.assertAlmostEqual(routes[0].length,21.4)
        for route in routes:
            self.assertEqual(route.role,'GEOMETRY_ONLY_NOT_COLLISION_CHECKED')
            self.assertTrue(all(-3.01<=x<=3.01 and 1.19<=y<=5.91 for x,y in route.xy))

    def test_no_tiny_final_gap(self):
        from uav_high_view.routes import route_hypotheses
        route=route_hypotheses((-4,4,0,7.1),1.,.1,.7,2.6)[-1]
        ys=sorted(set(y for _,y in route.xy));gaps=[b-a for a,b in zip(ys,ys[1:])]
        self.assertLess(max(gaps)-min(gaps),1e-9)
        self.assertLessEqual(max(gaps),.7)

    def test_invalid_or_unbounded_route(self):
        from uav_high_view.routes import route_hypotheses
        for args in [((-4,4,0,7),5,1,1,2.6),((-4,4,0,7),1,1,.001,2.6),((-4,4,0,7),1,1,1,4)]:
            with self.assertRaises(ValueError):route_hypotheses(*args)


if __name__=='__main__': unittest.main()
