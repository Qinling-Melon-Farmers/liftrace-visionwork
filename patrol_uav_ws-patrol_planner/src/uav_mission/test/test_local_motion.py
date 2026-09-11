"""Task/lease tests for local entry adoption, with no ROS master or flight."""
import copy
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_mission_runtime import profile,candidate,result_for,release_ack
from uav_mission.coverage_route import CoverageRoute
from uav_mission.mission_core import MissionCore,MissionConfig,MissionPhase
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.local_motion import LocalMotionConfig,validate_mode
from uav_mission.search_types import Waypoint
from uav_mission.planner_execution import MotionDecision,MotionGoal,PlannerMotionExecutor,PlannerMotionConfig


POSE=(-3.,2.,1.2)
ENTRY=(-1.8,1.2,1.2)


def make_runtime(enabled=True,max_per_mission=4):
    route=CoverageRoute([Waypoint(4.,2.,1.2),Waypoint(-4.,3.,1.2)],'local-test',2)
    runtime=MissionRuntime(MissionCore(profile(),MissionConfig(early_return_enabled=False)),route,
        LocalMotionConfig(enabled=enabled,bounds=(-4.3,4.3,0,7.1),max_per_mission=max_per_mission))
    runtime.start('mission-runtime',100.,POSE[:2])
    return runtime


def proposal(action,now=101.,entry=ENTRY,pose=POSE):
    return dict(schema_version=1,accepted=True,request_id=1,mission_id='mission-runtime',
        decision_seq=action.decision_seq,command=action.command,frame_id='camera_init',
        scope='LOCAL_OBSERVATION_ADVICE_ONLY',flight_authorized=False,known_free_proven=False,
        requires_trajectory_validation=True,geometry_scope='INFLATED_CLOUD_CENTERLINE_ONLY',gain_scope='ENTRY_VIEW_PROXY',
        created=now-.1,receipt_ros=now-.05,expires_at=now+.2,pose_stamp=now-.1,source_pose=list(pose),
        generation=2,source_snapshot_stamp=now-.1,map_stamp=now-.2,map_revision=5,planner_session='planner-1',
        nominal_goal=[action.goal.x,action.goal.y,action.goal.z],
        selected=dict(name='local_0_0',entry=list(entry),exit=[entry[0]+.6,entry[1],entry[2]],
                      geometry_clear=True,segment_status=[0,0],entry_gain_m2=.5,estimated_gain_m2=.5,score=.1))


def memory(now=101.,generation=2):
    return dict(epoch='mission-runtime',generation=generation,frame_id='camera_init',receipt_ros=now-.05)


def adopt(runtime,now=101.,data=None,mem=None,pose=POSE):
    return runtime.adopt_local_advice(data or proposal(runtime.core.active_action,now,pose=pose),
        mem or memory(now),now,pose,now-.05)


def terminal(action,seq,now,status='SUCCEEDED'):
    return replace(result_for(action,seq,status=status,stage='PLANNER',terminal=True),event_stamp_ns=int(now*1e9))


class LocalMotionTest(unittest.TestCase):
    def test_disabled_preserves_nominal_dispatch(self):
        r=make_runtime(False);old=r.core.active_action
        self.assertFalse(adopt(r).accepted);self.assertIs(r.core.active_action,old)
        self.assertEqual(r.route.current_index,0)

    def test_entry_completion_does_not_complete_nominal_and_preserves_deadline(self):
        r=make_runtime();old=r.core.active_action
        local=adopt(r).action
        self.assertEqual((local.goal.x,local.goal.y,local.goal.z),ENTRY)
        self.assertEqual(local.command,'SEARCH');self.assertFalse(local.has_target)
        self.assertEqual(local.deadline_at,113.)
        resumed=r.apply_result(terminal(local,1,105),105,ENTRY[:2]).action
        self.assertEqual(resumed.command,'RESUME');self.assertEqual(resumed.goal,old.goal)
        self.assertEqual(resumed.deadline_at,old.deadline_at)
        self.assertEqual(r.route.current_index,0);self.assertEqual(r.route.failure_count,0)
        done=r.apply_result(terminal(resumed,2,110),110,(4.,2.))
        self.assertTrue(done.accepted);self.assertEqual(r.route.current_index,1)

    def test_failure_does_not_spend_nominal_failure_budget(self):
        r=make_runtime();old=r.core.active_action;local=adopt(r).action
        resumed=r.apply_result(terminal(local,1,105,'FAILED'),105,POSE[:2]).action
        self.assertEqual(r.route.failure_count,0);self.assertEqual(resumed.deadline_at,old.deadline_at)
        retry=r.apply_result(terminal(resumed,2,110,'FAILED'),110,POSE[:2]).action
        self.assertEqual(r.route.failure_count,1);self.assertEqual(r.route.current_index,0)
        r.apply_result(terminal(retry,3,120,'FAILED'),120,POSE[:2])
        self.assertEqual(r.route.current_index,1);self.assertEqual(r.route.skipped_indices,[0])

    def test_local_timeout_returns_to_original_lease(self):
        r=make_runtime();old=r.core.active_action;local=adopt(r).action
        result=r.tick(local.deadline_at+.1,POSE[:2])
        self.assertEqual(result.action.goal,old.goal);self.assertEqual(result.action.deadline_at,old.deadline_at)
        self.assertEqual(r.route.failure_count,0);self.assertEqual(r.local_status()['active_seq'],0)

    def test_delayed_result_after_original_deadline_counts_one_nominal_failure(self):
        r=make_runtime();old=r.core.active_action;local=adopt(r).action
        result=r.apply_result(terminal(local,1,191,'FAILED'),191,POSE[:2])
        self.assertTrue(result.accepted);self.assertEqual(r.route.failure_count,1)
        self.assertEqual(result.action.goal,old.goal);self.assertGreater(result.action.deadline_at,191)

    def test_retired_nominal_and_local_successes_do_not_move_cursor(self):
        r=make_runtime();old=r.core.active_action;local=adopt(r).action
        self.assertFalse(r.apply_result(terminal(old,1,102),102,POSE[:2]).accepted)
        resumed=r.apply_result(terminal(local,1,105),105,ENTRY[:2]).action
        self.assertFalse(r.apply_result(terminal(local,2,106),106,ENTRY[:2]).accepted)
        self.assertIs(r.core.active_action,resumed);self.assertEqual(r.route.current_index,0)

    def test_each_nominal_waypoint_and_mission_have_bounded_adoptions(self):
        r=make_runtime(max_per_mission=1);local=adopt(r).action
        resumed=r.apply_result(terminal(local,1,105),105,ENTRY[:2]).action
        self.assertEqual(adopt(r,106,pose=ENTRY).reason,'local_waypoint_budget_used')
        r.apply_result(terminal(resumed,2,110),110,(4.,2.))
        self.assertEqual(adopt(r,111,pose=(4.,2.,1.2)).reason,'local_mission_budget_used')

    def test_target_waiting_at_adoption_has_priority(self):
        r=make_runtime();r.ingest([candidate(now=100.9)],100.9)
        out=adopt(r)
        self.assertEqual(out.action.command,'APPROACH');self.assertEqual(r.local_status()['used'],0)

    def test_delivery_interrupt_clears_local_lease_and_resumes_nominal_normally(self):
        r=make_runtime();old=r.core.active_action;adopt(r)
        r.ingest([candidate(now=102)],102)
        target=r.tick(102.1,POSE[:2]).action
        self.assertEqual(target.command,'APPROACH');self.assertEqual(r.local_status()['active_seq'],0)
        r.apply_result(replace(release_ack(target,1),event_stamp_ns=int(103e9)),103,(1.,0.))
        event=replace(result_for(target,2,status='SUCCEEDED',stage='RECOVERY',terminal=True),event_stamp_ns=int(104e9))
        resumed=r.apply_result(event,104,(1.,0.)).action
        self.assertEqual(resumed.goal,old.goal);self.assertEqual(resumed.deadline_at,194.)
        self.assertEqual(r.core.committed_slots,1);self.assertEqual(r.route.current_index,0)
        self.assertEqual(r.local_status()['used'],1)

    def test_mission_deadline_and_abort_clear_local_state(self):
        for abort in [False,True]:
            r=make_runtime();adopt(r)
            out=r.abort('manual_test',102) if abort else r.tick(700,POSE[:2])
            self.assertEqual(out.action.command,'ABORT' if abort else 'RETURN_HOME')
            self.assertEqual(r.local_status()['active_seq'],0)

    def test_bad_advice_is_rejected_without_retiring_the_original_goal(self):
        variants=[dict(schema_version=0),dict(decision_seq=99),dict(frame_id='other'),dict(generation=3),
                  dict(created=99),dict(expires_at=100),dict(map_stamp=95),dict(flight_authorized=True),
                  dict(gain_scope='WHOLE_EXIT_LEG'),dict(source_pose=[0,0,1.2]),dict(nominal_goal=[3,2,1.2])]
        for change in variants:
            r=make_runtime();old=r.core.active_action;data=proposal(old);data.update(change)
            self.assertFalse(adopt(r,data=data).accepted);self.assertIs(r.core.active_action,old)
            self.assertEqual(r.route.active.decision_seq,old.decision_seq)
        for change in [dict(entry_gain_m2=0),dict(segment_status=[1,0]),dict(entry=[-1.8,1.2,1.3]),
                       dict(entry=[-3.9,4.,1.2]),dict(entry=[10,2,1.2]),dict(entry=[float('nan'),2,1.2])]:
            r=make_runtime();old=r.core.active_action;data=proposal(old);data['selected'].update(change)
            self.assertFalse(adopt(r,data=data).accepted);self.assertIs(r.core.active_action,old)

    def test_new_memory_generation_and_old_pose_reject_adoption(self):
        r=make_runtime();self.assertFalse(adopt(r,mem=memory(generation=3)).accepted)
        self.assertFalse(r.adopt_local_advice(proposal(r.core.active_action),memory(),101,POSE,99).accepted)

    def test_near_deadline_and_height_transition_are_not_interrupted(self):
        r=make_runtime();self.assertFalse(adopt(r,180).accepted)
        r=make_runtime();self.assertFalse(adopt(r,pose=(-3,2,2.2)).accepted)

    def test_adjusted_arrival_is_not_reported_as_reaching_entry(self):
        r=make_runtime();local=adopt(r).action
        r.apply_result(terminal(local,1,105),105,(-1.8,2.2))
        self.assertEqual(r.local_status()['last_outcome']['reason'],'local_entry_adjusted_or_missed')
        self.assertEqual(r.route.current_index,0)

    def test_concurrent_advice_adopts_only_one_entry(self):
        r=make_runtime();data=proposal(r.core.active_action)
        with ThreadPoolExecutor(max_workers=4) as pool:
            outcomes=list(pool.map(lambda _:adopt(r,data=data),range(8)))
        self.assertEqual(sum(o.action is not None for o in outcomes),1)
        self.assertEqual(r.local_status()['used'],1)

    def test_immediate_failure_has_unique_transport_stamp_and_unchanged_deadline(self):
        r=make_runtime();original=r.core.active_action;local=adopt(r).action
        resumed=r.apply_result(terminal(local,1,101,'FAILED'),101,POSE[:2]).action
        self.assertGreater(resumed.issued_at,local.issued_at)
        self.assertEqual(resumed.deadline_at,original.deadline_at)
        executor=PlannerMotionExecutor(PlannerMotionConfig())
        for a in (original,local,resumed):
            goal=MotionGoal(a.goal.frame_id,a.goal.x,a.goal.y,a.goal.z)
            d=MotionDecision('mission-runtime',a.decision_seq,int(round(a.issued_at*1e9)),
                int(round(a.deadline_at*1e9)),a.command,'r2026',goal)
            out=executor.submit_decision(d,int(101e9))
            self.assertTrue(out.accepted,out.reason)

    def test_simulation_and_full_mode_guard(self):
        validate_mode(False,False,'full');validate_mode(True,True,'full')
        for enabled,sim,mode in [(True,False,'full'),(True,True,'post_delivery'),('true',True,'full')]:
            with self.assertRaises(ValueError):validate_mode(enabled,sim,mode)


if __name__=='__main__':unittest.main()
