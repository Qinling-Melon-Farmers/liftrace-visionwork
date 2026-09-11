#!/usr/bin/env python3
"""Synthetic logic flow using native precheck and the real task/executor classes.

No ROS initialization, planner search, vehicle simulation, or hardware output.
Only compact JSON and one figure are written; no raw array/log duplication.
"""
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'vision_ws/src/uav_coverage_memory/src'))
sys.path.insert(0,str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'))
from uav_coverage_memory.memory import Config
from uav_coverage_memory.queries import Snapshot
from uav_coverage_memory.local_proposals import ProposalConfig,Context,LocalProposer
from uav_mission.local_motion import LocalMotionConfig
from uav_mission.coverage_route import CoverageRoute
from uav_mission.search_types import Waypoint
from uav_mission.profile_policy import CompetitionProfile
from uav_mission.mission_core import MissionConfig,MissionCore,ResultEvent
from uav_mission.mission_runtime import MissionRuntime
from uav_mission.planner_execution import (MotionGoal,MotionDecision,PlannerMotionConfig,
    PlannerMotionExecutor,PlannerStatusEvent,SequencedMotionGoal,OdomSample)

spec=importlib.util.spec_from_file_location('native_fixture',ROOT/'docs/verification/local_search_proposals_20260912/replay.py')
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)


def main():
    profile=CompetitionProfile('r2026',dict(tent=1.,pillbox=1.5,bridge=2.,panzer=2.5,red_cross=10.),3,3)
    task=MissionRuntime(MissionCore(profile,MissionConfig(mission_frame='map',early_return_enabled=False)),
        CoverageRoute([Waypoint(4,2,1.2),Waypoint(-4,3,1.2)],'synthetic-entry',2),
        LocalMotionConfig(enabled=True,bounds=(-4.3,4.3,0,7.1)))
    executor=PlannerMotionExecutor(PlannerMotionConfig(mission_frame='map',arrival_dwell_ns=100000000))
    pose=(-3.,2.,1.2);timeline=[];actions=[];planner_seq=0

    def record(tag,now,extra=None):
        action=task.core.active_action
        timeline.append(dict(tag=tag,time=now,route_index=task.route.current_index,
            local_active=task.local_status()['active_seq'],active_seq=action.decision_seq if action else 0,
            active_deadline=action.deadline_at if action else None,extra=extra))

    def consume(out,now):
        assert out.accepted,out.reason
        next_action=None
        for event in out.events:
            outcome=task.apply_result(ResultEvent(**asdict(event)),now,pose[:2])
            assert outcome.accepted,outcome.reason
            if outcome.action is not None:next_action=outcome.action
        return next_action

    def dispatch(action,now):
        goal=MotionGoal(action.goal.frame_id,action.goal.x,action.goal.y,action.goal.z)
        d=MotionDecision('offline-entry',action.decision_seq,int(round(action.issued_at*1e9)),
            int(round(action.deadline_at*1e9)),action.command,'r2026',goal)
        actions.append(asdict(action))
        consume(executor.submit_decision(d,int(now*1e9)),now)
        return goal

    def planner(action,name,now,attempt,allow_ignored=False):
        nonlocal planner_seq
        planner_seq+=1
        goal=MotionGoal(action.goal.frame_id,action.goal.x,action.goal.y,action.goal.z)
        sequenced=SequencedMotionGoal(action.decision_seq,goal)
        event=PlannerStatusEvent(planner_seq,action.decision_seq,name,int(now*1e9),
            sequenced,sequenced,0.,attempt,'synthetic_planner_status')
        outcome=executor.apply_planner_status(event,int(now*1e9))
        if allow_ignored:
            assert outcome.reason=='foreign_planner_goal_ignored' and not outcome.events
            assert not executor.snapshot().faulted
            return None
        return consume(outcome,now)

    def arrive(action,start):
        nonlocal pose
        pose=(action.goal.x,action.goal.y,action.goal.z)
        out=None
        for t in [start,start+.13]:
            sample=OdomSample(int(t*1e9),'map',*pose,0.,0.,0.)
            possible=consume(executor.apply_odom(sample,int(t*1e9)),t)
            if possible is not None:out=possible
        assert out is not None,'arrival did not pass the existing executor dwell'
        return out

    original=task.start('offline-entry',9.,pose[:2]).action
    dispatch(original,9.);planner(original,'ACCEPTED',9.05,0);record('nominal_dispatched',9.05)
    cells=np.zeros((90,100),np.int8);cells[25:]=100
    snapshot=Snapshot(Config(-5,5,-1,8,.1,0,'map'),cells,10.,'offline-entry',2,cells==100)
    context=Context('offline-entry',original.decision_seq,'SEARCH','map',pose,10.,(4.,2.,1.2),original.deadline_at)
    proposer=LocalProposer(ProposalConfig((-4.3,4.3,0,7.1)))
    batch=proposer.prepare(snapshot,context,10.1)
    reply=native.fixture_probe(batch)
    advice=proposer.finish(batch,reply,context,2,10.2);advice['receipt_ros']=10.2
    assert advice['accepted']
    adopted=task.adopt_local_advice(advice,dict(epoch='offline-entry',generation=2,frame_id='map',receipt_ros=10.),
                                   10.25,pose,10.2)
    assert adopted.accepted,adopted.reason
    local=adopted.action;dispatch(local,10.25);record('entry_adopted',10.25,advice['selected'])
    planner(original,'CANCELLED',10.3,0)
    planner(local,'ACCEPTED',10.31,0)
    planner(local,'PLANNING',10.32,1);planner(local,'TRAJECTORY_READY',10.4,1)
    planner(local,'TRAJECTORY_FINISHED',14.,1)
    resumed=arrive(local,14.02)
    assert task.route.current_index==0 and resumed.goal==original.goal
    assert resumed.deadline_at==original.deadline_at
    record('entry_arrived_cursor_still_zero',14.15)
    dispatch(resumed,14.15)
    # Delayed status from the superseded original generation is harmless.
    planner(original,'TRAJECTORY_FINISHED',14.2,1,allow_ignored=True)
    assert task.route.current_index==0
    record('late_original_status_ignored',14.2)
    planner(resumed,'ACCEPTED',14.3,0);planner(resumed,'PLANNING',14.31,1)
    planner(resumed,'TRAJECTORY_READY',14.4,1);planner(resumed,'TRAJECTORY_FINISHED',20.,1)
    next_nominal=arrive(resumed,20.02)
    assert task.route.current_index==1
    record('nominal_arrived_cursor_advances',20.15)
    report=dict(scope='SYNTHETIC_TASK_EXECUTOR_LOGIC_NOT_FLIGHT_OR_PLANNER_SEARCH',
        native_probe=reply,advice=advice,actions=actions,timeline=timeline,
        original_deadline=original.deadline_at,resume_deadline=resumed.deadline_at,
        next_nominal_goal=asdict(next_nominal.goal),checks=dict(entry_adopted=True,
            old_deadline_preserved=True,entry_did_not_advance_route=True,late_status_ignored=True,
            nominal_arrival_advanced=True),new_simulation_runs=0)
    (HERE/'flow.json').write_text(json.dumps(report,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(11,4.5))
    points=np.array([(-3.,2.),(local.goal.x,local.goal.y),(4.,2.)])
    axes[0].plot(points[:,0],points[:,1],'-o',color='#167d88',label='Synthetic goal sequence')
    for p,text in zip(points,['Start','Local entry','Original nominal']):
        axes[0].annotate(text,p,xytext=(0,10),textcoords='offset points',ha='center')
    axes[0].set_xlim(-4.2,5.2);axes[0].set_ylim(.4,3);axes[0].set_aspect('equal')
    axes[0].set_xlabel('Fixture x (m)');axes[0].set_ylabel('Fixture y (m)');axes[0].grid(alpha=.25)
    axes[0].set_title('Commands, not a measured flight path')
    times=[r['time'] for r in timeline];indices=[r['route_index'] for r in timeline]
    axes[1].step(times,indices,where='post',color='#167d88',lw=2)
    axes[1].scatter(times,indices,color='#167d88')
    axes[1].axvline(14.15,color='#b76532',ls='--',label='Entry arrival: cursor stays 0')
    axes[1].set_yticks([0,1]);axes[1].set_ylim(-.1,1.2);axes[1].set_xlim(8.5,21.)
    axes[1].set_xlabel('Synthetic input time (s)');axes[1].set_ylabel('Nominal route cursor')
    axes[1].set_title('Original deadline remains 99.0 s');axes[1].legend(fontsize=8,loc='upper left');axes[1].grid(alpha=.25)
    fig.suptitle('Local entry handover | native geometry check + existing task/executor classes')
    fig.text(.5,.02,'Planner status and odometry are constructed inputs. No Gazebo/PX4 run or time-saving claim.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.06,1,.94));fig.savefig(HERE/'flow.png',dpi=160);plt.close(fig)
    print(json.dumps(report['checks'],indent=2))


if __name__=='__main__':main()
