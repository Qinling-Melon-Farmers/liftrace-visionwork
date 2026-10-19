#!/usr/bin/env python3
"""Reproduce the task/geometry/final-setpoint reset case without ROS or hardware."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'))
from uav_mission.task_frame_continuity import (
    BoundaryRejected, FcReset, FcState, LioHealth, Pose, TaskFrameBoundary, Transform)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    identity=Transform((0.,0.,0.),(0.,0.,0.,1.))
    boundary=TaskFrameBoundary(task_frame='synthetic_task',fc_frame='map',lio_frame='synthetic_lio',
        fc_epoch='synthetic_fc_boot',lio_epoch='synthetic_lio_boot',task_from_fc=identity,
        task_from_lio=identity,body_to_camera=Transform((0.,0.,-.16),(0.,1.,0.,0.)),
        ground_z=0.,calibration_verified=True,initial_reset_counter=0)
    def observe(t,fc_z=.6,lio_z=.6,armed=True):
        return boundary.observe(Pose(t,'map',(0.,0.,fc_z),(0.,0.,0.,1.)),
            Pose(t,'synthetic_lio',(0.,0.,lio_z),(0.,0.,0.,1.)),
            LioHealth(t,'synthetic_lio_boot','synthetic_lio',True,True),
            FcState(t,True,armed,'OFFBOARD' if armed else 'POSCTL'),t)
    for t in (1.,1.06,1.12): observe(t,armed=False)
    observe(1.18)
    def snapshot(t):
        snap=boundary.snapshot(t)
        command=boundary.command(Pose(t,'synthetic_task',(0.,0.,.6),(0.,0.,0.,1.)),t)
        return dict(body_agl=snap['body_agl'],camera_agl=snap['camera_agl'],
            ground_z=snap['ground_z'],fc_observed_z=boundary.fc.xyz[2],
            command_fc_z=command.xyz[2],fc_height_error=command.xyz[2]-boundary.fc.xyz[2],
            projected_ground_point=boundary.project_pixel(100.,0.,(500.,500.,0.,0.),t),
            generation=boundary.generation)
    before=snapshot(1.18)
    accepted=boundary.apply_reset(FcReset(1.2,'synthetic_fc_boot',0,1,'map',
        Transform((0.,0.,.524),(0.,0.,0.,1.)),True),1.2)
    observe(1.21,fc_z=1.124)
    hold=boundary.hold_request(1.21)
    blocked_projection=blocked_descent=False
    try: boundary.snapshot(1.21)
    except BoundaryRejected: blocked_projection=True
    try: boundary.command(Pose(1.21,'synthetic_task',(0.,0.,.3),(0.,0.,0.,1.)),1.21)
    except BoundaryRejected: blocked_descent=True
    waiting=dict(accepted_reset=accepted,ready=boundary.ready,
        projected_pose_inhibited=blocked_projection,descent_inhibited=blocked_descent,
        release_inhibited=not boundary.release_allowed(permission_stamp=1.21,valid_until=2.,
            now=1.21,generation=1),fc_hold_request=asdict(hold) if hold else None)
    observe(1.27,fc_z=1.124); observe(1.33,fc_z=1.124)
    after=snapshot(1.33)
    result=dict(case='synthetic_authoritative_fc_z_reset_0.524',observe_only=True,
        real_flight_validation=False,before=before,waiting=waiting,after=after,
        old_unmapped_setpoint_error=.6-1.124)
    assert abs(before['camera_agl']-after['camera_agl']) < 1e-12
    assert abs(after['fc_height_error']) < 1e-12
    assert blocked_projection and blocked_descent and waiting['release_inhibited']
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__': main()
