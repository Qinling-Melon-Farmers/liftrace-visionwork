from pathlib import Path
import ast, copy, sys
from types import SimpleNamespace
"""Read-only reproduction against the reviewed tree; no ROS nodes or hardware.

Assertions describe defects at runtime revision 87fb2726, not desired behavior.
After fixes, update this review separately rather than preserving these defects.
"""
root=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(root/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),str(root/'patrol_uav_ws-patrol_planner/src/uav_mission/test')]
from test_planner_execution import decision,status,BASE,NSEC
from uav_mission.planner_execution import PlannerMotionExecutor,PlannerMotionConfig
path=root/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_planner_bridge.py'
tree=ast.parse(path.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='NavigationPlannerBridge')
names={'_apply_outcome','_handle_callback_exception'}
body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
ns={'rospy':SimpleNamespace(logerr=lambda *a:None,logwarn=lambda *a:None)}
exec(compile(ast.fix_missing_locations(ast.Module(body=body,type_ignores=[])),str(path),'exec'),ns)
class Pub:
    def __init__(self): self.messages=[]
    def publish(self,m): self.messages.append(copy.deepcopy(m))
def bridge():
    return SimpleNamespace(_recovery_context_pub=Pub(),_result_pub=Pub(),_result_message=lambda e:e,_transaction=None,_landing=None,_publish_status=lambda **k:None,_output_enabled=True)
e=PlannerMotionExecutor(PlannerMotionConfig(initial_plan_timeout_ns=12*NSEC))
d=decision(deadline=BASE+60*NSEC);e.submit_decision(d,BASE)
e.apply_planner_status(status(1,8,'ACCEPTED',BASE+1),BASE+1)
out=e.tick(BASE+12*NSEC)
b=bridge();ns['_apply_outcome'](b,out)
print('initial_plan_timeout:',out.handoff,'terminal=',out.events[0].terminal,'recovery_revocations=',len(b._recovery_context_pub.messages),'original_remaining_s=',48)
assert out.handoff=='CANCEL_REQUIRED' and not b._recovery_context_pub.messages
e=PlannerMotionExecutor(PlannerMotionConfig(initial_plan_timeout_ns=12*NSEC));e.submit_decision(d,BASE)
e.apply_planner_status(status(1,8,'ACCEPTED',BASE+1),BASE+1)
e.apply_planner_status(status(2,8,'TRAJECTORY_READY',BASE+2),BASE+2)
from dataclasses import replace
out=e.apply_planner_status(replace(status(3,8,'FAILED_ATTEMPT',BASE+3*NSEC),reason='recovery_path_invalidated'),BASE+3*NSEC)
print('recovery_failure:',out.reason,'terminal=',[x.terminal for x in out.events],'handoff=',repr(out.handoff),'executor_active=',e.snapshot().active_decision_seq)
assert not out.handoff and not out.events[0].terminal
b=bridge();ns['_handle_callback_exception'](b,'timer',RuntimeError('transport_failure'))
print('adapter_fault: output_enabled=',b._output_enabled,'recovery_revocations=',len(b._recovery_context_pub.messages))
assert not b._output_enabled and not b._recovery_context_pub.messages
print('PASS: three runtime state mismatches reproduced using production methods')
