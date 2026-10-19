set -e
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export PYTHONDONTWRITEBYTECODE=1
cat > /tmp/recovery_review_20261006/health_cases.py <<'PY'
import importlib.util,json
from pathlib import Path
out=Path(__file__).parent
spec=importlib.util.spec_from_file_location('boundary',out/'task_frame_continuity_candidate.py')
m=importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name]=m;spec.loader.exec_module(m)
rows=[]
q=(0.,0.,0.,1.)
def init():
    b=m.TaskFrameBoundary(task_frame='task',fc_frame='map',lio_frame='camera_init',
        fc_epoch='synthetic-fc-boot',lio_epoch='synthetic-lio-epoch',
        task_from_fc=m.Transform((0.,0.,0.),q),task_from_lio=m.Transform((0.,0.,0.),q),
        body_to_camera=m.Transform((0.,0.,-.16),(0.,1.,0.,0.)),ground_z=-.22,
        calibration_verified=True,initial_reset_counter=0)
    for t in (10.,10.06,10.12):
        observe(b,t,2.78,2.78,armed=False)
    assert b.ready
    return b
def observe(b,t,fc_z,lio_z,armed=True,healthy=True):
    return b.observe(m.Pose(t,'map',(0.,0.,fc_z),q),
        m.Pose(t,'camera_init',(0.,0.,lio_z),q),
        m.LioHealth(t,'synthetic-lio-epoch','camera_init',healthy,True),
        m.FcState(t,True,armed,'OFFBOARD'),t)
b=init()
observe(b,10.20,3.18,2.78)
assert not b.ready and b.reason=='unexplained_fc_lio_disagreement'
rows.append(dict(case='FC_step_without_authoritative_event',ready=b.ready,reason=b.reason,residual=b.last_residual))
b=init()
# Healthy synthetic producers both report real ascent; no FC reset.
for i in range(1,5):
    t=10.12+.05*i;z=2.78+.08*i;observe(b,t,z,z)
s=b.snapshot(t)
assert abs(s['body'].xyz[2]-3.1)<1e-9
rows.append(dict(case='physical_ascent_with_supplied_synthetic_health',ready=b.ready,task_z=s['body'].xyz[2],body_agl=s['body_agl'],source_is_synthetic=True))
b=init();observe(b,10.20,3.18,2.78)
reset=m.FcReset(10.20,'synthetic-fc-boot',0,1,'map',m.Transform((0.,0.,.4),q),True)
assert b.apply_reset(reset,10.21) and not b.ready
assert b.hold_request(10.21) is None
assert not b.release_allowed(permission_stamp=10.12,valid_until=10.6,now=10.21,generation=0)
for t in (10.22,10.28,10.34):observe(b,t,3.18,2.78)
s=b.snapshot(t)
command=b.command(m.Pose(t,'task',(0.,0.,2.78),q),t)
assert abs(s['body'].xyz[2]-2.78)<1e-9 and abs(command.xyz[2]-3.18)<1e-9
rows.append(dict(case='authoritative_reset_with_supplied_synthetic_health',ready=b.ready,task_z=s['body'].xyz[2],fc_command_z=command.xyz[2],generation=b.generation,old_permission_allowed=False,source_is_synthetic=True))
b=init();bad=m.FcReset(10.15,'synthetic-fc-boot',0,1,'map',m.Transform((0.,0.,.4),q),False)
assert not b.apply_reset(bad,10.15)
rows.append(dict(case='untrusted_reset',ready=b.ready,reason=b.reason))
b=init();observe(b,10.20,2.78,2.78,healthy=False)
assert not b.ready
rows.append(dict(case='unhealthy_lio',ready=b.ready,reason=b.reason))
(out/'health_results.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
PY
python /tmp/recovery_review_20261006/health_cases.py
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
git status --porcelain=v1 > /tmp/recovery_review_20261006/status_after.txt
git rev-parse HEAD > /tmp/recovery_review_20261006/head_after.txt

