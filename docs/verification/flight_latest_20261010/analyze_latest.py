"""Output-local analysis of the 2026-10-10 closed bag. No ROS runtime."""
from pathlib import Path
import json,csv,re,collections,bisect,html
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
root=Path("/home/xhj/liftrace-worktrees/r2026-board-vision-tests")
out=root/"试飞产物"/"analysis_flight_latest_20261010"
docs=root/"docs/verification/flight_latest_20261010"
d=json.loads((out/"data.json").read_text())
o=json.loads((out/"observations.json").read_text())
logs=json.loads((out/"rosout.json").read_text())
meta=json.loads((out/"bag_metadata.json").read_text())
def stats(v):
    a=np.asarray(v,float)
    return dict(zip(("min","p50","p95","p99","max"),map(float,np.percentile(a,[0,50,95,99,100])))) if len(a) else None
def transitions(rows,fn):
    result=[];prev=object()
    for r in rows:
        v=fn(r["m"])
        if v!=prev:result.append(dict(t=r["t"],state=v));prev=v
    return result
def decode(m):return json.loads(m["data"])
metadata=[decode(r["m"]) for r in o["/board_trials/run_metadata"]]
ground=metadata[-1]["ground_reference"]["ground_z"]
freeze=[r for r in logs if "[DropGeometry] frozen slot=" in r["m"]["msg"]][0]
f=freeze["t"]
match=re.search(r"center=\(([^,]+),([^)]+)\) FC=\(([^,]+),([^)]+)\)",freeze["m"]["msg"])
cx,cy,fx,fy=map(float,match.groups())
align=[r for r in logs if "[PatrolControl] External ALIGN" in r["m"]["msg"]][0]["t"]
motion=[]
for r in d["rows"]["odom"]:
    p=r["m"]["pose"]["pose"]
    xyz=np.array([p["position"][k] for k in ("x","y","z")])
    q=[p["orientation"][k] for k in ("x","y","z","w")]
    arm=Rotation.from_quat(q).apply([-.12,0,0])
    fc_err=float(np.linalg.norm(xyz[:2]-[fx,fy]))
    outlet_err=float(np.linalg.norm((xyz+arm)[:2]-[cx,cy]))
    motion.append(dict(t=r["t"],source_t=r["stamp"],age=r["t"]-r["stamp"],
       x=xyz[0],y=xyz[1],z=xyz[2],agl=xyz[2]-ground,fc_error=fc_err,
       outlet_error=outlet_err,both_xy=fc_err<.04 and outlet_err<.04,
       height_ok=bool(.35<=xyz[2]-ground<=.45)))
after=[r for r in motion if f<=r["t"]<221.332]
def first(cond):
    return next((r for r in after if cond(r)),None)
def snap(r):
    return {k:float(v) if isinstance(v,(float,np.floating)) else v for k,v in r.items()} if r else None
xy=first(lambda r:r["both_xy"])
joint=first(lambda r:r["both_xy"] and r["height_ok"])
height=first(lambda r:r["height_ok"])
perm=o["/mission/release_permission"]
first_perm=next((r for r in perm if r["m"]["permitted"]),None)
mission_trans=transitions(o["/navigation/mission_status"],lambda m:{k:decode(m).get(k) for k in ("phase","active_command","committed_slots")})
fc_trans=transitions(o["/mavros/state"],lambda m:{k:m[k] for k in ("armed","mode")})
ext_trans=transitions(o["/mavros/extended_state"],lambda m:{"landed_state":m["landed_state"]})
perm_trans=transitions(perm,lambda m:{"permitted":m["permitted"],"reason":m["reason"]})
ready=o["/uav_vision/drop_ready"]
ready_reasons=collections.Counter(r["m"]["reason"] for r in ready if r["t"]>=align)
valid_ev=[r for r in d["rows"]["evidence"] if r["m"].get("evidence_valid")]
counts={}
for key in ("raw","resolved","refined","mapped"):
    counts[key]=dict(messages=len(d["rows"][key]),classes=dict(collections.Counter(t["class_name"] for r in d["rows"][key] for t in r["m"]["detections"])))
def diagnostic_stats(key,t0=138.228,t1=221.332):
    vals=collections.defaultdict(list)
    for r in o[key]:
        if not t0<=r["t"]<t1:continue
        for status in r["m"]["status"]:
            for kv in status["values"]:
                try:
                    number=float(kv["value"]);vals[kv["key"]].append(number)
                except ValueError:pass
    return {k:stats(v) for k,v in vals.items()}
controller_logs=[r for r in logs if r["m"]["name"]=="/patrol_control"]
important=["[DropGeometry] waiting:","[DropGeometry] motion wait:","[DropGeometry] release_ready","Waiting for release authority"]
context=o["/uav_vision/alignment_target_context"]
ct=transitions(context,lambda m:{k:m.get(k) for k in ("active","mission_id","decision_seq","attempt","payload_slot","semantic_target_id","semantic_target_class","geometry_target_id","has_target","align_mode")})
events=[]
def add(t,stage,source,detail):
    events.append(dict(bag_seconds=round(t,6),stage=stage,source=source,detail=detail))
for tr in mission_trans:add(tr["t"],"mission","bag receipt",json.dumps(tr["state"],ensure_ascii=False))
for tr in fc_trans:add(tr["t"],"fc_state","bag receipt",json.dumps(tr["state"]))
for tr in ext_trans:add(tr["t"],"landed_state","bag receipt",json.dumps(tr["state"]))
for tr in perm_trans:
    if tr["t"]>=align:add(tr["t"],"permission","bag receipt",json.dumps(tr["state"]))
for r in logs:
    msg=re.sub(r"\x1b\[[0-9;]*m","",r["m"]["msg"])
    if "frozen slot=" in msg or "External ALIGN" in msg or "Waiting for release authority" in msg or "[DropGeometry] motion wait" in msg or "[DropGeometry] waiting:" in msg or "[DropGeometry] release_ready" in msg:
        add(r["t"],"controller",r["m"]["name"]+" / rosout receipt",msg)
for name,r in (("first_both_xy_lt_4cm",xy),("first_height_in_band",height),("first_joint_geometry_sample",joint)):
    if r:add(r["t"],name,"offline reconstruction from /navigation/local_odom",json.dumps(snap(r)))
events.sort(key=lambda r:r["bag_seconds"])
with (out/"focus_timeline.csv").open("w",newline="",encoding="utf-8") as fp:
    w=csv.DictWriter(fp,fieldnames=list(events[0]));w.writeheader();w.writerows(events)
(out/"focus_timeline.json").write_text(json.dumps(events,indent=2,ensure_ascii=False),encoding="utf-8")
with (out/"compensation_observed.csv").open("w",newline="") as fp:
    w=csv.DictWriter(fp,fieldnames=list(motion[0]));w.writeheader();w.writerows(motion)
s=dict(bag=meta,ground_z=ground,align_t=align,freeze_t=f,align_to_freeze_sec=f-align,
    freeze_log=freeze["m"]["msg"],first_both_xy_lt_4cm=snap(xy),
    first_height_035_045=snap(height),first_joint_geometry=snap(joint),
    freeze_to_joint_sec=joint["t"]-f if joint else None,
    align_to_joint_sec=joint["t"]-align if joint else None,
    joint_geometry_samples=sum(r["both_xy"] and r["height_ok"] for r in after),
    first_permission_t=first_perm["t"] if first_perm else None,
    permission_transitions=perm_trans,ready_reason_counts=dict(ready_reasons),
    valid_release_evidence_count=len(valid_ev),
    first_valid_release_evidence_t=valid_ev[0]["t"] if valid_ev else None,
    context_identity_transitions=ct,
    last_context=context[-1],
    controller_log_counts={text:sum(text in r["m"]["msg"] for r in controller_logs) for text in important},
    servo_runtime_logs=[r for r in logs if r["t"]>align and re.search(r"\bServo\b|\bACK\b|raw.*call|release_ready",r["m"]["msg"],re.I)],
    mission_transitions=mission_trans,fc_transitions=fc_trans,landed_transitions=ext_trans,
    mission_last=decode(o["/navigation/mission_status"][-1]["m"]),
    mission_last_receipt=o["/navigation/mission_status"][-1]["t"],
    last_result=o["/navigation/mission_result"][-1],detection_counts=counts,
    detector_active_diagnostics=diagnostic_stats("/uav_vision/perf"),
    lio_active_diagnostics=diagnostic_stats("/laserMapping/realtime"),
    fusion_incomplete_logs=sum("ignore incomplete fusion frame" in r["m"]["msg"] and r["t"]>align for r in logs),
    camera_images=len(d["frames"]),camera_receipt_gap_sec=stats(np.diff([r["t"] for r in d["frames"]])),
    camera_stamp_gap_sec=stats(np.diff([r["stamp"] for r in d["frames"]])),
    nav_odom_age_active=stats([r["age"] for r in motion if 138.228<=r["t"]<221.332]),
    limitations=["Frozen center and FC goal are rounded to 1 mm by rosout.",
       "Reconstructed geometry is not logged DropAlignmentFeedback or exact controller callback order.",
       "Ground reference is a stationary estimate, not independent range/landing truth.",
       "No ReleaseResult/Servo service response topic, no parcel landing ground truth, no ULog for this run located.",
       "Bag metadata deployment source dates to 2026-10-08; cannot identify updated ARM binary from that metadata alone."])
(out/"focus_summary.json").write_text(json.dumps(s,indent=2,ensure_ascii=False),encoding="utf-8")
(docs/"summary.json").write_text(json.dumps(s,indent=2,ensure_ascii=False),encoding="utf-8")
fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
t=np.array([r["t"] for r in after])
axes[0].plot(t,[r["agl"] for r in after],label="FC AGL from recorded ground reference")
axes[0].axhspan(.35,.45,color="green",alpha=.15,label="0.35-0.45 m")
axes[1].plot(t,[r["fc_error"] for r in after],label="FC to rounded compensated goal")
axes[1].plot(t,[r["outlet_error"] for r in after],label="rotated outlet to rounded frozen center")
axes[1].axhline(.04,color="red",ls="--",label="4 cm")
axes[2].step([r["t"] for r in perm if r["t"]>=f],[r["m"]["permitted"] for r in perm if r["t"]>=f],where="post",label="recorded release permission")
for ax in axes:
    ax.axvline(f,color="black",ls=":",label="freeze")
    if joint:ax.axvline(joint["t"],color="green",ls=":",label="first joint offline sample")
    ax.grid(alpha=.3);ax.legend(fontsize=8,loc="upper right")
axes[0].set_ylabel("estimated AGL (m)");axes[1].set_ylabel("horizontal error (m)");axes[2].set_ylabel("permission");axes[2].set_xlabel("bag receipt seconds")
fig.suptitle("2026-10-10 panzer slot 1: observed geometry and permission (no physical truth)")
fig.tight_layout();fig.savefig(out/"compensation_timing.png",dpi=140);fig.savefig(docs/"compensation_timing.png",dpi=140);plt.close(fig)
page='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>10月10日新增飞行重点回放</title><style>body{background:#16191e;color:#eee;max-width:1500px;margin:20px auto;font:17px sans-serif}video{width:100%}a{color:#8cf}button{padding:6px;margin:4px}table{border-collapse:collapse;width:100%}td,th{border:1px solid #555;padding:7px;vertical-align:top}</style><h1>10月10日新增 bag：对准与补偿下降</h1><p>离线原记录。未记录 drop_alignment_feedback 和 ReleaseResult；计算的FC/投口误差是按记录姿态、12cm杆臂和日志四舍五入中心重建，不能代替在线反馈或物理落点真值。坐标 Z 换算的 AGL 使用包内静止地面基准。10fps视频重复显示约5Hz原片。</p><video id="v" controls preload="metadata" src="dashboard.mp4"></video><script>function seek(t){v.currentTime=Math.max(0,t);v.play()}function sw(f){let t=v.currentTime;v.src=f;v.addEventListener('loadedmetadata',()=>v.currentTime=t,{once:true});v.load()}</script>'''
for name in ("dashboard","camera_annotated","camera_raw","trajectory"):
    page+=f'<button onclick="sw(\'{name}.mp4\')">{name}</button>'
page+='<p><a href="index.html">四视图播放器</a> · <a href="focus_timeline.csv">重点时间线</a> · <a href="focus_summary.json">数据摘要</a> · <a href="compensation_observed.csv">补偿重建明细</a> · <a href="compensation_timing.png">补偿/许可时序图</a></p><table><tr><th>秒/跳转</th><th>阶段</th><th>记录</th></tr>'
for r in events:
    page+=f'<tr><td><button onclick="seek({max(0,r["bag_seconds"]-2):.6f})">{r["bag_seconds"]:.3f}s</button></td><td>{html.escape(r["stage"])}</td><td>{html.escape(r["detail"])}<br><small>{html.escape(r["source"])}</small></td></tr>'
(out/"focus_player.html").write_text(page+"</table></html>",encoding="utf-8")
print(json.dumps({k:s[k] for k in ("align_t","freeze_t","align_to_freeze_sec","first_both_xy_lt_4cm","first_height_035_045","first_joint_geometry","freeze_to_joint_sec","align_to_joint_sec","joint_geometry_samples","first_permission_t","controller_log_counts","servo_runtime_logs","last_context","detector_active_diagnostics","lio_active_diagnostics","nav_odom_age_active")},indent=2,ensure_ascii=False))
