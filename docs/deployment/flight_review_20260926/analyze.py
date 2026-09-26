"""Offline figures for 2026-09-26 19:12 bag. Uses existing bag_replay exports.
No ROS publishers, services, inference, or flight configuration writes.
"""
import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

PHASES = [
    ("Takeoff / initial hold", 3.841, 12.371),
    ("Outbound search", 12.371, 19.027),
    ("Approach red_cross", 19.027, 20.760),
    ("Capture / align", 20.760, 24.699),
    ("Release descent", 24.699, 27.872),
    ("Recovery", 27.872, 30.312),
    ("Resume", 30.353, 32.179),
    ("Search to x=6", 32.179, 37.530),
    ("Return home", 37.530, 57.670),
    ("Landing handoff hold", 57.670, 58.840),
    ("AUTO.LAND", 58.840, 63.844),
    ("Manual takeover", 63.844, 68.974),
]
EVENTS = [(19.027, "Red interrupt"), (27.872, "Release ACK"),
          (37.530, "Return"), (52.528, "Panzer confirmed"),
          (58.840, "AUTO.LAND"), (63.844, "POSCTL")]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", type=Path, required=True,
                    help="Directory containing replay/data.json and supplement.json")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    d = json.loads((args.run / "replay/data.json").read_text())
    extra = json.loads((args.run / "supplement.json").read_text())
    rows = d["rows"]
    a = np.array([[p["t"], *[p["m"]["pose"]["pose"]["position"][k] for k in "xyz"]]
                  for p in rows["odom"]])
    t, xyz = a[:, 0], a[:, 1:]
    q = np.array([[p["m"]["pose"]["pose"]["orientation"][k] for k in ("x","y","z","w")]
                  for p in rows["odom"]])
    yaw = np.rad2deg(np.unwrap(Rotation.from_quat(q).as_euler("xyz")[:, 2]))
    vel = np.column_stack([(np.interp(t+.25, t, xyz[:, i]) -
                             np.interp(t-.25, t, xyz[:, i]))/.5 for i in range(3)])
    vxy = np.linalg.norm(vel[:, :2], axis=1)
    sp = np.array([[p["t"], *[p["m"]["pose"]["position"][k] for k in "xyz"]]
                   for p in rows["setpoint"]])
    sq = np.array([[p["m"]["pose"]["orientation"][k] for k in ("x","y","z","w")]
                   for p in rows["setpoint"]])
    syaw = np.rad2deg(np.unwrap(Rotation.from_quat(sq).as_euler("xyz")[:, 2]))
    ground = float(np.median(xyz[t < 3.5, 2]))
    summary = {
        "bag_duration_s": d["duration"], "camera_frames": len(d["frames"]),
        "speed_method": "0.5s centered position difference, bag receipt time, not ground truth",
        "local_ground_z_m": ground, "assumed_landed_fc_clearance_m": 0.22,
        "estimated_agl_formula": "local_z - initial_ground_local_z + 0.22",
        "phase_metrics": [],
        "releases": rows["release"],
        "candidate_confirmation": {},
        "manual_takeover_confirmed_by_user": True,
        "earlier_panzer_drop_photos_are_other_flight": True,
        "latest_remote": "e94d0a7a7fb3c52d172cb29c958698a9de93b584",
        "flight_revision": "not recorded; behavior agrees with pre-flight 4e831ce",
    }
    for name, lo, hi in PHASES:
        m = (t >= lo) & (t < hi)
        summary["phase_metrics"].append(dict(
            phase=name, start_s=lo, end_s=hi, duration_s=hi-lo,
            vxy_p50_mps=float(np.median(vxy[m])),
            vxy_p95_mps=float(np.percentile(vxy[m],95)),
            local_z_min_m=float(xyz[m,2].min()), local_z_max_m=float(xyz[m,2].max())))
    for cls in ("panzer", "red_cross"):
        items = [(r["t"], z) for r in rows["targets"] for z in r["m"]["targets"]
                 if z["class_name"] == cls and z["map_valid"] and z["state"] == 2]
        summary["candidate_confirmation"][cls] = dict(
            first_t=items[0][0], first_consecutive=items[0][1]["consecutive_observe_count"],
            first_xyz=items[0][1]["map_point"], last_valid_t=items[-1][0],
            last_valid_xyz=items[-1][1]["map_point"], valid_confirmed_rows=len(items))
    m = (t >= 58.84) & (t < 63.844)
    summary["auto_land_yaw_range_deg"] = [float(yaw[m].min()),float(yaw[m].max())]
    summary["final_xy_m"] = np.median(xyz[t>75,:2],axis=0).tolist()
    summary["final_home_error_m"] = float(np.linalg.norm(summary["final_xy_m"]))
    summary["last_landing_settle"] = json.loads(
        extra["/navigation/planner_bridge_status"][-1]["m"]["data"])["landing_settle"]
    summary["selected_targets"] = {
        cls: dict(count=len(v), first_t=v[0]["t"] if v else None,
                  last_t=v[-1]["t"] if v else None)
        for cls in ("circle","panzer","red_cross")
        for v in [[r for r in rows["selected"] if r["m"]["class_name"]==cls]]}
    identities={}
    for r in rows["targets"]:
        for z in r["m"]["targets"]:
            if z["map_valid"]:
                identities[str(z["id"])]=dict(class_name=z["class_name"],xyz=z["map_point"])
    summary["last_valid_identities"]=identities
    summary["release_pose"] = {
        k:float(np.interp(rows["release"][0]["t"],t,xyz[:,i])) for i,k in enumerate("xyz")}
    summary["release_agl_estimate_m"] = summary["release_pose"]["z"]-ground+.22
    (args.out/"metrics.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False)+"\n")

    plt.rcParams.update({"font.size":10,"axes.grid":True,"grid.alpha":.22,
                         "savefig.dpi":160,"axes.spines.top":False,"axes.spines.right":False})
    fig, ax = plt.subplots(4,1,figsize=(12,10),sharex=True,layout="constrained")
    ax[0].plot(t,xyz[:,2],label="FC local Z",color="#147d92")
    ax[0].plot(sp[:,0],sp[:,3],label="Published position setpoint Z",alpha=.7,color="#df8a27")
    ax[0].axhline(ground,color="grey",ls=":",label="Initial ground local Z")
    ax[0].set_ylabel("Local Z (m)");ax[0].legend(ncol=3,loc="lower left",fontsize=8)
    ax[1].plot(t,vxy,label="XY speed",color="#245daa")
    ax[1].plot(t,vel[:,2],label="Vertical speed (signed)",alpha=.6,color="#aa4d92")
    ax[1].set_ylabel("Speed (m/s)");ax[1].legend(ncol=2,fontsize=8)
    ax[2].plot(t,yaw,label="Measured yaw",color="#147d92")
    ax[2].plot(sp[:,0],syaw,label="Published setpoint yaw",alpha=.7,color="#df8a27")
    ax[2].set_ylabel("Yaw (deg)");ax[2].legend(ncol=2,fontsize=8)
    for j, cls in enumerate(["panzer","red_cross"]):
        for key,level,color,label in [("raw",j*3,"#8ebae4","YOLO"),("mapped",j*3+1,"#db9e38","Map valid")]:
            ts=[r["t"] for r in rows[key] for z in r["m"]["detections"]
                if z["class_name"]==cls and (key!="raw" or r["m"].get("source")=="target_detector")
                and (key!="mapped" or z["map_valid"])]
            ax[3].scatter(ts,[level]*len(ts),marker="|",s=60,color=color)
        ts=[r["t"] for r in rows["targets"] for z in r["m"]["targets"]
            if z["class_name"]==cls and z["map_valid"] and z["state"]==2]
        ax[3].scatter(ts,[j*3+2]*len(ts),marker="|",s=60,color="#348360")
    ax[3].set_yticks(range(6),["Panzer YOLO","Panzer map","Panzer confirmed",
                               "Red YOLO","Red map","Red confirmed"])
    ax[3].set_xlabel("Seconds from bag start (receipt time)")
    for axis in ax:
        axis.axvspan(37.53,57.67,color="#d4e3fa",alpha=.30)
        axis.axvspan(58.84,63.844,color="#f9dddd",alpha=.45)
        axis.set_xlim(0,72)
    for i,(sec,label) in enumerate(EVENTS):
        ax[0].axvline(sec,color="black",ls=":",lw=.7)
        ax[0].text(sec,.98 if i%2==0 else .84,label,rotation=90,va="top",
                   transform=ax[0].get_xaxis_transform(),fontsize=8)
    fig.suptitle("2026-09-26 19:12 | one red-cross release, panzer confirmed on return")
    fig.savefig(args.out/"height_speed_vision.png");plt.close(fig)

    fig,ax=plt.subplots(2,1,figsize=(12,7),layout="constrained")
    colors=plt.get_cmap("tab10")
    for i,(name,lo,hi) in enumerate(PHASES[:9]):
        m=(t>=lo)&(t<hi);ax[0].plot(xyz[m,0],xyz[m,1],lw=2,color=colors(i),label=name)
    sph=sp[sp[:,0]<57.67]
    ax[0].plot(sph[:,1],sph[:,2],":",color="#b06fa7",alpha=.6,label="Published setpoint")
    for cls in ("panzer","red_cross"):
        z=summary["candidate_confirmation"][cls]["last_valid_xyz"]
        ax[0].scatter(z["x"],z["y"],marker="x",s=65,color="black")
        ax[0].annotate(cls+" (vision estimate)",(z["x"],z["y"]),xytext=(0,20),
                       textcoords="offset points",ha="center",fontsize=9)
    ax[0].scatter([0],[0],marker="*",s=100,color="black",label="Home")
    ax[0].set(xlabel="X (map, m)",ylabel="Y (map, m)",ylim=(-.85,.65),
              title="Actual trajectory by phase; static map-camera_init TF is identity")
    ax[0].legend(ncol=4,fontsize=7,loc="lower right")
    ax[0].set_aspect("equal",adjustable="datalim")
    m=(t>=57.67)&(t<=70)
    c=ax[1].scatter(xyz[m,0],xyz[m,1],c=t[m],s=8,cmap="viridis")
    ax[1].scatter([0],[0],marker="*",s=140,color="black")
    for sec,label in [(58.84,"AUTO.LAND"),(61,"Yaw rotation"),(63.844,"POSCTL"),(68.974,"On ground")]:
        p=[np.interp(sec,t,xyz[:,i]) for i in [0,1]]
        ax[1].annotate(f"{label} {sec:.2f}s",p,xytext=(10,-22 if sec<60 or sec>68 else 12),
                       textcoords="offset points",fontsize=9,arrowprops={"arrowstyle":"-"})
    ax[1].set(xlabel="X (map, m)",ylabel="Y (map, m)",title="Landing detail (localization, not external truth)")
    ax[1].axis("equal");fig.colorbar(c,ax=ax[1],label="Bag time (s)")
    fig.savefig(args.out/"trajectory_landing.png");plt.close(fig)

    # Exact original image + recorded boxes for the two panzer views.
    fig,ax=plt.subplots(1,2,figsize=(13,5),layout="constrained")
    for axis,sec,title in zip(ax,[15.472,52.52],["Outbound: only one valid panzer observation",
                                               "Return: repeated observations, then CONFIRMED"]):
        rr=min([r for r in rows["mapped"] if any(z["class_name"]=="panzer"
               for z in r["m"]["detections"])],key=lambda r:abs(r["t"]-sec))
        f=min(d["frames"],key=lambda f:abs(f["stamp"]-rr["stamp"]))
        im=cv2.imread(str(args.run/"replay"/f["file"]))
        if im is None:raise FileNotFoundError(f["file"])
        axis.imshow(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
        for z in rr["m"]["detections"]:
            if z["class_name"]!="panzer":continue
            roi=z["roi"]
            axis.add_patch(plt.Rectangle((roi["x_offset"],roi["y_offset"]),roi["width"],roi["height"],
                                         fill=False,ec="lime",lw=2))
            axis.scatter(z["center_px"]["x"],z["center_px"]["y"],c="cyan",s=40,marker="+")
            p=z["map_point"]
            caption=f"panzer {z['class_confidence']:.3f} | map ({p['x']:.2f}, {p['y']:.2f}) m"
            axis.text(15,im.shape[0]-22,caption,color="white",bbox={"facecolor":"black","alpha":.7})
        axis.set_title(f"{title}\nimage {rr['stamp']:.3f}s / received {rr['t']:.3f}s\n"
                       f"image match error {abs(f['stamp']-rr['stamp'])*1000:.1f}ms",fontsize=10)
        axis.axis("off")
    fig.savefig(args.out/"panzer_two_passes.jpg",dpi=180);plt.close(fig)
    cap=cv2.VideoCapture(str(args.run/"replay/dashboard.mp4"))
    for sec,name in [(24.7,"red_alignment.jpg"),(27.9,"red_release.jpg"),(52.8,"panzer_return.jpg"),(61.0,"landing_rotation.jpg")]:
        cap.set(cv2.CAP_PROP_POS_MSEC,sec*1000)
        ok,frame=cap.read()
        if not ok:raise RuntimeError(f"Cannot read video at {sec}")
        cv2.imwrite(str(args.out/name),frame,[cv2.IMWRITE_JPEG_QUALITY,90])
    cap.release()
    print(json.dumps({k:v for k,v in summary.items() if k not in ("phase_metrics","releases")},indent=2))

if __name__=="__main__":
    main()