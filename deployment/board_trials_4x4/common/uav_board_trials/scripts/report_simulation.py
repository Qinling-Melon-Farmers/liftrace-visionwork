#!/usr/bin/env python3
"""Export board-module SITL trajectories, metrics and video review index."""
import argparse,csv,json,math,os,subprocess,sys,html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from trial_config import TRIAL_FOLDERS

def probe(path):
    if not path.exists():return None
    result=subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=width,height,nb_frames,r_frame_rate,duration','-of','json',str(path)],capture_output=True,text=True)
    try:return json.loads(result.stdout)['streams'][0]
    except (ValueError,KeyError,IndexError):return None

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[5]);ap.add_argument('--finish-videos',action='store_true');ap.add_argument('--dashboard',action='store_true');ap.add_argument('--runs-glob',default='board8_*');ap.add_argument('--output',type=Path);ap.add_argument('--trials',nargs='+',choices=list(TRIAL_FOLDERS));args=ap.parse_args()
    root=args.root;out=args.output or root/'docs/verification/board_modules_20260927';out.mkdir(parents=True,exist_ok=True)
    cases={}
    for run in sorted((root/'logs').glob(args.runs_glob)):
        path=run/'gate_status.json'
        if path.exists():
            g=json.loads(path.read_text());trial=g.get('trial')
            if trial in TRIAL_FOLDERS:cases[trial]=(run,g)
    rows=[];sections=[]
    for trial,folder in TRIAL_FOLDERS.items():
        if args.trials and trial not in args.trials:continue
        if trial not in cases:rows.append(dict(trial=trial,status='PENDING'));continue
        run,g=cases[trial];data=run/'generated';reference=json.loads((data/'ground_reference.json').read_text());scene=json.loads((data/'scene.json').read_text())
        if args.finish_videos:subprocess.run([sys.executable,str(Path(__file__).with_name('finish_recording.py')),str(data)],check=True)
        if args.dashboard:subprocess.run([sys.executable,str(Path(__file__).with_name('render_trial_replay.py')),str(data)],check=True)
        videos={name:probe(data/(name+'.mp4')) for name in ('report_replay','overview','camera_raw','camera_annotated')}
        pose=list(csv.DictReader((data/'navigation_pose.csv').open()))
        values=np.array([[float(r[k]) for k in ('t','x','y','z')] for r in pose]) if pose else np.empty((0,4))
        fig,axes=plt.subplots(2,2,figsize=(12,9));ax=axes[0,0]
        ax.add_patch(Rectangle((0,-2),4,4,fill=False,color='black',lw=1));ax.plot(0,0,'k^',label='Start; +X forward')
        for ob in scene['obstacles']:
            x,y=ob['xy']
            if 'size' in ob:
                dx,dy,_=ob['size'];ax.add_patch(Rectangle((x-dx/2,y-dy/2),dx,dy,color='gray',alpha=.5))
            else:ax.add_patch(Circle((x,y),.4,color='green',alpha=.35))
        for target in scene['targets']:
            ax.plot(target['x'],target['y'],'rx');ax.annotate(target['class_name'],(target['x'],target['y']),fontsize=8)
        runtime=__import__('yaml').safe_load((data/'runtime.yaml').read_text())
        planned=runtime['high_view_probe']['config']['survey_xy'] if reference['mode'].startswith('high') or reference['mode']=='memory_only' else [a[:2] for a in runtime['trial']['waypoints']]
        if reference['mode']=='landing':planned=[a[:2] for a in runtime['mission']['post_delivery_route']]
        ax.plot(*np.array(planned).T,'k--',alpha=.4,label='Guide route')
        row=dict(contact_count=g.get('contact',{}).get('actual_collision_count'),checks=g.get('checks',{}),final_truth_xyz=g.get('final_truth_xyz'),trial=trial,status=g['status'],committed=g.get('committed_deliveries'),expected=g.get('expected_deliveries'),mission_s=g.get('elapsed_sim_s'),reason=g.get('supervisor',{}).get('end_reason'),run=str(run.relative_to(root)),videos=videos)
        if len(values)>1:
            t=values[:,0];rel=t-t[0];xy=values[:,1:3];dt=np.diff(t);ds=np.linalg.norm(np.diff(xy,axis=0),axis=1);valid=dt>.01
            row.update(recorded_sim_s=float(rel[-1]),path_m=float(ds.sum()),max_estimated_agl_m=float(max(values[:,3])-reference['ground_z']))
            ax.plot(values[:,1],values[:,2],color='#0072b2',label='Estimated FC trajectory')
            axes[0,1].plot(rel,values[:,3]-reference['ground_z'],label='Estimated AGL');axes[0,1].set(ylabel='FC height (m)',xlabel='Recording ROS time (s)');axes[0,1].axhline(reference['settings']['low_agl'],ls='--',color='gray')
            axes[1,0].plot(rel[1:][valid],ds[valid]/dt[valid],label='Estimated');axes[1,0].axhline(.5,ls='--',color='gray');axes[1,0].set(xlabel='Recording ROS time (s)',ylabel='Estimated horizontal speed (m/s)')
            truth_path=data/'truth_pose.csv'
            if truth_path.exists():
                truth=list(csv.DictReader(truth_path.open()))
                tv=np.array([[float(r[k]) for k in ('t','x','y','z','vx','vy','vz')] for r in truth])
                if len(tv)>1:
                    ax.plot(tv[:,1],tv[:,2],color='#e69f00',alpha=.8,label='Gazebo truth')
                    axes[0,1].plot(tv[:,0]-t[0],tv[:,3],color='#e69f00',label='Truth world Z')
                    axes[1,0].plot(tv[:,0]-t[0],np.linalg.norm(tv[:,4:6],axis=1),color='#e69f00',label='Truth')
                    row['truth_path_m']=float(np.linalg.norm(np.diff(tv[:,1:3],axis=0),axis=1).sum())
                    row['truth_max_height_m']=float(tv[:,3].max())
                    row['truth_speed_p95_m_s']=float(np.percentile(np.linalg.norm(tv[:,4:6],axis=1),95))
            axes[0,1].legend(fontsize=8);axes[1,0].legend(fontsize=8)
            counts=[];times=[]
            for line in (data/'vision_events.jsonl').read_text().splitlines():
                try:e=json.loads(line)
                except ValueError:continue
                if e['kind']=='mission':times.append(e['t']-t[0]);counts.append(e['data'].get('committed_slots',0))
            axes[1,1].step(times,counts,where='post');axes[1,1].set(xlabel='Recording ROS time (s)',ylabel='Committed mock slots',ylim=(-.1,3.2),yticks=[0,1,2,3])
        ax.set(xlabel='X / forward (m)',ylabel='Y / left (m)',xlim=(-.9,4.4),ylim=(-2.4,2.4),aspect='equal');ax.legend(fontsize=8)
        for a in axes.flat:a.grid(alpha=.2)
        fig.suptitle(f'{folder}: {g["status"]} | drops {g.get("committed_deliveries",0)} | {row["reason"]}')
        fig.tight_layout();fig.savefig(out/(trial+'.png'),dpi=140);plt.close(fig)
        link=lambda name:html.escape(os.path.relpath(data/name,out).replace(os.sep,'/'))
        video_html=''.join(f'<details><summary>{n} ({v.get("duration","?")}s)</summary><video controls preload="none" src="{link(n+("_h264.mp4" if (data/(n+"_h264.mp4")).exists() else ".mp4"))}"></video></details>' for n,v in videos.items() if v)
        sections.append(f'<section><h2>{folder} — {g["status"]}</h2><p>任务启动至结束 {row["mission_s"]} ROS秒；模拟投递 {row["committed"]}/{row["expected"]}；结束原因 {row["reason"]}。<a href="{link("gate_status.json")}">验收详情</a></p><img src="{trial}.png">{video_html}</section>')
        rows.append(row)
    (out/'summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    table='\n'.join(f'| {r["trial"]} | {r["status"]} | {r.get("committed","—")}/{r.get("expected","—")} | {r.get("mission_s","—")} | {r.get("reason","—")} |' for r in rows)
    (out/'RESULTS.md').write_text('# 板端同链仿真结果\n\n'+ '| 专项 | 状态 | 投递 | 任务ROS秒 | 结束原因 |\n| --- | --- | --- | --- | --- |\n'+table+'\n\n任务计时从手动启动服务被接受开始，不包含前置自动起飞。视频和图表覆盖准备、起飞及结束；估计高度不等同于 Gazebo 真值测量。\n\n三路录像逐组检查可解码，完整入口：[index.html](index.html)。仿真使用笔记本 PyTorch 和模拟执行器，不能替代板端 RKNN/机构实投验收。\n')
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>专项仿真验收</title><style>body{font:16px sans-serif;max-width:1200px;margin:auto;padding:24px;background:#fafafa}video,img{max-width:100%;width:100%}section{background:white;padding:20px;margin:20px 0;border:1px solid #ddd}summary{cursor:pointer;padding:10px}</style><h1>板端专项：同链仿真与录像</h1><p>真实 Gazebo/PX4/定位/规划/视觉链；模拟执行器。三路录像分别播放；固定5fps编码，消息丢帧会缩短视频时长，准确时序以帧时间CSV与ROS计时为准。任务计时不包含前置起飞。图中蓝线为飞控估计；有独立真值记录的轮次另画橙线（高度为Gazebo世界Z），目标与障碍轮廓仅用于事后分析。</p>'+''.join(sections))
    print(json.dumps([dict(trial=r['trial'],status=r['status']) for r in rows],indent=2))
if __name__=='__main__':main()
