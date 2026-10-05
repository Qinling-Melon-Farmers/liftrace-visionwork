from pathlib import Path
import json,subprocess,argparse
H=Path(__file__).resolve().parents[4];D=H/'docs/verification/seed38_resume_20261005';B=Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests')
p=argparse.ArgumentParser();p.add_argument('--matrix',type=Path,default=H/'logs/seed38_resume_20261005_batch/matrix.json');p.add_argument('--out',type=Path,default=D);a=p.parse_args();D=a.out;D.mkdir(parents=True,exist_ok=True)
rows=json.loads(a.matrix.read_text())['results']
for r in rows:
 run=Path(r['run']);name=f"{r['seed']}_{r['variant']}";out=D/(name+'_centers');data=run/'center_export.jsonl'
 if not data.exists():subprocess.run(['/usr/bin/python3',str(Path(__file__).parent/'center_compare.py'),'export','--bag',str(run/'vision_metrics.bag'),'--out',str(data)],check=True)
 if not out.exists():
  subprocess.run(['/home/xhj/miniconda3/envs/rl_drone/bin/python',str(Path(__file__).parent/'center_compare.py'),'analyze','--data',str(data),'--run',str(run),'--world',str(Path(r['scene'])/'field.world'),'--out',str(out),'--pipeline-root',str(B),'--world-to-eval-xyz','0','0','-.22','--world-to-eval-yaw','0','--transform-note','FLU world XY equals mission XY; local Z ground=-0.22, only XY distances compared','--hint-frame','camera_init','--scope-label',name],check=True)
 if not (out/'stages').exists():
  subprocess.run(['/home/xhj/miniconda3/envs/rl_drone/bin/python',str(Path(__file__).parent/'center_stages.py'),'--run',str(run),'--centers',str(out),'--data',str(data),'--truth-local-ground-z','-.22'],stdout=subprocess.DEVNULL,check=True)
 print('CENTERS DONE',name)