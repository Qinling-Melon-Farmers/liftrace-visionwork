from pathlib import Path
import json,subprocess
H=Path(__file__).resolve().parents[4];D=H/'docs/verification/latest_six_20261005';B=Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests')
rows=json.loads((H/'logs/latest_six_20261005_batch/matrix.json').read_text())['results']
for r in rows:
 run=Path(r['run']);name=f"{r['seed']}_{r['variant']}";out=D/(name+'_centers');data=run/'center_export.jsonl'
 if not data.exists():subprocess.run(['/usr/bin/python3',str(Path(__file__).parent/'center_compare.py'),'export','--bag',str(run/'vision_metrics.bag'),'--out',str(data)],check=True)
 if not out.exists():
  subprocess.run(['/home/xhj/miniconda3/envs/rl_drone/bin/python',str(Path(__file__).parent/'center_compare.py'),'analyze','--data',str(data),'--run',str(run),'--world',str(D/f"generated/{r['variant']}_{r['seed']}/{r['variant']}_seed{r['seed']}/field.world"),'--out',str(out),'--pipeline-root',str(B),'--world-to-eval-xyz','0','0','-.22','--world-to-eval-yaw','0','--transform-note','FLU world XY equals mission XY; local Z ground=-0.22, only XY distances compared','--hint-frame','camera_init','--scope-label',name],check=True)
 if not (out/'stages').exists():
  subprocess.run(['/home/xhj/miniconda3/envs/rl_drone/bin/python',str(Path(__file__).parent/'center_stages.py'),'--run',str(run),'--centers',str(out),'--data',str(data),'--truth-local-ground-z','-.22'],stdout=subprocess.DEVNULL,check=True)
 print('CENTERS DONE',name)