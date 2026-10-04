from pathlib import Path
import json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
H=Path(__file__).resolve().parents[3];D=H/'docs/verification/route_speed_20261004'
scenes=json.loads((D/'scenes.json').read_text());rows=[s for s in scenes if s['seed']==31 and s['variant']!='rectangle_baseline']
fig,axs=plt.subplots(1,3,figsize=(14,6),constrained_layout=True);summary=[]
step=.02;xx,yy=np.meshgrid(np.arange(-.5+step/2,7.95,step),np.arange(-5+step/2,5,step));area=(7.95+.5)*10
long=3.6*2.6/1.84;short=1.8*2.6/1.84
for ax,s in zip(axs,rows):
 mask=np.zeros_like(xx,dtype=bool);full=np.zeros_like(xx,dtype=bool)
 for a,b in zip(s['survey'],s['survey'][1:]):
  assert abs(a[0]-b[0])<1e-6 or abs(a[1]-b[1])<1e-6
  for m,margin in ((mask,0),(full,.5)):
   m|=(xx>=min(a[0],b[0])-long/2+margin)&(xx<=max(a[0],b[0])+long/2-margin)&(yy>=min(a[1],b[1])-short/2+margin)&(yy<=max(a[1],b[1])+short/2-margin)
 ax.contourf(xx,yy,mask.astype(int),levels=[-.5,.5,1.5],colors=['#fee2e2','#dbeafe'])
 ax.add_patch(Rectangle((-.5,-5),10,10,fill=False,color='black'))
 ax.plot([7.975,7.975],[-5,3.5],color='gray',lw=4)
 for x,y in s['trees']:ax.add_patch(Circle((x,y),.43,color='#166534',alpha=.65))
 path=np.asarray(s['survey']);ax.plot(path[:,0],path[:,1],'o-',color='#c2410c')
 for i,p in enumerate(path):ax.text(*p,str(i+1),fontsize=8)
 for t in s['targets']:ax.text(t['x'],t['y'],t['class'],fontsize=7)
 legal=(xx>=0)&(xx<=7.45)&(abs(yy)<=4.5)
 covered=mask.mean()*area;complete=(full&legal).sum()/legal.sum()*100
 summary.append(dict(route=s['variant'],geometric_length_m=sum(math.dist(a,b) for a,b in zip(s['survey'],s['survey'][1:])),search_area_m2=area,ideal_center_coverage_m2=covered,ideal_center_coverage_percent=100*mask.mean(),full_1m_axis_aligned_target_coverage_of_legal_centers_percent=complete))
 ax.set(xlim=(-.8,9.8),ylim=(-5.3,5.3),aspect='equal',xlabel='Forward X / m',ylabel='Left Y / m',title=s['variant']+' | %.1fm'%summary[-1]['geometric_length_m'])
 ax.grid(alpha=.2)
fig.suptitle('Clear 10x10m; camera AGL 2.6m; fixed heading; ideal coverage ignores occlusion and early interrupt')
fig.savefig(D/'route_geometry.png',dpi=160);plt.close(fig)
(D/'route_geometry.json').write_text(json.dumps(dict(lens_agl_m=2.6,fc_agl_m=2.76,footprint_m=[long,short],grid_resolution_m=step,assumptions='Measured 3.6x1.8m at FC2m, lens1.84m; nadir horizontal ground; full route; no occlusion/no detection recall guarantee; full target metric axis-aligned 1m square, not arbitrary rotation',routes=summary),indent=2))
print(json.dumps(summary,indent=2))