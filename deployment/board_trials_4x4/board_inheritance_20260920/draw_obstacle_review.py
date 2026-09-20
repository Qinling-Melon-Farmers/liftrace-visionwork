from pathlib import Path
import json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle,Polygon
D=Path(__file__).resolve().parent
data=json.loads((D/'tree_dimensions.json').read_text())
fig,axs=plt.subplots(1,2,figsize=(12,5))
for ax,goal,title in zip(axs,[(3.,0.),(2.,0.)],['A: free destination behind obstacle','B: destination inside inflated obstacle']):
 ax.add_patch(Rectangle((1.4,-.6),1.2,1.2,color='orange',alpha=.3,label='Inflated footprint: illustrative 1.2 x 1.2 m'))
 ax.add_patch(Rectangle((1.7,-.3),.6,.6,color='gray',alpha=.8,label='Hypothetical table: 0.6 x 0.6 m'))
 ax.add_patch(Circle(goal,.15,fill=False,color='red',lw=2,label='Goal adjustment radius: 0.15 m'))
 ax.scatter(*goal,color='red',marker='*',s=130);ax.scatter(.3,0,color='black',label='Start')
 ax.plot([.3,goal[0]],[0,goal[1]],'--',color='gray')
 if goal[0]==3.:
  route=np.array([[.3,0],[1.1,0],[1.2,.8],[2.8,.8],[3.,0.]])
  ax.plot(*route.T,color='blue',lw=2,label='Possible geometric detour, not a flight trace')
 else:ax.text(2.,.85,'Small goal neighborhood is occupied',ha='center',fontsize=9)
 ax.set(xlim=(0,3.5),ylim=(-1.4,1.4),xlabel='X (m)',ylabel='Y (m)',title=title);ax.set_aspect('equal');ax.grid(alpha=.2)
handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=2,fontsize=8);fig.tight_layout(rect=(0,.15,1,1));fig.savefig(D/'goal_radius_vs_detour.png',dpi=150,bbox_inches="tight");plt.close(fig)
fig,ax=plt.subplots(figsize=(7,6));h=np.array(data['with_pedestal']['horizontal_hull']);w=data['with_pedestal']['span'][0]
for margin,color,label in [(.4,'orange','High supported column: +0.40 m / side'),(.3,'cornflowerblue','Low: +0.30 m / side')]:
 # Axis-aligned bounds only; do not pretend these are a raycast map.
 lo=h.min(axis=0)-margin;span=np.ptp(h,axis=0)+2*margin
 ax.add_patch(Rectangle(lo,*span,color=color,alpha=.25,label=label+' (AABB)'))
ax.add_patch(Polygon(h,color='green',alpha=.5,label='Tree + pedestal projected convex hull'))
ax.add_patch(Rectangle((-.3,-.3),.6,.6,fill=False,color='black',label='0.60 x 0.60 m pedestal'))
ax.set(xlim=(-.95,.95),ylim=(-.95,.95),xlabel='Tree-local X (m)',ylabel='Tree-local Y (m)',title='Model reference geometry, not observed LiDAR occupancy')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8);fig.tight_layout();fig.savefig(D/'tree_footprint_inflation.png',dpi=150,bbox_inches="tight");plt.close(fig)
