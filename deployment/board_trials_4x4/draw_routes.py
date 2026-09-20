from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
r=Path(__file__).resolve().parent
for folder,title,route in [('01_visual_interrupt','Single visual interruption | nominal straight route',[(0,0),(.6,0),(3,0)]),('02_high_view_revisit','Full 2.6m circle | destinations come from camera memory',[(0,0),(1,-1),(3,-1),(3,1),(1,1),(1,-1)]),('03_h_landing','H landing | 1.0m approach, vertical rise to 1.8m',[(0,0),(1.3,0),(2,0)])]:
    fig,ax=plt.subplots(figsize=(8,7));ax.add_patch(Rectangle((0,-2),4,4,fill=False,lw=2));ax.plot(*zip(*route),'o--',lw=1.4,label='Nominal route, not recorded flight')
    ax.scatter(0,0,s=70,color='black',zorder=7)
    ax.annotate('',xy=(.8,0),xytext=(0,0),arrowprops=dict(arrowstyle='-|>',color='red',lw=2.5),zorder=8)
    ax.text(.4,.15,'+X / initial heading',ha='center',color='red',fontsize=10)
    ax.annotate('',xy=(0,.75),xytext=(0,0),arrowprops=dict(arrowstyle='-|>',color='green',lw=2),zorder=8)
    ax.text(.08,.75,'+Y / left',ha='left',color='green',fontsize=10)
    if folder=='01_visual_interrupt':
        ax.add_patch(Circle((2,0),.35,color='orange',alpha=.4));ax.text(2,-.6,'One target ahead\nMock drop then land here',ha='center')
    elif folder=='02_high_view_revisit':
        for xy,name in [((1.8,-.85),'bridge'),((3.,.85),'panzer'),((2.9,-.75),'red_cross')]:ax.add_patch(Circle(xy,.15,color='orange',alpha=.6));ax.text(xy[0],xy[1]+.2,name,ha='center',fontsize=8)
        ax.text(2,-1.8,'Illustrative target placements only; not stored in the planner',ha='center',fontsize=8)
    else:
        ax.add_patch(Circle((2,0),.5,fill=False,color='black'));ax.text(2,0,'H',ha='center',va='center',fontsize=20)
        ax.annotate('Fixed XY climb\n1.0 -> 1.8 m AGL',xy=(2,0),xytext=(2.25,1.05),arrowprops=dict(arrowstyle='->'))
    ax.set(xlim=(-.5,4.3),ylim=(-2.3,2.3),xlabel='Fixed initial-forward X (m)',ylabel='Fixed initial-left Y (m)',title=title);ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend(loc='upper left',fontsize=8)
    fig.tight_layout();fig.savefig(r/folder/'route.png',dpi=150);plt.close(fig)
