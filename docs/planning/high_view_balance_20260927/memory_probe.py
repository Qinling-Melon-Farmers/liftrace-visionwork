from pathlib import Path
import json,sys
R=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(R/'vision_ws/src/uav_high_view/src'))
from uav_high_view.navigation_memory import NavigationMemory
from uav_high_view.core import Epoch,Hint,Key
e=Epoch('probe','local','camera')
m=NavigationMemory(['panzer','pillbox','bridge','red_cross'],600_000_000_000)
def h(i,xy,source='bbox',n=1,t=1):
 return Hint(e,Key(i,t*1_000_000_000,source),'panzer',xy,.45,t*1_000_000_000,1.,n)
m.update([h(0,(0.,0.),t=1)],e,1_000_000_000)
m.update([h(1,(.33,3.36),t=2)],e,2_000_000_000)
visible=m.update([h(2,(3.12,-.34),'vision',3,3)],e,3_000_000_000)
rows=m.verification_hints(3_000_000_000)
out={'scenario':'synthetic sequence modeled on observed spatial alternatives; not detector rerun',
 'visible_after_new_refined':list(visible),'suspended':sorted(m.suspended),
 'retained_locations':{k:[x.xy for x in v] for k,v in rows.items()},
 'new_refined_location_retained':any(x.xy==(3.12,-.34) for x in list(visible.values())+list(rows.get('panzer',())))}
visible=m.update([h(3,(3.12,-.34),'vision',3,604)],e,604_000_000_000)
out['after_old_hint_ttl']={'visible':list(visible),'suspended':sorted(m.suspended)}
Path(__file__).with_name('memory_probe_after.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
