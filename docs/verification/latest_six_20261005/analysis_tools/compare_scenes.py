from pathlib import Path
import json,xml.etree.ElementTree as E
H=Path(__file__).resolve().parents[4];D=H/'docs/verification/latest_six_20261005';old=H/'docs/verification/route_speed_20261004'
checks=[]
for x in json.loads((D/'scenes.json').read_text()):
 a=Path(x['scene'])/'field.world';b=old/f"{x['variant']}_seed{x['seed']}/field.world"
 def norm(p):
  w=E.parse(p).getroot().find('world');return [(e.tag,E.tostring(e,encoding='unicode')) for e in w if e.tag in ['include','model']]
 checks.append(dict(seed=x['seed'],variant=x['variant'],scene_models_equal=norm(a)==norm(b)))
(D/'scene_comparison.json').write_text(json.dumps(checks,indent=2));print(checks)