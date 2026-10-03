from pathlib import Path
import zipfile
root=Path(__file__).resolve().parents[3]
assets=root/'试飞产物'
out=root/'logs/flight_delivery_20260921'
out.mkdir(exist_ok=True)
for p in assets.glob('*.zip'):
    with zipfile.ZipFile(p) as z:
        items=z.infolist()
        print(p.name,len(items),sum(i.file_size for i in items))
        if p.name.startswith('corridor_diag'):
            dest=out/p.stem
            for i in items:
                target=(dest/i.filename).resolve()
                if not target.is_relative_to(dest.resolve()):raise ValueError(i.filename)
            z.extractall(dest)
            print('\n'.join(i.filename for i in items)[:18000])
        else:
            (out/'source_inventory.txt').write_text('\n'.join(i.filename for i in items))
            print('\n'.join(i.filename for i in items if any(s in i.filename for s in ('kino_replan_fsm','traj_server','corridor','patrol_control.yaml','AGENTS.md')))[:10000])
