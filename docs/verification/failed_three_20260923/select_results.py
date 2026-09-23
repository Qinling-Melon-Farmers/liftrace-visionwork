from pathlib import Path
import json,copy
r=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search');d=r/'docs/verification/failed_three_20260923'
first=json.loads((r/'logs/failed_three_20260923_batch/matrix.json').read_text())
repeat=json.loads((r/'logs/failed_three_20260923_seed40_repeat_batch/matrix.json').read_text())
assert first['status']==repeat['status']=='COMPLETE'
assert first['source']==repeat['source']
selected=copy.deepcopy(first)
selected['results']=[v if v['seed']!=40 else repeat['results'][0] for v in first['results']]
selected['selection_note']='User explicitly requested same-version seed40 repeat and replacement in main comparison. Original attempt retained.'
selected['superseded']=[v for v in first['results'] if v['seed']==40]
(d/'selected_matrix.json').write_text(json.dumps(selected,indent=2))
(d/'initial_matrix.json').write_text(json.dumps(first,indent=2))
(d/'seed40_repeat_matrix.json').write_text(json.dumps(repeat,indent=2))