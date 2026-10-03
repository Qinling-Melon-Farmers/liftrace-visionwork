"""Validate report references, Gate outcomes and paired comparison arithmetic."""
from pathlib import Path
import json,re,math
D=Path(__file__).resolve().parent
def main():
    records=json.loads((D/'summary.json').read_text());assert len(records)==8
    lookup={(v['seed'],v['method']):v for v in records};assert len(lookup)==8
    for m in records:
        gate=json.loads((Path(m['run'])/'gate_status.json').read_text())
        assert gate['status']==m['status']
        if m['status']=='PASS':
            assert all(gate['checks'].values()) and len(m['commit_times'])==3
            assert math.isclose(m['completed_mission_s'],gate['metrics']['mission_ros_sec'],abs_tol=1e-8)
        else:assert m['completed_mission_s'] is None
    comparisons=json.loads((D/'comparisons.json').read_text())
    for v in comparisons['layout_validation']:assert v['same_world'] and v['same_target_layout']
    for e in comparisons['effects']:
        a=lookup[e['seed'],e['from_method']];b=lookup[e['seed'],e['to_method']]
        if a['completed_mission_s'] is None or b['completed_mission_s'] is None:assert e['full_gain_pct'] is None
        else:assert math.isclose(e['full_gain_pct'],100*(a['completed_mission_s']-b['completed_mission_s'])/a['completed_mission_s'],abs_tol=1e-8)
    page=(D/'index.html').read_text();images=re.findall(r'<img[^>]+src="([^"]+)"',page)
    assert len(images)==36 and all((D/p).is_file() for p in images)
    for link in re.findall(r'href="([^"]+)"',page):assert (D/link).is_file()
    selection=json.loads((D/'selection.json').read_text());assert selection['final_rule_qualified_selection'] is None
    result=dict(records=8,gate_pass=sum(m['status']=='PASS' for m in records),failures_preserved=sum(m['status']!='PASS' for m in records),image_links=len(images),pairing_and_arithmetic='PASS',overall='PASS')
    (D/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
