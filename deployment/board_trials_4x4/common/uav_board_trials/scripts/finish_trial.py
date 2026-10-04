#!/usr/bin/env python3
"""Summarize state and bag records without reading images or encoding video."""
from pathlib import Path
import json,sys,html
from trial_result import evaluate

def summarize(out):
    out=Path(out);latest={};mock_calls=0
    if (out/'vision_events.jsonl').exists():
        for line in (out/'vision_events.jsonl').read_text().splitlines():
            try:e=json.loads(line)
            except ValueError:continue
            latest[e['kind']]=e['data'];mock_calls+=int(e['kind']=='mock')
            if e['kind']=='mission' and e['data'].get('active_command')=='LAND':latest['landing_command']=e['data']
    supervisor=json.loads((out/'supervisor_result.json').read_text()) if (out/'supervisor_result.json').exists() else {}
    reference=json.loads((out/'ground_reference.json').read_text()) if (out/'ground_reference.json').exists() else {}
    result=evaluate(supervisor,latest,reference.get('settings',{}))
    committed=result['committed_deliveries'];result['mock_service_calls']=mock_calls
    # Compatibility for existing report readers; real service completions are not labelled mock.
    result['expected_mock_deliveries']=result['expected_deliveries'] if result['actuator_mode']=='mock' else 0
    result['committed_mock_deliveries']=committed if result['actuator_mode']=='mock' else 0
    (out/'result.json').write_text(json.dumps(result,indent=2))
    return result

def main():
    out=Path(sys.argv[1]);result=summarize(out);links=[]
    bag=out/'bag_recording.json'
    if bag.exists():
        record=json.loads(bag.read_text())
        links.append('<p>Bag '+html.escape(record['status'])+'; <a href="bag_recording.json">topic counts / missing data</a></p>')
        links.extend('<p><a href="'+html.escape(name,quote=True)+'">'+html.escape(name)+'</a></p>' for name in record.get('bags',[]))
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Board trial bag review</title><h1>Bag与任务记录</h1><p>结果：'+result['status']+'；投递确认 '+str(result['committed_deliveries'])+' 次。<a href="result.json">结果详情</a></p><p>相机图像及视觉结果仅录入bag；下载后使用tools/bag_replay合成视频。轻量事件及轨迹记录保留用于结束状态判断。</p>'+''.join(links))
    print('Trial review:',out/'index.html')
if __name__=='__main__':main()
