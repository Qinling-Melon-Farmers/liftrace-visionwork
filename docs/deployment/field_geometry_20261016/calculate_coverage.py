from pathlib import Path
import json,shutil,subprocess,sys
OUT=Path(__file__).resolve().parent
# Regenerate both variants; the root outputs always describe the original default.
for script in (OUT/'calculate_default.py',OUT/'optional_shiftback/calculate_coverage.py'):
    subprocess.run([sys.executable,str(script)],check=True,stdout=subprocess.DEVNULL)
default=json.loads((OUT/'metrics.json').read_text(encoding='utf-8'))
optional=json.loads((OUT/'optional_shiftback/metrics.json').read_text(encoding='utf-8'))
assert default['variant']=='default' and default['enabled_by_default'] is True
assert optional['variant']=='optional_shiftback' and optional['enabled_by_default'] is False
for a,b in zip(default['results'],optional['results']):
    assert a['key']==b['key'] and a['fc_agl_m']==b['fc_agl_m']
    delta=0 if a['key']=='high_rectangle' else -1
    for pa,pb in zip(a['route'],b['route']):
        assert abs(pb[0]-pa[0]-delta)<1e-10 and pb[1]==pa[1]
for ext in ('.png','.pdf','.csv'):
    shutil.copy2(OUT/'optional_shiftback'/('comparison'+ext),OUT/('comparison_shiftback_1m'+ext))
shutil.copy2(OUT/'optional_shiftback/metrics.json',OUT/'metrics_shiftback_1m.json')
(OUT/'comparison_variants.json').write_text(json.dumps(dict(default_variant='default',optional_enabled_by_default=False,default=default,optional_shiftback=optional),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
default_report=(OUT/'report.txt').read_text(encoding='utf-8')
optional_report=(OUT/'optional_shiftback/report.txt').read_text(encoding='utf-8')
report=('两套路线覆盖报告：先默认，再可选\n'
        '默认：原牛耕/回字不平移；comparison.png及根目录metrics.json、comparison.csv均为默认。\n'
        '可选：optional_shiftback，仅选用时将两条低位路线全部点X减1.0m；不是默认启用。\n'
        '两套高位路线相同，走廊不动，低位形状、点序及连接方式不变；本任务不修改YAML。\n'
        '可选图：comparison_shiftback_1m.png；可选完整数据/脚本/分图：optional_shiftback/。\n\n'
        '一、原默认（不后移）\n'+default_report+'\n二、optional_shiftback：可选后移1m（非默认）\n'+optional_report)
(OUT/'report.txt').write_text(report,encoding='utf-8')
print('Default restored; optional_shiftback preserved and explicitly disabled by default.')
for a,b in zip(default['results'],optional['results']):
    print(a['key'],a['fc_agl_m'],a['coverage_percent'],b['coverage_percent'])
