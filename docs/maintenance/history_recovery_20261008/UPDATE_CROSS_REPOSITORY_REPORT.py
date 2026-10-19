import pathlib,json
D=pathlib.Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests/docs/maintenance/history_recovery_20261008')
check=json.loads((D/'cross_repository_check.json').read_text())
assert check['found_in_other_repositories']==12 and check['not_found_in_checked_repositories']==0
s=json.loads((D/'summary.json').read_text())
s['cross_repository_followup']={'checked_at_utc':check['checked_at_utc'],'originally_absent_from_shared_repo':12,'found_in_navigation_repository':12,'found_in_liftrace_sim':0,'unlocated_in_specified_repositories':0,'details':'cross_repository_check.json'}
s['bundle_is_export_time_snapshot']=True
s['post_snapshot_main_not_in_bundle']=check['post_snapshot_main']
(D/'summary.json').write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
ar=json.loads((D/'archive_observations.json').read_text())
bysha={x['commit']:x for x in check['records']}
for entry in ar.values():
 for v in entry['observations']:
  if not v['exists']:
   match=bysha[v['commit']]
   v['exists_scope']='liftrace shared Git only'
   v['external_sources']=match['found_in']
   v['recovery_status']='present in navigation Git; not lost; not included by the liftrace bundle'
(D/'archive_observations.json').write_text(json.dumps(ar,ensure_ascii=False,indent=2),encoding='utf-8')
p=D/'INDEX.md'
text=p.read_text()
text=text.replace('65 个在本共享仓存在，12 个只有记录','65 个在本共享仓存在；其余 12 个经补查全部在导航 Git 存在且有 ref，未丢失')
text=text.replace('12 个缺失对象明确标 exists=false；只能说本共享仓查无该对象，不能推断所有来源仓都已丢失。本轮不跨仓扫描、fetch 或修改。',
'12 个原标 exists=false 的对象仅在视觉共享仓不存在；本次指定仓补查已全部在导航 Git 找到，并以 external_sources 补录原作者、日期和现存引用。没有联网 fetch 或修改来源仓。')
text=text.replace('12 个缺失对象逐项标 exists=false。','12 个仅在视觉共享仓不存在的对象保持 exists=false，并追加导航仓 external_sources。')
text=text.replace('12 个缺失对象明确标 exists=false；','12 个仅在视觉共享仓不存在的对象保持 exists=false；')
text=text.replace('12 个缺失对象','12 个仅在视觉共享仓不存在的对象')
# Current original wording in final report
text=text.replace('12 个缺失对象明确标 exists=false；只能说本共享仓查无该对象，不能推断所有来源仓都已丢失。本轮不跨仓扫描、fetch 或修改。',
'12 个对象仅在视觉共享仓不存在；本次指定来源仓补查已全部在导航 Git 找到，见 external_sources；未联网 fetch 或修改来源仓。')
text=text.replace('12 个仅在视觉共享仓不存在的对象明确标 exists=false；只能说本共享仓查无该对象，不能推断所有来源仓都已丢失。本轮不跨仓扫描、fetch 或修改。',
'12 个对象仅在视觉共享仓不存在；本次指定来源仓补查已全部在导航 Git 找到，见 external_sources；未联网 fetch 或修改来源仓。')
text=text.replace('后续主代理提交不会自动纳入。','后续主代理提交不会自动纳入；**此 bundle 不包含随后完成的 main@6cd0207fb5ea669416d01c06d6a1b65eac6730e9**。')
lines=['','## 2026-10-08 指定来源仓只读补查','',
'此前视觉共享 Git 查无的 **12/12 个 commit 均在 /home/xhj/liftrace-controlwork-nav 的导航 Git 中存在，且有现存 ref 承载；没有找不到的对象**。/home/xhj/liftrace-sim 中这 12 个 SHA 均不存在。这不改变视觉 bundle 的边界，导航对象未导入视觉仓，也未追加到 bundle。','',
'只检查用户指定目录。导航两个现存工作树 drop-height-board-reference-20261003、high-view-liveness-20260919 与 /home/xhj/liftrace-controlwork-nav 共用 /home/xhj/liftrace-controlwork-nav/.git，去重后查询一个导航对象库；另查一个 liftrace-sim 对象库。没有联网 fetch、push、工作树切换或新增 ref。','',
'完整查询路径、cat-file 结果、原作者/提交者/ISO 日期/父提交、直接 ref 与包含 ref 见 [cross_repository_check.json](cross_repository_check.json)。下表列的是现存包含引用（提交可能是其祖先，不把它当作 ref 的 tip）。','',
'| 完整提交 | 源 Git 仓 | 现存包含 ref |','|---|---|---|']
for v in check['records']:
 for source in v['found_in']:
  lines.append('| '+v['commit']+' | '+source['git_common_dir']+' | '+'；'.join(source['containing_refs'])+' |')
lines+=['',
'### bundle 快照边界补充','',
'现有 bundle 的导出快照时刻是 '+s['snapshot_time']+'（UTC），当时 refs/heads/main 为 '+check['post_snapshot_main']['bundle_ref_snapshot_main']+'。主代理之后完成的 **main@'+check['post_snapshot_main']['reported_commit']+' 不在此 bundle 中**，已在原独立 bundle 导入仓用 cat-file 确认对象不存在。未重建 bundle，未修改其导出 ref 清单、原大小或原验证记录。','',
'本次仅修改本报告目录；35 个保全 tags 留由主代理推送。补查完成后停止。']
p.write_text(text+'\n'.join(lines)+'\n',encoding='utf-8')
record=D/'CHANGE_RECORD.md'
with record.open('a',encoding='utf-8') as f:
 f.write('\n## 2026-10-08：指定来源仓只读补查\n\n- 日期：2026-10-08（Asia/Shanghai）。\n- 改动范围：仅本报告目录的 INDEX.md、summary.json、archive_observations.json，新增 cross_repository_check.json 与补查脚本。\n- 具体改动：对视觉共享 Git 中原查无的 12 个 SHA，在指定导航 Git 与 liftrace-sim 逐项 cat-file；记录导航工作树共用 Git、原作者日期及现存 ref；明确 bundle 不含导出后主代理 main@6cd0207fb5ea669416d01c06d6a1b65eac6730e9。\n- 验证结果：12/12 在导航 Git 存在且有包含 ref；liftrace-sim 中 0/12；指定仓中未定位数 0。独立 bundle 导入仓查无新 main commit。全部只是只读 Git 查询，无 fetch/push、tag 或来源文件修改，bundle 未重建。\n- 遗留问题：旧 bundle 是导出时刻视觉仓快照，不覆盖导航独立 Git 或其后主代理提交；不能据此声称这些历史对象丢失。\n- 下一步：本次按用户要求停止。35 tags 由主代理推送；无需本代理继续操作。\n')
# Ensure the report and machine-readable classification agree
assert '12/12' in p.read_text()
assert all(v['found_in'] and all(r['containing_refs'] for r in v['found_in']) for v in check['records'])
assert sum(not v['exists'] for d in ar.values() for v in d['observations'])==12
assert all(v.get('external_sources') for d in ar.values() for v in d['observations'] if not v['exists'])
print('Report update PASS: 12/12 located in navigation Git; only authorized report directory written; stopped.')
