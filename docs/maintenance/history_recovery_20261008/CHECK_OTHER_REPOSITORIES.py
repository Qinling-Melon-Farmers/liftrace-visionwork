import json,pathlib,subprocess,datetime,os
D=pathlib.Path('/home/xhj/liftrace-worktrees/r2026-board-vision-tests/docs/maintenance/history_recovery_20261008')
R=pathlib.Path('/home/xhj/liftrace')
env=dict(os.environ,GIT_OPTIONAL_LOCKS='0',GIT_NO_LAZY_FETCH='1')
def git(p,*args):
 return subprocess.run(['git','--no-optional-locks','-C',str(p),*args],env=env,capture_output=True,text=True)
ar=json.loads((D/'archive_observations.json').read_text())
missing=sorted({v['commit'] for d in ar.values() for v in d['observations'] if not v['exists']})
assert len(missing)==12
paths=[pathlib.Path('/home/xhj/liftrace-controlwork-nav')]
base=pathlib.Path('/home/xhj/liftrace-controlwork-worktrees')
if base.is_dir(): paths.extend(p for p in sorted(base.iterdir()) if p.is_dir() and (p/'.git').exists())
paths.append(pathlib.Path('/home/xhj/liftrace-sim'))
repos={};unavailable=[]
for p in paths:
 if not p.is_dir():unavailable.append({'path':str(p),'reason':'directory absent'});continue
 c=git(p,'rev-parse','--git-common-dir')
 if c.returncode:unavailable.append({'path':str(p),'reason':'not a git checkout'});continue
 common=(p/c.stdout.strip()).resolve()
 # Normal local repos only; do not allow cat-file to invoke a promisor fetch.
 partial=git(p,'config','--get','extensions.partialClone')
 if partial.returncode==0 and partial.stdout.strip():raise RuntimeError('partial clone requires offline-specific handling: '+str(p))
 repo=repos.setdefault(str(common),{'git_common_dir':str(common),'check_path':str(p),'checkouts':[]})
 repo['checkouts'].append(str(p))
records=[]
for sha in missing:
 found=[];checked=[]
 for repo in repos.values():
  p=repo['check_path'];cp=git(p,'cat-file','-e',sha+'^{commit}')
  checked.append({'git_common_dir':repo['git_common_dir'],'check_path':p,'exists':cp.returncode==0})
  if cp.returncode==0:
   vals=git(p,'show','-s','--format=%H%x00%an%x00%ae%x00%aI%x00%cn%x00%ce%x00%cI%x00%P%x00%s',sha)
   if vals.returncode:raise RuntimeError(vals.stderr)
   refs=git(p,'for-each-ref','--contains='+sha,'--format=%(refname)')
   direct=git(p,'for-each-ref','--points-at='+sha,'--format=%(refname)')
   if refs.returncode or direct.returncode:raise RuntimeError('ref lookup failed')
   found.append(dict(repo,metadata=dict(zip(['commit','author_name','author_email','author_date','committer_name','committer_email','committer_date','parents','subject'],vals.stdout.strip().split('\0'))),containing_refs=refs.stdout.splitlines(),direct_refs=direct.stdout.splitlines()))
 records.append({'commit':sha,'found_in':found,'checked_repositories':checked})
main=git(R,'rev-parse','--verify','6cd0207f^{commit}')
if main.returncode:raise RuntimeError('reported new main commit unavailable locally')
head=main.stdout.strip()
bundle_git=pathlib.Path('/home/xhj/liftrace-archives/20261008-history/verify.git')
bundle_has=git(bundle_git,'cat-file','-e',head+'^{commit}').returncode==0
if bundle_has:raise RuntimeError('new main unexpectedly exists in offline import')
current_main=git(R,'rev-parse','refs/heads/main').stdout.strip()
result={'checked_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'local cat-file and existing ref reads only; no fetch, push, checkout, new tags or bundle mutation','repositories':list(repos.values()),'unavailable_paths':unavailable,'original_missing_in_shared_repo':len(missing),'found_in_other_repositories':sum(bool(x['found_in']) for x in records),'not_found_in_checked_repositories':sum(not x['found_in'] for x in records),'records':records,'post_snapshot_main':{'reported_commit':head,'current_main_at_check':current_main,'exists_in_independent_bundle_import':bundle_has,'bundle_ref_snapshot_main':next(x['object'] for x in json.loads((D/'bundle_refs.json').read_text()) if x['ref']=='refs/heads/main')}}
(D/'cross_repository_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='records'},ensure_ascii=False,indent=2))
for v in records:
 print(v['commit'],[(r['git_common_dir'],len(r['containing_refs'])) for r in v['found_in']])
