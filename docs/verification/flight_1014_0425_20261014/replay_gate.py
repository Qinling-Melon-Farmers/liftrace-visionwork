"""仅离线回算：读取指定Git版本的生产计时类，不修改飞行源码。"""
from pathlib import Path
import argparse,csv,subprocess,tempfile
ap=argparse.ArgumentParser();ap.add_argument('--run-dir',type=Path,required=True);ap.add_argument('--nav-repo',type=Path,required=True);ap.add_argument('--revision',default='2adc69bf9ae7cc3b62cbd88f06ba61034c20f427');args=ap.parse_args()
source='patrol_uav_ws-patrol_planner/src/patrol_control/include/patrol_control/landing_high_alignment.h'
header=subprocess.check_output(['git','show',args.revision+':'+source],cwd=args.nav_repo,text=True)
runner=Path(__file__).with_name('gate_runner.cpp');n=args.run_dir/'numeric'
old='''        for (double stamp : {odom, receipt, mark, mark_receipt})
            if (stamp <= 0 || stamp > now || now-stamp > 0.2) return reject();'''
new='''        for (double stamp : {odom, receipt})
            if (stamp <= 0 || stamp > now || now-stamp > 0.2) return reject();
        for (double stamp : {mark, mark_receipt})
            if (stamp <= 0 || stamp > now || now-stamp > 0.5) return reject();'''
assert header.count(old)==1
sensitivity=header.replace(old,new).replace('mark-last_mark_ > 0.2','mark-last_mark_ > 0.5').replace('mark_receipt-last_mark_receipt_ > 0.2','mark_receipt-last_mark_receipt_ > 0.5')
with tempfile.TemporaryDirectory(prefix='h-gate-offline-') as tmp:
 base=Path(tmp);inc=base/'patrol_control';inc.mkdir()
 for text,name in [(header,'h_gate_cpp_result.csv'),(sensitivity,'h_gate_cpp_sensitivity.csv')]:
  (inc/'landing_high_alignment.h').write_text(text)
  binary=base/'gate_runner';subprocess.run(['g++','-std=c++14','-O2','-I'+str(base),str(runner),'-o',str(binary)],check=True)
  subprocess.run([str(binary),str(n/'gate_input.txt'),str(n/name)],check=True)
  rows=list(csv.DictReader((n/name).open()));ok=[float(x['now'])-1791923122.167234 for x in rows if x['ready']=='1'];print(name,'qualifications=',len(ok),'first=',ok[0] if ok else None)