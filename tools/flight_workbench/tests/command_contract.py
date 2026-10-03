"""Emit the backend command contract for the dependency-free JS regression."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import wb_board
config=wb_board.load_config()
cases=[]
for group in config['groups']:
    for mode in ('preview','flight'):
        for check in (False,True):
            for speed in (group.get('speed_options') or [None]):
                real=group.get('release')=='real' and mode=='flight'
                command,_=wb_board.build_group_command(config,group,mode,real_release=real,
                    check_config=check,capture_speed=speed)
                cases.append(dict(group=group,mode=mode,real=real,check=check,speed=speed,command=command))
print(json.dumps(dict(connection=config['connection'],cases=cases),ensure_ascii=False))
