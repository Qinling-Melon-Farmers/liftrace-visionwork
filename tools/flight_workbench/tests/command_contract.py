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
                for real in ([False, True] if 'real' in group.get('release_options', []) else [False]):
                    variants = [dict()]
                    if group.get('channel') == 'competition':
                        if mode == 'flight' and not check and not real:
                            continue
                        variants = [dict(motion_optimization=m, obstacle_columns=o)
                                    for m in (None, 'on', 'off') for o in (None, 'on', 'off')]
                        variants.append(dict(competition_config="deployment/custom field'; echo INVALID; #.yaml"))
                    elif not wb_board.is_observation(group):
                        variants.append(dict(motion_optimized=True))
                    if group.get('survey_patterns'):
                        variants += [dict(motion_optimized=True, survey_pattern=p)
                                     for p in group['survey_patterns']]
                    if group.get('resume_survey_supported'):
                        variants += [dict(motion_optimized=True, survey_pattern='snake3', resume_survey=r)
                                     for r in ('on', 'off')]
                    if group.get('lighting_options'):
                        variants += [dict(capture_lighting=l) for l in group['lighting_options']]
                    for options in variants:
                        command,_=wb_board.build_group_command(config,group,mode,real_release=real,
                            check_config=check,capture_speed=speed,**options)
                        cases.append(dict(group=group,mode=mode,real=real,check=check,speed=speed,
                                          options=options,command=command))
print(json.dumps(dict(connection=config['connection'],cases=cases),ensure_ascii=False))
