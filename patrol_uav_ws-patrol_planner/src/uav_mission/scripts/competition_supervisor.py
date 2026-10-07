#!/usr/bin/env python3
"""独立整机入口；--check-config/--output-dir 只离线检查与生成。"""
import argparse,json
from pathlib import Path
import yaml
from uav_mission.competition_config import validate,generate,apply_overrides
from uav_mission.hardware_session import run_session

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=('preview','flight'),help='preview 预览；flight 使用真实受保护投递链')
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--site-config',type=Path,required=True,help='现场测量 YAML；不会自动选择测试场地')
    p.add_argument('--model',type=Path)
    p.add_argument('--metadata',type=Path)
    p.add_argument('--motion-optimization',choices=('on','off'),help='显式覆盖运动优化；省略继承 YAML')
    p.add_argument('--obstacle-columns',choices=('on','off'),help='显式覆盖障碍禁越柱；省略继承 YAML，关闭不删除实体障碍')
    p.add_argument('--motion-action-timeout',type=float,help='单个运动动作秒数；省略保留 YAML 或正式默认 90 秒')
    p.add_argument('--check-config',action='store_true',help='只检查配置，绝不启动 ROS')
    p.add_argument('--output-dir',type=Path,help='配合 --check-config 离线生成 runtime/control/overrides')
    p.add_argument('--fc-reference',type=float,nargs=3,metavar=('X','Y','Z'),help='离线生成的静止 FC 参考，单位 m；仅与 --output-dir 同用')
    a,extra=p.parse_known_args()
    if any(':=' not in value for value in extra):p.error('unknown arguments: '+str(extra))
    if a.output_dir is not None and not a.check_config:p.error('--output-dir requires --check-config')
    if (a.output_dir is None)!=(a.fc_reference is None):p.error('--output-dir and --fc-reference must be used together')
    a.trial='competition';a.real_release=a.mode=='flight'
    try:
        settings=apply_overrides(yaml.safe_load(a.site_config.read_text()),a.motion_optimization,a.obstacle_columns,a.motion_action_timeout)
        validate(settings,flight=a.mode=='flight' or not a.check_config or a.output_dir is not None)
    except ValueError as error:p.error(str(error))
    cfg=a.root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition'
    rig=yaml.safe_load((cfg/'known_rig.yaml').read_text())
    startup=yaml.safe_load((cfg/'mapping_startup.yaml').read_text())
    if a.check_config:
        if a.output_dir is not None:generate(a.root,a.output_dir,settings,a.fc_reference,rig)
        print(json.dumps(dict(status='CONFIG_VALID',ros_started=False,site_confirmed=settings['site_confirmed'],
            motion_optimization=settings.get('motion_optimization',{}).get('enabled',False),
            obstacle_columns=settings['obstacle_columns_enabled'],
            motion_action_timeout=settings.get('motion_action_timeout',90.)),ensure_ascii=False))
        return
    run_session(a,settings,rig,startup,generate,'uav_mission','competition_')
if __name__=='__main__':main()
