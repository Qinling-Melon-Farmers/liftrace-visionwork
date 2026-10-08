"""Explicit 08 speed overlays; geometry and H observation settings are not inputs."""
from pathlib import Path
import copy
import yaml

def apply_speed_profile(root, settings, trial, profile):
    if profile not in ('limited', 'competition'):
        raise ValueError('Unknown speed profile')
    if trial != 'full_mission' or settings.get('trial_kind') != 'full_mission':
        raise ValueError('--speed-profile only supports 08 full_mission')
    settings['speed_profile'] = profile
    if profile == 'competition':
        path = Path(root)/'deployment/board_trials_4x4/08_full_mission/competition_speed.yaml'
        values = yaml.safe_load(path.read_text(encoding='utf-8'))
        allowed = {'cruise_speed', 'cruise_acceleration', 'drop_agl',
                   'following_speed_profile', 'motion_optimization', 'resume_survey_enabled'}
        if not isinstance(values, dict) or set(values) != allowed:
            raise ValueError('08 speed overlay must not contain geometry or H settings')
        settings.update(copy.deepcopy(values))
    return settings
