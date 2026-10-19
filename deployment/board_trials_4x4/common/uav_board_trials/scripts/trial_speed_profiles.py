"""Explicit 08 speed overlays; geometry and H observation settings are not inputs."""
from pathlib import Path
import copy
import yaml

def apply_speed_profile(root, settings, trial, profile):
    if profile not in ('limited', 'competition'):
        raise ValueError('Unknown speed profile')
    low_multi=trial in ('low_multi','low_coverage_full') and settings.get('trial_kind')=='low_multi_corridor'
    if not low_multi and (trial != 'full_mission' or settings.get('trial_kind') != 'full_mission'):
        raise ValueError('--speed-profile only supports 08 full_mission')
    settings['speed_profile'] = profile
    if profile == 'competition':
        path = Path(root)/'deployment/board_trials_4x4/08_full_mission/competition_speed.yaml'
        values = yaml.safe_load(path.read_text(encoding='utf-8'))
        allowed = {'cruise_speed', 'cruise_acceleration', 'drop_agl',
                   'following_speed_profile', 'motion_optimization', 'resume_survey_enabled'}
        window = {'release_min_agl', 'release_max_agl'}
        if not isinstance(values, dict) or set(values)-allowed-window or not allowed <= set(values):
            raise ValueError('08 speed overlay must not contain geometry or H settings')
        if low_multi:
            # Reuse flight speeds only; preserve task 2 release height and logic.
            for key in ('drop_agl','release_min_agl','release_max_agl','resume_survey_enabled'):
                values.pop(key,None)
        settings.update(copy.deepcopy(values))
    return settings
