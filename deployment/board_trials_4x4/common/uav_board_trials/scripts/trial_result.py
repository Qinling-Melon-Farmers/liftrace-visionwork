"""Recorded outcome checks; an actuator ACK is not a measured parcel landing."""
def evaluate(supervisor,latest,settings):
    mission=latest.get('mission',{});high=latest.get('high',{})
    trial=supervisor.get('trial');committed=mission.get('committed_slots',0)
    count=settings.get('delivery_count',2)
    expected={'visual_interrupt':1,'low_multi':count,'full_mission':3}.get(trial,high.get('trial_memory_count',0))
    if trial in ('landing','corridor_landing','memory_only','high_speed_capture'):expected=0
    landed=supervisor.get('end_reason')=='landed_after_flight'
    healthy=not mission.get('mission_failed',False)
    handoff=latest.get('land_handoff',{})
    local_landed=(handoff.get('mode_sent') is True and healthy and
                  handoff.get('mission_id')==mission.get('mission_id') and
                  handoff.get('decision_seq',0)>0 and (
                  handoff.get('decision_seq')==mission.get('active_decision_seq') or
                  (mission.get('phase')=='COMPLETE' and
                   latest.get('landing_command',{}).get('active_command')=='LAND' and
                   latest['landing_command'].get('mission_id')==mission.get('mission_id') and
                   latest['landing_command'].get('active_decision_seq')==handoff.get('decision_seq'))))
    if trial in ('landing','corridor_landing'):
        success=landed and healthy and mission.get('phase')=='COMPLETE' and committed==0
    elif trial=='full_mission':
        success=landed and healthy and mission.get('phase')=='COMPLETE' and committed==3 and mission.get('post_delivery_route_complete') is True
    elif trial=='high_speed_capture':
        success=(landed and healthy and committed==0 and high.get('capture_complete') is True
                 and latest.get('terminal_hover',{}).get('stage') in ('DESCEND_TO_HOVER','PILOT_HANDOFF'))
    elif trial=='memory_only':
        success=(landed and local_landed and committed==0 and high.get('memory_only') is True
                 and 1<=high.get('trial_memory_count',0)<=3)
    else:
        success=landed and local_landed and 1<=expected<=3 and committed==expected
    return dict(status=('CAPTURED' if trial=='high_speed_capture' else 'PASS') if success else 'INCOMPLETE',trial=trial,
                actuator_mode=supervisor.get('actuator_mode',settings.get('actuator_mode','mock')),
                expected_deliveries=expected,committed_deliveries=committed,
                validation_scope='capture_route_and_manual_end_only' if trial=='high_speed_capture' else 'trial',
                speed_and_recognition_validation='PENDING_OFFLINE' if trial=='high_speed_capture' else 'not_evaluated_here',
                final_mission=mission,final_high_view=high,supervisor=supervisor,
                auto_land_handoff=handoff)
