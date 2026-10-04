import copy,unittest
from trial_result import evaluate

class TerminalResultTests(unittest.TestCase):
    def test_complete_three_drop_return_hover_and_manual_landing_is_pass(self):
        sup=dict(trial='high_priority',end_reason='landed_after_flight',ever_armed=True,
                 ever_airborne=True,armed=False,landed_state=1,actuator_mode='real')
        data=dict(mission=dict(mission_id='flight',phase='COMPLETE',mission_failed=False,committed_slots=3),
                  high=dict(trial_memory_count=3),terminal_hover=dict(stage='PILOT_HANDOFF'),
                  landing_command=dict(mission_id='flight',active_command='LAND',active_decision_seq=9))
        settings=dict(terminal_hover_agl=.3)
        self.assertEqual(evaluate(sup,data,settings)['status'],'PASS')
        for key,patch in [('mission',dict(mission_failed=True)),('mission',dict(phase='ABORT')),
                          ('mission',dict(committed_slots=2)),('terminal_hover',dict(stage='DESCEND_TO_HOVER')),
                          ('landing_command',dict(mission_id='another')),('landing_command',dict(active_decision_seq=0))]:
            bad=copy.deepcopy(data);bad[key].update(patch)
            self.assertEqual(evaluate(sup,bad,settings)['status'],'INCOMPLETE')
        for patch in (dict(armed=True),dict(landed_state=2),dict(ever_airborne=False)):
            self.assertEqual(evaluate({**sup,**patch},data,settings)['status'],'INCOMPLETE')
        self.assertEqual(evaluate(sup,data,{})['status'],'INCOMPLETE')

if __name__=='__main__':unittest.main()
