import unittest
from uav_mission.navigation_recovery_config import recovery_parameters

class RecoveryConfigurationTest(unittest.TestCase):
    def test_default_disabled_at_every_receiver(self):
        p=recovery_parameters({},-.22)
        enabled={k:v for k,v in p.items() if k.endswith('/enabled') or k.endswith('/recovery_layers_enabled')}
        self.assertEqual(len(enabled),5)
        self.assertFalse(any(enabled.values()))
    def test_opt_in_complete_wiring_and_measured_reference(self):
        p=recovery_parameters({'navigation_recovery':{'enabled':True}},-.22)
        for prefix in ('/navigation_recovery','/traj_server/navigation_recovery','/fast_planner_node/navigation_recovery'):
            self.assertTrue(p[prefix+'/enabled'])
            self.assertAlmostEqual(p[prefix+'/hard_min_z'],-.17)
            self.assertAlmostEqual(p[prefix+'/hard_max_z'],3.28)
            self.assertEqual(p[prefix+'/max_seconds'],8.)
        self.assertTrue(p['/fast_planner_node/sdf_map/recovery_layers_enabled'])
        self.assertTrue(p['/navigation/planner_bridge/navigation_recovery/enabled'])
        self.assertFalse(any('ev_' in k or 'reset' in k for k in p))
    def test_no_unbounded_configuration_or_truthy_strings(self):
        for values in ({'enabled':'false'},{'enabled':1},{'max_seconds':100},None):
            with self.assertRaises(ValueError):recovery_parameters({'navigation_recovery':values},0.)
        with self.assertRaises(ValueError):recovery_parameters({},float('nan'))

if __name__=='__main__':unittest.main()
