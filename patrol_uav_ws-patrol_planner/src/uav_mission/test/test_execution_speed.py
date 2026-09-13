import unittest
import ast
from pathlib import Path
from types import SimpleNamespace
from uav_mission.execution_speed import FollowingSpeed


class SpeedTests(unittest.TestCase):
    def test_transit_stays_fast_until_staging_arrival(self):
        p=FollowingSpeed()
        self.assertEqual(p.select('RETURN_HOME','post_delivery_route:1',0),('TRANSIT_TO_CORRIDOR',1.0))
        self.assertEqual(p.select('RETURN_HOME','post_delivery_route:2',1),('CORRIDOR',.15))
        self.assertEqual(p.select('RETURN_HOME','post_delivery_route:9',8),('CORRIDOR',.15))

    def test_precision_and_search_resume(self):
        p=FollowingSpeed()
        self.assertEqual(p.select('SEARCH','high_view_full:SURVEY',0)[1],1.0)
        self.assertEqual(p.select('APPROACH','high_weight_search_interrupt',0)[1],.4)
        self.assertEqual(p.select('RESUME','coverage_resume',0)[1],1.0)
        self.assertEqual(p.select('LAND','land',9)[1],.15)

    def test_unrecognized_return_does_not_inherit_transit_speed(self):
        self.assertEqual(FollowingSpeed().select('RETURN_HOME','emergency',0),('PRECISION',.4))

    def test_invalid_profile_rejected(self):
        for kw in ({'cruise_lead_m':float('nan')},{'corridor_after_waypoints':0},{'corridor_lead_m':1.1}):
            with self.assertRaises(ValueError):FollowingSpeed(**kw)

    def test_dispatch_speed_wins_over_old_stage_zero(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'scripts/navigation_mission_manager.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_publish_action')
        body=[n for n in method.body if (isinstance(n,ast.If) and (isinstance(n.test,ast.BoolOp) or isinstance(n.test,ast.Name) and n.test.id=='speed_config')) or
              (isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='speed_config')]
        values={'~following_speed_profile':{'cruise_lead_m':1.0},'~mission/post_delivery_parameter_stages':[
            {'after_completed_waypoints':0,'parameters':{'/px4_max_distance':.15,'/traj_server/traj_server/target_dist':.15}}]}
        ros=SimpleNamespace(get_param=lambda k,default=None:values.get(k,default),set_param=lambda k,v:values.update({k:v}),
                            loginfo=lambda *args:None,Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:100.)))
        obj=SimpleNamespace(_runtime=SimpleNamespace(core=SimpleNamespace(post_delivery_route_index=0)))
        exec(compile(ast.Module(body=body,type_ignores=[]),'speed stages','exec'),dict(rospy=ros,self=obj,FollowingSpeed=FollowingSpeed,
             action=SimpleNamespace(command='RETURN_HOME',reason='post_delivery_route:1')))
        self.assertEqual(values['/px4_max_distance'],1.0)
        self.assertEqual(values['/traj_server/traj_server/target_dist'],1.0)
        self.assertEqual(obj._following_speed_state[0],'TRANSIT_TO_CORRIDOR')


if __name__=='__main__':unittest.main()
