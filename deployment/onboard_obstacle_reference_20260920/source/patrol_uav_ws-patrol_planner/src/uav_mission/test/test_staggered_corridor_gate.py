#!/usr/bin/env python3
"""New geometry must not disable passage order, bounds, or collision checks."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import yaml
from uav_mission.contact_policy import relevant_contact_pairs

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'staggered_gate', str(ROOT / 'scripts/navigation_vcl06_assertion.py'))
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
CONFIG = yaml.safe_load((ROOT / 'config/vcl06_horizontal_field_runtime.yaml').read_text())

class StaggeredGateTest(unittest.TestCase):
    def test_both_flat_h_supports_are_ground_but_wall_contacts_remain(self):
        launch = ET.parse(ROOT / 'launch/navigation_horizontal_search_vcl06.launch').getroot()
        patterns = yaml.safe_load(launch.find(
            "rosparam[@param='/gazebo_contact_monitor/ignored_collision_patterns']").text)
        body = 'iris_mid360::iris::base_link::base_link_inertia_collision'
        supports = [(body, name+'::link::collision') for name in ['landing_h', 'landing_h_clone']]
        self.assertEqual(relevant_contact_pairs(supports, patterns), [])
        wall = ('iris_mid360::competition_contact_link::competition_guard_collision',
                'toudi2::Wall_9::Wall_9_Collision')
        self.assertEqual(relevant_contact_pairs(supports+[wall], patterns),
                         [tuple(sorted(wall))])

    def reducer(self):
        return MODULE.Vcl06GateReducer(
            post_delivery_route=CONFIG['mission']['post_delivery_route'],
            post_delivery_doors=CONFIG['post_delivery_gate']['doors'],
            expected_door_order=CONFIG['post_delivery_gate']['expected_door_order'])

    def test_new_geometry_and_legacy_defaults_are_distinct(self):
        reducer = self.reducer()
        self.assertEqual([d.name for d in reducer.post_delivery_doors],
                         ['corridor_entry', 'Wall_20', 'Wall_22'])
        with self.assertRaises(ValueError):
            MODULE._normalize_doors(CONFIG['post_delivery_gate']['doors'], 7)

    def test_missing_duplicate_or_reordered_sections_rejected(self):
        original = CONFIG['post_delivery_gate']['doors']
        order = CONFIG['post_delivery_gate']['expected_door_order']
        for values in [original[:-1], list(reversed(original)),
                       [original[0], original[0], original[2]]]:
            with self.assertRaises(ValueError):
                MODULE._normalize_doors(values, 7, order)

    def test_each_passage_bounds_and_direction_remain_enforced(self):
        for door_index in range(3):
            for invalid in ['lateral', 'height', 'reverse']:
                reducer = self.reducer()
                for index, door in enumerate(reducer.post_delivery_doors):
                    axis = 0 if door.axis == 'x' else 1
                    p = [0.0, 0.0, 1.4]
                    p[axis] = door.coordinate - .1
                    p[1-axis] = (door.lateral_min + door.lateral_max)/2
                    if index == door_index:
                        if invalid == 'lateral':p[1-axis] = door.lateral_max + .1
                        if invalid == 'height':p[2] = door.z_max + .1
                    q = list(p);q[axis] += .2
                    if index == door_index and invalid == 'reverse':p,q=q,p
                    reducer._observe_door_segment(door,p,q,door.route_indices[0])
                    if index == door_index:break
                self.assertTrue(reducer.errors,(door_index,invalid))

    def test_collision_is_still_a_failure(self):
        reducer = self.reducer()
        reducer.observe_status('contact',{'status':'READY','ready':True,'actual_collision_count':1})
        self.assertIn('actual_collision',reducer.report()['errors'])

if __name__ == '__main__':
    unittest.main()
