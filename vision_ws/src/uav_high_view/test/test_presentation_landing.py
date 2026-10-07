import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('compose',Path(__file__).resolve().parents[1]/'scripts/presentation_compose.py')
compose=importlib.util.module_from_spec(spec);spec.loader.exec_module(compose)

class LandingWindowTest(unittest.TestCase):
    def test_only_final_route_or_land(self):
        self.assertFalse(compose.landing_window_active({'command':4,'reason':'post_delivery_route:8/9:rev:segment_complete'}))
        self.assertTrue(compose.landing_window_active({'command':4,'reason':'post_delivery_route:9/9:rev:segment_complete'}))
        self.assertTrue(compose.landing_window_active({'command':5}))
        self.assertFalse(compose.landing_window_active({'command':0}))
        self.assertFalse(compose.landing_window_active({'reason':'post_delivery_route:bad'}))
