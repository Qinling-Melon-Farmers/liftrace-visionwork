#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('px4_sitl_start', str(Path(__file__).resolve().parents[1]/'scripts/px4_sitl_start.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Px4ParameterHookTest(unittest.TestCase):
    def test_native_defaults_before_profile(self):
        script = module.parameter_script('/px4 dir/etc/init.d-posix/px4-rc.params', {'EKF2_HGT_REF': 0, 'EKF2_BARO_NOISE': .15})
        self.assertIn(". '/px4 dir/etc/init.d-posix/px4-rc.params'", script)
        self.assertLess(script.index('. '), script.index('param set'))
        self.assertIn('param set EKF2_HGT_REF 0\n', script)

    def test_invalid_input_not_shell_code(self):
        for values in ({'NAME;exit': 0}, {'EKF2_HGT_REF':'$(cmd)'}, {'EKF2_BARO_NOISE':float('nan')}):
            with self.assertRaises(ValueError):module.parameter_script('/px4/params', values)


if __name__ == '__main__':unittest.main()
