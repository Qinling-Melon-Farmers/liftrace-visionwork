"""Exercise the production controller against isolated filesystem devices."""
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parent

class PWMDeviceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix='servo_pwm_build_')
        cls.binary = Path(cls.build.name)/'driver'
        subprocess.run(['g++','-std=c++11','-I',str(PACKAGE/'include'),str(PACKAGE/'test_pwm_device.cpp'),str(PACKAGE/'src/PWMController.cpp'),'-o',str(cls.binary)],check=True)

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def setUp(self):
        self.work = tempfile.TemporaryDirectory(prefix='servo_pwm_device_')
        self.base = Path(self.work.name)
        self.root = self.base/'class/pwm'
        self.root.mkdir(parents=True)

    def tearDown(self):
        self.work.cleanup()

    def device(self, chip, address, initialized=True):
        path = self.base/'platform'/address/'pwm'/('pwmchip'+str(chip))
        path.mkdir(parents=True)
        (self.root/path.name).symlink_to(path)
        if initialized:
            p = path/'pwm0';p.mkdir()
            for name,value in dict(enable='0',duty_cycle='0',period='20000000',polarity='normal').items():
                (p/name).write_text(value)
        return path/'pwm0'

    def call(self, chip=-1):
        return subprocess.run([str(self.binary),str(self.root),'unused',str(chip)],capture_output=True,text=True)

    def test_reordered_d2_controller_receives_production_pulse_and_d3_does_not(self):
        old = self.device(0,'fd8b0010.pwm')
        new = self.device(9,'fd8b0000.pwm')
        result = self.call()
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('pwmchip9/pwm0',result.stdout)
        self.assertEqual((new/'duty_cycle').read_text(),'2100000')
        self.assertEqual((new/'enable').read_text(),'0')
        self.assertEqual((old/'duty_cycle').read_text(),'0')

    def test_old_controller_is_not_a_fallback_when_d2_missing(self):
        old = self.device(0,'fd8b0010.pwm')
        result = self.call()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('found 0',result.stderr)
        self.assertEqual((old/'duty_cycle').read_text(),'0')

    def test_duplicate_address_is_rejected_before_output(self):
        one = self.device(8,'fd8b0000.pwm')
        two = self.device(9,'fd8b0000.pwm')
        result = self.call()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('found 2',result.stderr)
        self.assertEqual((one/'duty_cycle').read_text(),'0')
        self.assertEqual((two/'duty_cycle').read_text(),'0')

    def test_explicit_wrong_chip_still_checks_platform_address(self):
        old = self.device(0,'fd8b0010.pwm');self.device(9,'fd8b0000.pwm')
        result = self.call(0)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('address mismatch',result.stderr)
        self.assertEqual((old/'duty_cycle').read_text(),'0')

    def test_uninitialized_d2_is_rejected_before_service_start(self):
        self.device(9,'fd8b0000.pwm',initialized=False)
        result=self.call()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('not initialized/writable',result.stderr)

if __name__ == '__main__':unittest.main()
