"""Offline: real transaction/PWM/lock code, temporary files only; no ROS/hardware."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

PKG = Path(__file__).resolve().parents[1]
modules = {'rospy': types.SimpleNamespace(logerr=lambda *a: None),
           'std_msgs': types.ModuleType('std_msgs'),
           'std_msgs.msg': types.SimpleNamespace(Float32MultiArray=object),
           'std_srvs': types.ModuleType('std_srvs'),
           'std_srvs.srv': types.SimpleNamespace(Trigger=object, TriggerResponse=object),
           'patrol_control': types.ModuleType('patrol_control'),
           'patrol_control.srv': types.SimpleNamespace(SetServoDuty=object, SetServoDutyResponse=object)}
spec = importlib.util.spec_from_file_location('servo_debug_tested', PKG/'scripts/servo_debug_node.py')
node = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, modules):
    spec.loader.exec_module(node)


class ImmediateWait:
    def is_set(self): return False
    def wait(self, timeout): return False


class ServoTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.channels = []
        for name in ['rear', 'right', 'left']:
            p = self.root/name; p.mkdir()
            (p/'enable').write_text('0'); (p/'duty_cycle').write_text('0')
            self.channels.append(node.PWMChannel(str(p), name))

    def test_first_send_and_boundaries_all_slots(self):
        for channel in self.channels:
            for duty in [500000, 1700000, 2500000]:
                for hold in [False, True]:
                    result = channel.safe_pulse(duty, hold, cancel=ImmediateWait())
                    self.assertTrue(result[0], result)
                    self.assertEqual(channel.read_duty(), duty)
                    self.assertEqual(channel.read_enable(), hold)

    def test_idempotent_enable_skips_write(self):
        with patch.object(node, '_write_text', side_effect=AssertionError('redundant write')):
            self.assertTrue(self.channels[0].set_enable(False))

    def test_invalid_duty_never_enabled(self):
        for duty in [499999, 2500001]:
            self.assertFalse(self.channels[0].safe_pulse(duty, cancel=ImmediateWait())[0])
            self.assertFalse(self.channels[0].read_enable())

    def test_failure_each_write_stage_reports_failure(self):
        original = node._write_text
        for stage in ['disable', 'duty', 'enable', 'final_disable']:
            channel = self.channels[0]
            Path(channel.enable_path).write_text('1' if stage=='disable' else '0')
            def write(path, value):
                attr=path.rsplit('/',1)[1]; value=str(value)
                reject=(stage in ['disable','final_disable'] and attr=='enable' and value=='0'
                        or stage=='enable' and attr=='enable' and value=='1'
                        or stage=='duty' and attr=='duty_cycle')
                return False if reject else original(path,value)
            with self.subTest(stage=stage), patch.object(node,'_write_text',side_effect=write):
                result=channel.safe_pulse(1800000,cancel=ImmediateWait())
                self.assertFalse(result[0], result)

    def test_exception_and_readback_mismatch_cleanup(self):
        original=node._write_text
        def mismatch(path,value):
            if path.endswith('duty_cycle'):
                Path(path).write_text('999999');return True
            return original(path,value)
        with patch.object(node,'_write_text',side_effect=mismatch):
            self.assertFalse(self.channels[0].safe_pulse(1800000,cancel=ImmediateWait())[0])
        self.assertFalse(self.channels[0].read_enable())
        with patch.object(self.channels[0],'set_duty',side_effect=OSError('fixture error')):
            self.assertFalse(self.channels[0].safe_pulse(1800000,cancel=ImmediateWait())[0])
        self.assertFalse(self.channels[0].read_enable())

    def test_stop_preserves_duty_and_fault_recovers_only_after_successful_stop(self):
        tx=node.ServoTransactions(self.channels)
        for ch in self.channels:
            ch.set_duty(1800000);ch.set_enable(True)
        with patch.object(self.channels[0],'set_enable',return_value=False):
            self.assertFalse(tx.stop()[0])
            self.assertFalse(tx.set(1,1800000,True)[0])
        self.assertTrue(tx.stop()[0])
        self.assertEqual([ch.read_duty() for ch in self.channels],[1800000]*3)
        self.assertTrue(tx.set(1,1800000,True)[0])
        self.assertTrue(tx.stop()[0])

    def test_stop_cancels_active_and_queued_requests(self):
        tx=node.ServoTransactions(self.channels)
        active_entered=threading.Event();queued_waiting=threading.Event()
        base=threading.Lock()
        class ObservedLock:
            def __enter__(self):
                if threading.current_thread().name=='queued': queued_waiting.set()
                base.acquire()
            def __exit__(self,*args): base.release()
        tx.transaction_lock=ObservedLock()
        results={}
        def pulse(duty,hold,cancel):
            self.channels[0].set_enable(True);active_entered.set()
            if not cancel.wait(2): raise AssertionError('stop did not interrupt')
            self.channels[0].set_enable(False)
            return False,0,'cancelled'
        with patch.object(self.channels[0],'safe_pulse',side_effect=pulse):
            a=threading.Thread(target=lambda:results.update(active=tx.set(1,1800000,True)))
            q=threading.Thread(name='queued',target=lambda:results.update(queued=tx.set(1,2000000,True)))
            a.start();self.assertTrue(active_entered.wait(1));q.start();self.assertTrue(queued_waiting.wait(1))
            self.assertTrue(tx.stop()[0]);a.join(1);q.join(1)
            self.assertFalse(a.is_alive());self.assertFalse(q.is_alive())
        self.assertFalse(results['active'][0]);self.assertFalse(results['queued'][0])
        self.assertTrue(all(not ch.read_enable() for ch in self.channels))

    def test_requests_during_stop_and_after_shutdown_rejected(self):
        tx=node.ServoTransactions(self.channels)
        entered=threading.Event();release=threading.Event()
        def close(enabled):
            entered.set();release.wait(2);return True
        with patch.object(self.channels[0],'set_enable',side_effect=close):
            thread=threading.Thread(target=tx.stop);thread.start();self.assertTrue(entered.wait(1))
            self.assertFalse(tx.set(2,1800000,True)[0]);release.set();thread.join(1)
        self.assertTrue(tx.stop(shutdown=True)[0])
        self.assertFalse(tx.set(2,1800000,True)[0])

    def test_failed_send_latches_unconfirmed_output_fault(self):
        tx=node.ServoTransactions(self.channels)
        Path(self.channels[0].enable_path).write_text('1')
        with patch.object(self.channels[0],'set_enable',return_value=False):
            self.assertFalse(tx.set(1,1800000,False)[0])
        self.assertFalse(tx.set(2,1800000,True)[0])
        self.assertTrue(tx.stop()[0])


class CompiledHardwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        source=cls.root/'check.cpp';cls.bin=cls.root/'check'
        source.write_text(r'''
#include "actuator_pwm/HardwareLock.h"
#include "actuator_pwm/PWMController.h"
#include "actuator_pwm/CheckedPulse.h"
#include <iostream>
int main(int argc,char** argv) {
 try {
  if(std::string(argv[1])=="lock") { actuator_pwm::HardwareLock lock(argv[2]);std::cout<<"locked"<<std::endl;char c;std::cin.get(c); }
  else { PWMController pwm(0,0,"",argv[2]);if(!pwm.disable() || !checkedPulse(pwm,1800000,[](){})) return 3; }
  return 0;
 } catch(const std::exception& e) {std::cerr<<e.what();return 2;}
}
''')
        subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I'+str(PKG/'include'),str(source),str(PKG/'src/PWMController.cpp'),'-o',str(cls.bin)],check=True,capture_output=True)

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def test_python_cpp_and_initialization_lock_contend_same_inode(self):
        lock=self.root/'hardware.lock';lock.touch(mode=0o600)
        inode=lock.stat().st_ino
        holder=node.HardwareLock(str(lock))
        try:
            self.assertEqual(subprocess.run([str(self.bin),'lock',str(lock)],capture_output=True).returncode,2)
            self.assertNotEqual(subprocess.run(['flock','-n',str(lock),'true']).returncode,0)
            with self.assertRaises(RuntimeError):node.HardwareLock(str(lock))
        finally:holder.close()
        process=subprocess.Popen([str(self.bin),'lock',str(lock)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        try:
            self.assertEqual(process.stdout.readline().strip(),'locked')
            with self.assertRaises(RuntimeError):node.HardwareLock(str(lock))
            self.assertNotEqual(subprocess.run(['flock','-n',str(lock),'true']).returncode,0)
        finally:
            process.stdin.close();process.wait(timeout=2);process.stdout.close()
        holder=node.HardwareLock(str(lock));holder.close()
        self.assertEqual(subprocess.run(['flock','-n',str(lock),'true']).returncode,0)
        self.assertEqual(lock.stat().st_ino,inode)
        self.assertEqual(lock.stat().st_mode & 0o777,0o600)

    def test_compiled_controller_pulse_and_idempotent_zero(self):
        root=self.root/'sysfs';p=root/'pwmchip0/pwm0';p.mkdir(parents=True,exist_ok=True)
        (p/'enable').write_text('0');(p/'duty_cycle').write_text('0')
        subprocess.run([str(self.bin),'pulse',str(root)],check=True,capture_output=True)
        self.assertEqual((p/'enable').read_text(),'0')
        self.assertEqual((p/'duty_cycle').read_text(),'1800000')


if __name__=='__main__':unittest.main()
