#!/usr/bin/env python3
"""Orange Pi 5 Plus debug service. Owns PWM exclusively until shutdown.

Slots: rear febf0000 Pin16, right febf0020 Pin7, left febf0010 Pin18.
Status: [rear_duty, rear_enabled, right_duty, right_enabled, left_duty, left_enabled].
Run init_pwm.sh first. No startup pulses; acknowledgements are electrical only.
"""
import fcntl
import os
import threading

import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from patrol_control.srv import SetServoDuty, SetServoDutyResponse

PWM_PERIOD_NS = 20_000_000
DUTY_MIN_NS = 500_000
DUTY_MAX_NS = 2_500_000
EXPECTED_DEVICES = ['febf0000.pwm', 'febf0020.pwm', 'febf0010.pwm']
LOCK_PATH = '/run/lock/liftrace-servo.lock'


class HardwareLock:
    def __init__(self, path=LOCK_PATH):
        self.fd = os.open(path, os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(self.fd)
            self.fd = None
            raise RuntimeError('PWM is occupied by initialization/raw/debug service')

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def _read_text(path):
    try:
        with open(path) as handle:
            return handle.read().strip()
    except OSError:
        return None


def _write_text(path, value):
    try:
        with open(path, 'w') as handle:
            handle.write(str(value))
            handle.flush()
        return _read_text(path) == str(value)
    except OSError as exc:
        rospy.logerr('PWM write failed %s=%s: %s', path, value, exc)
        return False


def _find_pwm_chip(device_name):
    paths = [f'/sys/class/pwm/pwmchip{i}' for i in range(64)]
    matches = [p + '/pwm0' for p in paths if os.path.isdir(p)
               and f'/{device_name}/' in os.path.realpath(p)]
    return matches[0] if len(matches) == 1 else None


def _is_orange_pi_5_plus():
    model = _read_text('/proc/device-tree/model') or ''
    return 'OPi 5 Plus' in model or 'Orange Pi 5 Plus' in model


class PWMChannel:
    def __init__(self, pwm_path, slot_name):
        self.pwm_path = pwm_path
        self.slot_name = slot_name

    @property
    def enable_path(self):
        return self.pwm_path + '/enable'

    @property
    def duty_path(self):
        return self.pwm_path + '/duty_cycle'

    def read_enable(self):
        return _read_text(self.enable_path) == '1'

    def read_duty(self):
        try:
            return int(_read_text(self.duty_path))
        except (TypeError, ValueError):
            return None

    def set_enable(self, enabled):
        target = '1' if enabled else '0'
        if _read_text(self.enable_path) == target:
            return True
        return _write_text(self.enable_path, target)

    def set_duty(self, duty_ns):
        return DUTY_MIN_NS <= duty_ns <= DUTY_MAX_NS and _write_text(self.duty_path, duty_ns)

    def safe_pulse(self, duty_ns, hold=False, wait_sec=0.3, cancel=None):
        cancel = cancel or threading.Event()
        success = False
        message = 'PWM operation failed'
        try:
            if cancel.is_set():
                raise RuntimeError('request cancelled')
            if not self.set_enable(False):
                raise RuntimeError('failed to disable before set_duty')
            if cancel.wait(0.02):
                raise RuntimeError('request cancelled')
            if not self.set_duty(duty_ns):
                raise RuntimeError('failed to write/read duty')
            if cancel.is_set():
                raise RuntimeError('request cancelled')
            if not self.set_enable(True):
                raise RuntimeError('failed to enable/read output')
            if cancel.wait(wait_sec):
                raise RuntimeError('request cancelled')
            if not hold and not self.set_enable(False):
                raise RuntimeError('failed to disable after pulse')
            if self.read_duty() != duty_ns or _read_text(self.enable_path) != ('1' if hold else '0'):
                raise RuntimeError('final PWM readback mismatch')
            success, message = True, 'ok'
        except Exception as exc:
            message = str(exc)
        finally:
            if not success:
                try:
                    if not self.set_enable(False) or _read_text(self.enable_path) != '0':
                        message += '; output disable unconfirmed'
                except Exception as exc:
                    message += '; cleanup failed: ' + str(exc)
        return success, self.read_duty() or 0, message


class ServoTransactions:
    """No stale queued request can re-enable PWM after stop has returned."""
    def __init__(self, channels):
        self.channels = channels
        self.state_lock = threading.Lock()
        self.transaction_lock = threading.Lock()
        self.stop_lock = threading.Lock()
        self.generation = 0
        self.cancel = threading.Event()
        self.stopping = False
        self.fault = False
        self.shutting_down = False

    def set(self, slot, duty, hold):
        if type(slot) is not int or not 1 <= slot <= 3 or not DUTY_MIN_NS <= duty <= DUTY_MAX_NS:
            return False, 0, 'invalid slot or duty'
        with self.state_lock:
            if self.stopping or self.fault or self.shutting_down:
                return False, 0, 'stopping/fault/shutdown: stop successfully before sending'
            generation, cancel = self.generation, self.cancel
        with self.transaction_lock:
            with self.state_lock:
                if generation != self.generation or self.stopping or self.fault or self.shutting_down:
                    return False, 0, 'stale request cancelled'
            channel = self.channels[slot - 1]
            result = channel.safe_pulse(duty, hold, cancel=cancel)
            if not result[0] and _read_text(channel.enable_path) != '0':
                with self.state_lock:
                    self.fault = True
            return result

    def stop(self, shutdown=False):
        with self.stop_lock:
            with self.state_lock:
                self.stopping = True
                self.generation += 1
                self.cancel.set()
                if shutdown:
                    self.shutting_down = True
            errors = []
            with self.transaction_lock:
                for channel in self.channels:
                    try:
                        if not channel.set_enable(False) or _read_text(channel.enable_path) != '0':
                            errors.append(channel.slot_name + ': disable readback failed')
                    except Exception as exc:
                        errors.append(channel.slot_name + ': ' + str(exc))
                with self.state_lock:
                    self.fault = bool(errors)
                    # A failed explicit shutdown can be retried with stop/shutdown.
                    if errors and shutdown:
                        self.shutting_down = False
                    self.cancel = threading.Event()
                    self.stopping = False
            return not errors, '; '.join(errors) or 'all PWM outputs disabled; duty unchanged'


class ServoDebugNode:
    def __init__(self):
        if not _is_orange_pi_5_plus():
            raise RuntimeError('Orange Pi 5 Plus required')
        self.hardware_lock = HardwareLock()
        try:
            # Acquire before ROS registration can replace a same-name node.
            rospy.init_node('servo_debug_node', anonymous=False)
            channels = []
            for device, name in zip(EXPECTED_DEVICES, ['rear', 'right', 'left']):
                path = _find_pwm_chip(device)
                if path is None:
                    raise RuntimeError('PWM device unavailable: ' + device)
                for attr in ['period', 'polarity', 'enable', 'duty_cycle']:
                    if not os.access(path + '/' + attr, os.W_OK):
                        raise RuntimeError('PWM not writable; run init_pwm.sh: ' + path)
                if (_read_text(path + '/period') != str(PWM_PERIOD_NS)
                        or _read_text(path + '/polarity') != 'normal'
                        or _read_text(path + '/enable') != '0'):
                    raise RuntimeError('require 20ms/normal/disabled; run init_pwm.sh')
                channels.append(PWMChannel(path, name))
            self.channels = channels
            self.transactions = ServoTransactions(channels)
            rospy.on_shutdown(self.cleanup)
            rospy.Service('/SetServoDuty', SetServoDuty, self._handle_set_duty)
            rospy.Service('/servo_debug/stop', Trigger, self._handle_stop)
            rospy.Service('/servo_debug/shutdown', Trigger, self._handle_shutdown)
            self.status_pub = rospy.Publisher('/servo_debug/status', Float32MultiArray, queue_size=10)
            rospy.Timer(rospy.Duration(0.1), self._publish_status)
        except BaseException:
            if hasattr(self, 'transactions'):
                self.cleanup()
            self.hardware_lock.close()
            raise

    def _handle_set_duty(self, req):
        ok, actual, message = self.transactions.set(req.slot, int(req.duty_cycle_ns), bool(req.hold))
        return SetServoDutyResponse(success=ok, actual_duty_ns=actual, message=message)

    def _handle_stop(self, req):
        ok, message = self.transactions.stop()
        return TriggerResponse(success=ok, message=message)

    def _handle_shutdown(self, req):
        ok, message = self.transactions.stop(shutdown=True)
        if ok:
            timer = threading.Timer(0.1, rospy.signal_shutdown, args=['explicit servo debug shutdown'])
            timer.daemon = True
            timer.start()
        return TriggerResponse(success=ok, message=message)

    def cleanup(self):
        ok, message = self.transactions.stop(shutdown=True)
        if not ok:
            rospy.logerr('PWM shutdown unconfirmed: %s', message)
        return ok

    def _publish_status(self, event):
        msg = Float32MultiArray()
        msg.data = [value for ch in self.channels
                    for value in [float(ch.read_duty() or 0), 1.0 if ch.read_enable() else 0.0]]
        self.status_pub.publish(msg)

    def run(self):
        try:
            rospy.spin()
        finally:
            self.cleanup()
            self.hardware_lock.close()


if __name__ == '__main__':
    ServoDebugNode().run()
