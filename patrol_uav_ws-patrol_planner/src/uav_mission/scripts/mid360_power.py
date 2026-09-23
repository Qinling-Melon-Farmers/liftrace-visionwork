#!/usr/bin/env python3
"""Control one MID360 through the installed Livox SDK2; no ROS/pip dependency.

Run only while livox_ros_driver2 is stopped (SDK command ports are exclusive).
Boot mode support depends on device firmware; a nonzero ACK is a failure.
"""
import argparse
import ctypes as C
import json
import struct
import sys
import threading
import socket


class Info(C.Structure):
    _pack_ = 1
    _fields_ = [('dev_type', C.c_uint8), ('sn', C.c_char * 16),
                ('lidar_ip', C.c_char * 16)]


class Ack(C.Structure):
    _pack_ = 1
    _fields_ = [('ret_code', C.c_uint8), ('error_key', C.c_uint16)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['query', 'boot-wakeup', 'boot-normal', 'normal', 'sleep', 'wakeup'])
    p.add_argument('--host-ip', default='192.168.1.100')
    p.add_argument('--lidar-ip', default='192.168.1.175')
    p.add_argument('--sdk', default='/usr/local/lib/liblivox_lidar_sdk_shared.so')
    p.add_argument('--config', required=True, help='Existing driver MID360_config.json')
    p.add_argument('--timeout', type=float, default=12)
    args = p.parse_args([arg for arg in sys.argv[1:] if not arg.startswith(('__name:=', '__log:='))])
    with open(args.config, encoding='utf-8') as stream:
        config = json.load(stream)
    host = config['MID360']['host_net_info']
    # Fail rather than sharing the running driver's UDP ports.
    for key in ['cmd_data', 'push_msg', 'point_data', 'imu_data']:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind((host[key + '_ip'], host[key + '_port']))
    sdk = C.CDLL(args.sdk)
    discover = C.CFUNCTYPE(None, C.c_uint32, C.POINTER(Info), C.c_void_p)
    ack_type = C.CFUNCTYPE(None, C.c_int32, C.c_uint32, C.POINTER(Ack), C.c_void_p)
    query_type = C.CFUNCTYPE(None, C.c_int32, C.c_uint32, C.c_void_p, C.c_void_p)
    sdk.LivoxLidarSdkInit.argtypes = [C.c_char_p, C.c_char_p, C.c_void_p]
    sdk.LivoxLidarSdkInit.restype = C.c_bool
    sdk.LivoxLidarSdkStart.restype = C.c_bool
    sdk.SetLivoxLidarInfoChangeCallback.argtypes = [discover, C.c_void_p]
    sdk.QueryLivoxLidarInternalInfo.argtypes = [C.c_uint32, query_type, C.c_void_p]
    for name in ['SetLivoxLidarWorkModeAfterBoot', 'SetLivoxLidarWorkMode']:
        getattr(sdk, name).argtypes = [C.c_uint32, C.c_int, ack_type, C.c_void_p]
    ready, done = threading.Event(), threading.Event()
    handles, outcome = [], []

    @discover
    def found(handle, info, _):
        if info and info.contents.lidar_ip.decode() == args.lidar_ip and info.contents.dev_type == 9:
            if not handles:
                handles.append(handle)
                print('MID360', info.contents.sn.decode(), args.lidar_ip, flush=True)
                ready.set()

    @ack_type
    def ack(status, handle, response, _):
        result = {'status': status, 'ret_code': response.contents.ret_code if response else None,
                  'error_key': hex(response.contents.error_key) if response else None}
        print(json.dumps(result), flush=True)
        outcome.append(status == 0 and bool(response) and response.contents.ret_code == 0)
        done.set()

    @query_type
    def queried(status, handle, response, _):
        try:
            if status != 0 or not response:
                raise RuntimeError('query status=%s' % status)
            ret, count = struct.unpack('<BH', C.string_at(response, 3))
            if ret != 0 or count > 256:
                raise RuntimeError('query ret=%s count=%s' % (ret, count))
            offset, values = 3, {}
            for _index in range(count):
                key, size = struct.unpack('<HH', C.string_at(response + offset, 4))
                if size > 4096:
                    raise RuntimeError('invalid SDK response length')
                values[hex(key)] = C.string_at(response + offset + 4, size).hex()
                offset += 4 + size
            print(json.dumps(values, sort_keys=True), flush=True)
            outcome.append(True)
        except Exception as exc:
            print(str(exc), file=sys.stderr, flush=True)
            outcome.append(False)
        done.set()

    sdk.DisableLivoxSdkConsoleLogger()
    if not sdk.LivoxLidarSdkInit(args.config.encode(), args.host_ip.encode(), None):
        raise RuntimeError('SDK init failed; stop the driver and check the host IP/ports')
    try:
        sdk.SetLivoxLidarInfoChangeCallback(found, None)
        if not sdk.LivoxLidarSdkStart() or not ready.wait(args.timeout):
            raise RuntimeError('MID360 discovery timed out; check IP, cable and exclusive SDK access')
        handle = handles[0]
        if args.action == 'query':
            status = sdk.QueryLivoxLidarInternalInfo(handle, queried, None)
        elif args.action.startswith('boot-'):
            mode = {'boot-normal': 1, 'boot-wakeup': 2}[args.action]
            status = sdk.SetLivoxLidarWorkModeAfterBoot(handle, mode, ack, None)
        else:
            mode = {'normal': 1, 'wakeup': 2, 'sleep': 3}[args.action]
            status = sdk.SetLivoxLidarWorkMode(handle, mode, ack, None)
        if status != 0:
            raise RuntimeError('SDK rejected command: %s' % status)
        if not done.wait(args.timeout):
            raise RuntimeError('Command acknowledgement timed out; final device state is unknown')
        return 0 if outcome and outcome[0] else 1
    finally:
        sdk.LivoxLidarSdkUninit()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('MID360 control failed: %s' % exc, file=sys.stderr)
        sys.exit(1)
