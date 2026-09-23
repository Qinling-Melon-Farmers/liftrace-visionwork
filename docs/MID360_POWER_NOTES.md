# MID360 control (2026-09-11)

Added only an independent SDK2 control script and two launch files; no mapping,
navigation or existing driver logic changed. No compilation performed.

Start scanning and publishing CustomMsg (replaces, do not duplicate, msg_MID360):

```bash
roslaunch uav_mission mid360_start.launch
```

After stopping the driver, enter SDK `WakeUp` mode (2):

```bash
roslaunch uav_mission mid360_standby.launch
```

This one-shot command checks the device ACK. It is not a power switch and does
not promise a particular motor state, wattage or temperature. Stopping the ROS
driver alone does not request standby. Stop flight/mapping operation before use.

The device 47MDM540020075 at 192.168.1.175 reports firmware bytes 0d120215,
build date 2023/12/19. Actual SDK command results:

* Set boot mode WakeUp: status=0, ret_code=32, error_key=0x0020 (rejected).
* Set runtime Sleep (3): status=0, ret_code=3, error_key=0x001a (rejected).
* Set runtime WakeUp (2): status=0, ret_code=0. Query returned mode 0x001a=02
  and current work state 0x8006=02.

Power-on standby has NOT been configured. A firmware/device capability check
with Livox is needed for persistent boot behavior; do not assume an upgrade
will enable it. A host boot service would only act after host boot/network
readiness, not prevent initial scanning. No service or firmware was changed.

Full control utility supports query, normal, wakeup, sleep, boot-normal and
boot-wakeup, with rejected operations returning nonzero. The latter modes are
exposed for diagnostics, not a claim of support on this device. SDK access is
exclusive: the script refuses occupied driver ports. Use the existing installed
shared SDK library and the driver's JSON configuration; no Python packages needed.
