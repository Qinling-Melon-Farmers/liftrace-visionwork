# Orange Pi 5 calibrated actuator_pwm

Mapping: req=1 rear -> physical Pin11 / pwmchip4 / febf0020.pwm (rear wiring must be confirmed); req=2 right -> Pin7 / pwmchip5 / febf0030.pwm; req=3 left -> Pin16 / pwmchip0 / fd8b0010.pwm.
Calibration initial/release ns: rear 700000/1700000; right 1000000/2100000; left 1100000/2100000. Period 20000000 ns, polarity normal.

Run init_pwm.sh as sudo before starting the node. It checks hardware addresses, exports channels, sets zero duty and disables outputs. It grants the invoking user write access. Do not run initialization while a servo node is running.

The ROS node resets all three servos at startup, then exposes /Servo (patrol_control/Servo). req=1/2/3 releases the corresponding compartment. Restarting the node resets all three again. Disabling PWM does not disconnect external battery power.

This deployment was copied only: no compilation or physical motion validation performed. A true service response is not physical position feedback. Verify mechanical movement separately.

Build from a fresh terminal:
```bash
source /opt/ros/noetic/setup.bash
source ~/liftrace_r64_onboard_405bda42/vision_ws/devel/setup.bash
cd ~/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner
catkin_make -j2 -l2
source devel/setup.bash
rospack find actuator_pwm
rossrv show patrol_control/Servo
```

Start only when wiring and power are ready (startup moves servos):
```bash
sudo bash src/actuator_pwm/init_pwm.sh
roslaunch actuator_pwm launch_all.launch
```

In another terminal:
```bash
source ~/liftrace_r64_onboard_405bda42/patrol_uav_ws-patrol_planner/devel/setup.bash
rosservice type /Servo
rosservice call /Servo "req: 2"  # right Pin7
rosservice call /Servo "req: 3"  # left Pin16
# Confirm rear wiring to Pin11 before testing:
rosservice call /Servo "req: 1"
```
Execute one request at a time and inspect the corresponding mechanism. Repeated release requests do not reset it first.
