#!/usr/bin/env python3
"""Isolated ROS visualization test. No hardware, goal or control publishers."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=int, default=600)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    port = 11431
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ.pop('ROS_IP', None)
    os.environ['ROS_LOG_DIR'] = str(out / 'roslog')
    children = []
    handles = []
    def start(command, name):
        handle = open(out / (name + '.log'), 'w')
        handles.append(handle)
        child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        children.append(child)
        return child
    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate(): return
            time.sleep(.05)
        raise AssertionError('condition timed out')
    report = {'input': 'synthetic isolated data', 'duration_requested': args.seconds}
    try:
        master = start(['roscore', '-p', str(port)], 'master')
        def ready():
            try: return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/view_test')[0] == 1
            except OSError: return False
        wait(ready)
        import rospy
        import rosgraph
        from geometry_msgs.msg import Point, PoseStamped
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Header
        from visualization_msgs.msg import Marker
        rospy.init_node('obstacle_view_test', disable_signals=True)
        start(['rosrun', 'tf2_ros', 'static_transform_publisher',
               '0', '0', '0', '0', '0', '0', '1', 'test_world', 'camera_init'], 'test_tf')
        cloud_pub = rospy.Publisher('/sdf_map/occupancy_inflate', PointCloud2, queue_size=1)
        path_pub = rospy.Publisher('/planning_vis/trajectory', Marker, queue_size=20)
        pose_pub = rospy.Publisher('/navigation/local_pose', PoseStamped, queue_size=1)
        received = {}
        rospy.Subscriber('/navigation/obstacle_blocks', Marker, lambda m: received.update(map=m), queue_size=1)
        rospy.Subscriber('/navigation/planned_path', Marker, lambda m: received.update(path=m), queue_size=1)
        node = start(['rosrun', 'uav_mission', 'obstacle_view_node'], 'converter')
        wait(lambda: cloud_pub.get_num_connections() and path_pub.get_num_connections())
        def send_cloud(points, frame='camera_init'):
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id=frame), points))
        send_cloud([(x*.1+.025, y*.1+.025, z*.1+.025)
                    for x in range(3) for y in range(3) for z in range(3)])
        wait(lambda: 'map' in received and len(received['map'].points) == 26)
        send_cloud([])
        wait(lambda: received['map'].action == Marker.DELETE)
        trajectory = Marker()
        trajectory.header.frame_id = 'camera_init'
        trajectory.pose.orientation.w = 1
        trajectory.id = 300
        trajectory.type = Marker.SPHERE_LIST
        trajectory.points = [Point(i/1000., .2*math.sin(i/500.), .5) for i in range(7001)]
        path_pub.publish(trajectory)
        wait(lambda: 'path' in received and len(received['path'].points) == 1000)
        assert received['path'].points[-1].x == 7
        ignored = Marker(id=400, action=Marker.DELETE)
        path_pub.publish(ignored)
        time.sleep(.2)
        assert received['path'].action == Marker.ADD
        trajectory.action = Marker.DELETE
        path_pub.publish(trajectory)
        wait(lambda: received['path'].action == Marker.DELETE)
        trajectory.action = Marker.ADD
        path_pub.publish(trajectory)
        wait(lambda: received['path'].action == Marker.ADD)
        late = rospy.wait_for_message('/navigation/planned_path', Marker, timeout=3)
        assert len(late.points) == 1000
        path_pub.publish(Marker(action=Marker.DELETEALL))
        wait(lambda: received['path'].action == Marker.DELETE)
        send_cloud([(0,0,0)], 'wrong_frame')
        time.sleep(1.2)
        assert received['map'].action == Marker.DELETE
        report['functional_checks'] = 'PASS: surface, empty clear, path limit/endpoints, ignored ID, DELETE, DELETEALL, latch, wrong frame'
        # Dense, repeatable map exercises the cap and software renderer.
        points = [(x*.05+.0125, y*.05+.0125, z*.05+.0125)
                  for x in range(-40,40) for y in range(80) for z in range(20)
                  if x < -32 or x > 31 or y < 4 or y > 75 or
                  (-8 < x < 8 and 28 < y < 40)]
        cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='camera_init'), points)
        trajectory.points = [Point(-1+.1*math.sin(i/300.), .6+2.8*i/1999., .5) for i in range(2000)]
        trajectory.action = Marker.ADD
        cloud_pub.publish(cloud)
        path_pub.publish(trajectory)
        wait(lambda: received['map'].action == Marker.ADD)
        config = Path(__file__).resolve().parents[1] / 'config/obstacle_4x4/view.rviz'
        os.environ['LP_NUM_THREADS'] = '2'
        rviz = start(['rosrun', 'rviz', 'rviz', '-d', str(config), '__name:=obstacle_rviz_test'], 'rviz')
        pose = PoseStamped(header=Header(frame_id='camera_init'))
        pose.pose.orientation.w = 1
        pose.pose.position = Point(0,.6,.5)
        samples = []
        began = time.monotonic()
        previous_ticks = None
        previous_time = began
        clock_hz = os.sysconf('SC_CLK_TCK')
        while time.monotonic()-began < args.seconds:
            elapsed = time.monotonic()-began
            assert rviz.poll() is None and node.poll() is None and master.poll() is None
            cloud_pub.publish(cloud)
            path_pub.publish(trajectory)
            pose.header.stamp = rospy.Time.now()
            pose_pub.publish(pose)
            if int(elapsed) % 10 == 0:
                raw = Path('/proc/%d/stat' % rviz.pid).read_text().split(') ',1)[1].split()
                ticks = int(raw[11])+int(raw[12])
                now=time.monotonic()
                cpu = 0 if previous_ticks is None else (ticks-previous_ticks)/clock_hz/(now-previous_time)*100
                previous_ticks,previous_time=ticks,now
                rss = int(raw[21])*os.sysconf('SC_PAGE_SIZE')/1024/1024
                samples.append({'seconds': round(elapsed,1), 'rss_mb':round(rss,2), 'cpu_percent':round(cpu,1)})
                print(json.dumps(samples[-1]),flush=True)
            assert len(received['map'].points) <= 12000
            time.sleep(1)
        system = rosgraph.Master('/view_test').getSystemState()
        subscriptions = [topic for topic,nodes in system[1] if '/obstacle_rviz_test' in nodes]
        assert not any(t in subscriptions for t in ['/sdf_map/occupancy_inflate','/cloud_registered','/livox/lidar'])
        report.update(duration_seconds=round(time.monotonic()-began,1), samples=samples,
                      rviz_subscriptions=subscriptions, blocks=len(received['map'].points),
                      resolution=received['map'].scale.x, path_points=len(received['path'].points),
                      result='PASS')
        warm = [s['rss_mb'] for s in samples if s['seconds'] >= 60]
        report['warm_rss_range_mb'] = [min(warm), max(warm)] if warm else []
        print('VERIFY PASS',flush=True)
    except BaseException as exc:
        report.update(result='FAIL', error=repr(exc))
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try: child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
        for handle in handles: handle.close()
        (out/'report.json').write_text(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
