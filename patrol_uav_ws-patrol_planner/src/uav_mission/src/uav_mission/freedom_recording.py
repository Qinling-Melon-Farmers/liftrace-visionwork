"""Automatic, fail-open FreeDOM evidence recording. No control publications/services."""
import argparse
from collections import Counter, deque
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
import uuid

DEFAULT_INPUT = '/cloud_registered_body'
MAP_TOPICS = ['/freedom/static_pointcloud', '/sdf_map/occupancy', '/sdf_map/occupancy_inflate']
CONTEXT_TOPICS = ['/tf', '/tf_static', '/Odometry', '/laserMapping/realtime',
                  '/mavros/vision_pose/pose', '/mavros/local_position/pose',
                  '/mavros/local_position/odom', '/mavros/state', '/board_trials/run_metadata']
FINAL_STATES = ('complete', 'partial', 'failed', 'not_applicable')


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=str, allow_nan=False))
    os.replace(str(tmp), str(path))


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def process_identity(pid):
    try:
        text = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        return None if text[0] == 'Z' else text[19]
    except (OSError, IndexError):
        return None


def topic_name(value):
    if not isinstance(value, str) or not value.startswith('/') or any(c.isspace() for c in value):
        raise ValueError('mapping topics must be absolute ROS names')
    return value


def configuration(run, settings, context, expected_mapper):
    options = settings.get('mapping_recording', {})
    if not isinstance(options, dict):
        raise ValueError('mapping_recording must be a mapping')
    limits = dict(buffer_bytes=64*1024**2, max_message_bytes=32*1024**2,
                  split_bytes=512*1024**2, min_free_bytes=2*1024**3, max_total_bytes=8*1024**3)
    for name in limits:
        value = options.get(name, limits[name])
        if type(value) is not int or value <= 0:
            raise ValueError('invalid mapping recorder limit: ' + name)
        limits[name] = value
    if limits['max_message_bytes'] > limits['buffer_bytes']:
        raise ValueError('mapping message limit exceeds buffer')
    return dict(version=1, run_id=uuid.uuid4().hex, run_dir=str(Path(run).resolve()),
                started_at=time.time(), expected_mapper=bool(expected_mapper),
                input_topic=topic_name(options.get('input_topic', DEFAULT_INPUT)),
                limits=limits, context=context, settings=settings)


class MappingRecording:
    """Supervisor-owned child. Every public operation is fail-open."""
    def __init__(self, run, config):
        self.out = Path(run)/'mapping'
        self.out.mkdir(exist_ok=False)
        self.config = config
        self.process = self.stream = None
        self.closed = False
        self.last_check = 0
        self.last_warning = None
        atomic_json(self.out/'metadata.json', config)
        atomic_json(self.out/'manifest.json', dict(config, status='starting', files=[], active=True))

    def start(self):
        self.stream = (self.out/'recorder.log').open('w')
        self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
            '--directory', str(self.out), '--parent', str(os.getpid()),
            '--parent-start', str(process_identity(os.getpid()))],
            env=dict(os.environ), stdout=self.stream, stderr=subprocess.STDOUT, start_new_session=True)
        atomic_json(self.out/'owner.json', dict(pid=self.process.pid, start=process_identity(self.process.pid)))
        until=time.monotonic()+3
        while time.monotonic()<until:
            if (self.out/'ready.json').is_file():return self
            if self.process.poll() is not None:
                raise RuntimeError('mapping recorder exited during startup: '+str(read_json(self.out/'manifest.json').get('reason','inspect recorder.log')))
            time.sleep(.05)
        atomic_json(self.out/'late_start.json',dict(reason='recorder readiness not confirmed before flight initialization'))
        print('MAPPING_RECORDING_WARNING: recorder startup late; flight unchanged',flush=True)
        return self

    def check(self):
        if self.closed or time.monotonic()-self.last_check < 1:
            return
        self.last_check = time.monotonic()
        try:
            manifest = read_json(self.out/'manifest.json')
            if self.process.poll() is not None and manifest.get('status') not in FINAL_STATES:
                manifest.update(status='failed', active=False, reason='recorder exited without finalization')
                atomic_json(self.out/'manifest.json', manifest)
            warning = manifest.get('reason') if manifest.get('status') in ('partial', 'failed') else None
            if warning and warning != self.last_warning:
                print('MAPPING_RECORDING_WARNING: '+str(warning), flush=True)
                self.last_warning = warning
        except Exception as exc:
            print('MAPPING_RECORDING_WARNING: status unavailable: '+str(exc), flush=True)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.process is not None and self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    # Do not kill a writer mid-close. Its own watchdog will finish.
                    print('MAPPING_RECORDING_WARNING: writer still finalizing; PID '+str(self.process.pid), flush=True)
            if self.process is not None and self.process.poll() is not None:
                manifest=read_json(self.out/'manifest.json')
                if manifest.get('status') not in FINAL_STATES:
                    manifest.update(status='failed',active=False,reason='recorder exited without finalization')
                    atomic_json(self.out/'manifest.json',manifest)
                warning=manifest.get('reason') if manifest.get('status') in ('partial','failed') else None
                if warning and warning!=self.last_warning:
                    print('MAPPING_RECORDING_WARNING: '+str(warning),flush=True)
                    self.last_warning=warning
        except Exception as exc:
            print('MAPPING_RECORDING_WARNING: close failed: '+str(exc), flush=True)
        finally:
            if self.stream:
                try:self.stream.close()
                except Exception as exc:print('MAPPING_RECORDING_WARNING: log close failed: '+str(exc),flush=True)


def start_mapping_recording(run, settings=None, context=None, expected_mapper=True):
    recorder = None
    try:
        recorder = MappingRecording(run, configuration(run, settings or {}, context or {}, expected_mapper))
        return recorder.start()
    except Exception as exc:
        print('MAPPING_RECORDING_WARNING: startup failed; flight unchanged: '+str(exc), flush=True)
        try:
            out = Path(run)/'mapping'
            if recorder is None and out.exists():return None  # Never overwrite another recorder's manifest.
            if recorder:recorder.close()
            if not out.exists():out.mkdir()
            manifest = read_json(out/'manifest.json')
            manifest.update(version=1, run_id=manifest.get('run_id', uuid.uuid4().hex),
                status='failed', active=False, started_at=manifest.get('started_at',time.time()),
                files=manifest.get('files',[]), reason=str(exc))
            atomic_json(out/'manifest.json', manifest)
        except Exception:
            pass
        return None


class Recorder:
    """Bounded callback queue and a single raw ROS1 bag writer."""
    def __init__(self, directory, config, bag_factory, message_class, disk_free=None):
        self.out, self.config = Path(directory), config
        self.bag_factory, self.message_class = bag_factory, message_class
        self.disk_free = disk_free or (lambda: shutil.disk_usage(self.out).free)
        self.cv = threading.Condition()
        self.queue, self.queued_bytes = deque(), 0
        self.received, self.written, self.dropped, self.gaps = Counter(), Counter(), Counter(), Counter()
        self.last_seq = {}
        self.errors, self.files, self.source_changes = [], [], []
        self.input_topics = [config['input_topic']]
        self.stopping = threading.Event()
        self.bag = self.active_path = None
        self.total_bytes, self.volume_bytes, self.index = 0, 0, 0
        self.last_status = 0
        self.effective = {}
        self.mapper_seen = False
        self.finished = False
        self.thread = None

    def problem(self, reason):
        reason=str(reason)[:400]
        with self.cv:
            if reason not in self.errors and len(self.errors)<100:self.errors.append(reason)

    def collect(self, topic, msg, stamp):
        raw, header = bytes(msg._buff), dict(msg._connection_header or {})
        size = len(raw)
        with self.cv:
            if self.stopping.is_set():return
            self.received[topic] += 1
            if header.get('type') == 'sensor_msgs/PointCloud2' and len(raw) >= 12:
                seq = struct.unpack_from('<I', raw)[0]
                key=(topic, header.get('callerid',''))
                previous=self.last_seq.get(key)
                if previous is not None and previous < seq-1:self.gaps[topic] += seq-previous-1
                self.last_seq[key]=seq
            if (size > self.config['limits']['max_message_bytes'] or
                    self.queued_bytes+size > self.config['limits']['buffer_bytes'] or len(self.queue) >= 10000):
                self.dropped[topic] += 1
                return
            self.queue.append((topic, raw, header, stamp))
            self.queued_bytes += size
            self.cv.notify()

    def seal(self):
        if self.bag is None:return
        bag, path = self.bag, self.active_path
        self.bag = self.active_path = None
        bag.close()  # Rename only after rosbag finalized its index/header.
        final = path.with_suffix('')
        os.replace(str(path), str(final))
        size = final.stat().st_size
        self.total_bytes += size
        self.files.append(dict(name=final.name, size=size, closed=True))
        self.volume_bytes = 0

    def write_one(self, item):
        topic, raw, header, stamp = item
        kind, md5 = header.get('type'), header.get('md5sum')
        cls = self.message_class(kind) if kind else None
        if cls is None or cls._md5sum != md5:
            raise ValueError('unknown/mismatched ROS message type: '+str(kind))
        if self.bag is None:
            self.index += 1
            self.active_path=self.out/('freedom_%05d.bag.active'%self.index)
            self.bag=self.bag_factory(str(self.active_path), 'w', compression='lz4', chunk_threshold=1024**2)
        self.bag.write(topic, (kind,raw,md5,cls), stamp, raw=True, connection_header=header)
        with self.cv:self.written[topic] += 1
        self.volume_bytes = self.active_path.stat().st_size
        if self.volume_bytes >= self.config['limits']['split_bytes']:self.seal()

    def worker(self):
        try:
            while True:
                with self.cv:
                    while not self.queue and not self.stopping.is_set():self.cv.wait(.2)
                    if not self.queue:break
                    item=self.queue.popleft();self.queued_bytes-=len(item[1])
                self.write_one(item)
        except Exception as exc:
            self.problem('writer failed: '+str(exc));self.stopping.set()
        finally:
            try:self.seal()
            except Exception as exc:self.problem('bag finalization failed: '+str(exc))

    def status(self, final=False):
        with self.cv:
            counts=dict(self.written);received=dict(self.received);dropped=dict(self.dropped);gaps=dict(self.gaps)
            errors=list(self.errors)
        missing=[topic for topic in self.input_topics+['/tf_static','/freedom/static_pointcloud'] if not counts.get(topic)]
        if final:
            if any(dropped.values()):errors.append('bounded queue/message limit dropped messages')
            if any(gaps.values()):errors.append('pointcloud sequence gaps observed')
            if self.queue:errors.append('queued data not written')
            mapping_present=self.config['expected_mapper'] or any(counts.get(topic) for topic in MAP_TOPICS)
            if mapping_present and missing:errors.append('missing required topics: '+', '.join(missing))
            if not mapping_present and not errors:
                state='not_applicable'
            else:state='partial' if errors else 'complete'
            if (not self.files or not any(counts.values())) and state!='not_applicable':state='failed'
        else:state='finalizing' if self.stopping.is_set() else 'recording' if counts else 'waiting'
        manifest=dict(version=1,run_id=self.config['run_id'],started_at=self.config['started_at'],
            updated_at=time.time(),status=state,active=not final,files=list(self.files),
            total_bytes=self.total_bytes+self.volume_bytes,received_counts=received,topic_counts=counts,
            dropped_counts=dropped,sequence_gaps=gaps,missing_required=missing,
            reason='; '.join(errors),input_topics=list(self.input_topics),map_topics=MAP_TOPICS,
            source_changes=list(self.source_changes),mapper_seen=self.mapper_seen,
            expected_mapper=self.config['expected_mapper'])
        atomic_json(self.out/'manifest.json', manifest)
        if final:atomic_json(self.out/'summary.json', manifest)
        return manifest

    def check_budget(self):
        if self.disk_free() < self.config['limits']['min_free_bytes']:
            self.problem('remaining disk below recording floor; flight unchanged');self.stopping.set()
        if self.total_bytes+self.volume_bytes >= self.config['limits']['max_total_bytes']:
            self.problem('mapping recording size budget reached; flight unchanged');self.stopping.set()
        if self.stopping.is_set():
            with self.cv:
                while self.queue:
                    item=self.queue.popleft();self.dropped[item[0]]+=1
                self.queued_bytes=0;self.cv.notify_all()


def run(directory, parent, parent_start):
    out=Path(directory)
    config=read_json(out/'metadata.json')
    recorder=None
    lock=None
    subscriptions={}
    try:
        lock=(out.parent.parent/'.freedom_mapping_recording.lock').open('a')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        versions=dict(recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        source=config.get('context',{}).get('source_root')
        if source:
            try:versions['source_head']=subprocess.check_output(['git','-C',source,'rev-parse','HEAD'],text=True,stderr=subprocess.DEVNULL,timeout=2).strip()
            except (OSError,subprocess.SubprocessError):versions['source_head']='unavailable'
        atomic_json(out/'versions.json',versions)
        import rospy
        import rosbag
        from roslib.message import get_message_class
        try:os.nice(10)
        except OSError:pass
        rospy.init_node('freedom_mapping_recorder',anonymous=True,disable_signals=True,disable_rosout=True)
        recorder=Recorder(out,config,rosbag.Bag,get_message_class)
        if recorder.disk_free() < config['limits']['min_free_bytes']:raise RuntimeError('insufficient disk for mapping recording')
        recorder.thread=threading.Thread(target=recorder.worker,name='mapping-bag-writer',daemon=True)
        recorder.thread.start()
        def subscribe(topic):
            if topic in subscriptions:return
            def callback(msg):
                try:recorder.collect(topic,msg,rospy.Time.now())
                except Exception as exc:recorder.problem('callback failed: '+str(exc))
            subscriptions[topic]=rospy.Subscriber(topic,rospy.AnyMsg,callback,queue_size=1,buff_size=32*1024**2)
        for topic in list(dict.fromkeys([config['input_topic']]+MAP_TOPICS+CONTEXT_TOPICS)):subscribe(topic)
        atomic_json(out/'ready.json',dict(ready=True,at=time.time()))
        def stop(*unused):
            recorder.stopping.set()
            with recorder.cv:recorder.cv.notify_all()
        signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
        rospy.on_shutdown(stop)
        while not recorder.stopping.is_set() and not rospy.is_shutdown():
            if process_identity(parent)!=parent_start:
                recorder.problem('supervisor disappeared');stop();break
            if rospy.has_param('/freedom/pointcloud_topic'):
                recorder.mapper_seen=True
                actual=topic_name(rospy.get_param('/freedom/pointcloud_topic'))
                if actual not in recorder.input_topics:
                    recorder.source_changes.append(dict(at=time.time(),previous=recorder.input_topics[-1],actual=actual))
                    recorder.input_topics.append(actual);subscribe(actual)
                    recorder.problem('input topic changed after initial subscription; coverage gap possible')
            if (out/'late_start.json').is_file():recorder.problem('initialization started before recorder readiness; coverage gap possible')
            configs=out/'configs';configs.mkdir(exist_ok=True)
            for name in ['runtime.yaml','control.yaml','overrides.yaml','run_metadata.json','profile.yaml','camera_info.json']:
                source=out.parent/name
                if source.is_file() and source.stat().st_size<=2*1024**2:
                    content=source.read_bytes();dest=configs/name
                    if not dest.is_file() or dest.read_bytes()!=content:
                        temp=dest.with_name(dest.name+'.tmp');temp.write_bytes(content);os.replace(str(temp),str(dest))
            for key in ['/freedom','/laserMapping','/common','/preprocess','/mapping','/publish','/lio_external_pose']:
                if rospy.has_param(key):recorder.effective[key]=rospy.get_param(key)
            atomic_json(out/'effective_parameters.json',recorder.effective)
            recorder.check_budget();recorder.status()
            time.sleep(1)
        for subscriber in subscriptions.values():subscriber.unregister()
        recorder.stopping.set()
        with recorder.cv:recorder.cv.notify_all()
        recorder.thread.join(timeout=25)
        if recorder.thread.is_alive():
            recorder.problem('writer still finalizing')
            recorder.status()
            recorder.thread.join()  # Remain alive/owned, never publish a false closed manifest.
        recorder.finished=True
        recorder.status(final=True)
    except Exception as exc:
        if recorder:
            recorder.problem(str(exc));recorder.stopping.set()
            with recorder.cv:recorder.cv.notify_all()
            if recorder.thread:recorder.thread.join(timeout=25)
            if not recorder.thread or not recorder.thread.is_alive():recorder.status(final=True)
        else:
            atomic_json(out/'manifest.json',dict(version=1,run_id=config.get('run_id',out.parent.name),
                started_at=config.get('started_at',time.time()),status='failed',active=False,files=[],reason=str(exc)))
        print('MAPPING_RECORDING_WARNING: '+str(exc),flush=True)
    finally:
        if lock:lock.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',required=True);parser.add_argument('--parent',type=int,required=True)
    parser.add_argument('--parent-start',required=True)
    args=parser.parse_args()
    run(args.directory,args.parent,args.parent_start)
