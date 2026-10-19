"""Offline recording state tests: no ROS imports, compilation or device access."""
import importlib.util
import io
import fcntl
import subprocess
import sys
import json
from pathlib import Path
import struct
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SOURCE=Path(__file__).resolve().parents[1]/'src/uav_mission/freedom_recording.py'
spec=importlib.util.spec_from_file_location('freedom_recording_tested',SOURCE)
rec=importlib.util.module_from_spec(spec);spec.loader.exec_module(rec)

class FakeBag:
    fail_write=False
    fail_close=False
    def __init__(self,path,*args,**kwargs):
        self.path=Path(path);self.path.write_bytes(b'HEADER');self.rows=[]
    def write(self,topic,message,stamp,**kwargs):
        if self.fail_write:raise OSError('write fixture')
        self.rows.append((topic,message,stamp,kwargs))
        with self.path.open('ab') as stream:stream.write(message[1])
    def close(self):
        if self.fail_close:raise OSError('close fixture')
        with self.path.open('ab') as stream:stream.write(b'INDEX')

class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.out=Path(self.tmp.name)
        self.config=rec.configuration(self.out,{},dict(route='fixture'),True)
        self.r=rec.Recorder(self.out,self.config,FakeBag,lambda kind:SimpleNamespace(_md5sum='md5'),lambda:10**12)
    def tearDown(self):self.tmp.cleanup()
    def message(self,topic,seq=1,payload=b'data'):
        msg=SimpleNamespace(_buff=struct.pack('<III',seq,0,0)+payload,
            _connection_header=dict(type='sensor_msgs/PointCloud2',md5sum='md5',callerid='/fixture',latching='1'))
        self.r.collect(topic,msg,42);return msg
    def drain(self):
        self.r.stopping.set();self.r.worker();return self.r.status(final=True)
    def test_raw_full_rate_connection_headers_and_completion(self):
        msg=self.message(rec.DEFAULT_INPUT)
        self.r.write_one(self.r.queue.popleft());self.r.queued_bytes=0
        raw=self.r.bag.rows[0]
        self.assertEqual(raw[1][1],msg._buff);self.assertTrue(raw[3]['raw'])
        self.assertEqual(raw[3]['connection_header']['latching'],'1')
        self.message('/tf_static');self.message('/freedom/static_pointcloud')
        result=self.drain()
        self.assertEqual(result['status'],'complete');self.assertFalse(result['active'])
        self.assertTrue((self.out/result['files'][0]['name']).read_bytes().endswith(b'INDEX'))
        self.assertEqual(result['topic_counts'][rec.DEFAULT_INPUT],1)
    def test_split_preserves_all_messages(self):
        self.config['limits']['split_bytes']=1
        for topic in [rec.DEFAULT_INPUT,'/tf_static','/freedom/static_pointcloud']:self.message(topic)
        result=self.drain();self.assertEqual(result['status'],'complete');self.assertEqual(len(result['files']),3)
        self.assertFalse(list(self.out.glob('*.active')))
    def test_queue_limit_and_sequence_gap_report_partial(self):
        self.config['limits']['buffer_bytes']=40
        self.message(rec.DEFAULT_INPUT,1);self.message(rec.DEFAULT_INPUT,3);self.message(rec.DEFAULT_INPUT,4)
        self.assertLessEqual(self.r.queued_bytes,40)
        result=self.drain();self.assertEqual(result['status'],'partial')
        self.assertEqual(result['sequence_gaps'][rec.DEFAULT_INPUT],1)
        self.assertTrue(result['dropped_counts']);self.assertIn('/tf_static',result['missing_required'])
    def test_message_limit_rejects_without_queue_growth(self):
        self.config['limits']['max_message_bytes']=4
        self.message(rec.DEFAULT_INPUT);self.assertEqual(self.r.queued_bytes,0)
        self.assertEqual(self.drain()['status'],'failed')
    def test_write_failure_never_reports_complete(self):
        self.message(rec.DEFAULT_INPUT)
        with patch.object(FakeBag,'fail_write',True):result=self.drain()
        self.assertNotEqual(result['status'],'complete');self.assertIn('write fixture',result['reason'])
    def test_close_failure_keeps_active_unlisted(self):
        self.message(rec.DEFAULT_INPUT)
        with patch.object(FakeBag,'fail_close',True):result=self.drain()
        self.assertEqual(result['status'],'failed');self.assertEqual(result['files'],[])
        self.assertTrue(list(self.out.glob('*.bag.active')));self.assertFalse(list(self.out.glob('*.bag')))
    def test_disk_floor_and_total_stop_only_recorder(self):
        for reason in ['disk','total']:
            with self.subTest(reason=reason):
                self.r.stopping.clear();self.message(rec.DEFAULT_INPUT)
                if reason=='disk':self.r.disk_free=lambda:0
                else:self.r.disk_free=lambda:10**12;self.r.total_bytes=10**12
                self.r.check_budget();self.assertTrue(self.r.stopping.is_set());self.assertFalse(self.r.queue)
                self.assertTrue(self.r.errors)
    def test_observation_without_map_not_applicable_but_errors_visible(self):
        self.config['expected_mapper']=False;self.message('/tf_static')
        self.assertEqual(self.drain()['status'],'not_applicable')
        self.r.problem('disk failed');self.assertEqual(self.r.status(final=True)['status'],'partial')
    def test_observation_with_external_map_still_requires_mapping_evidence(self):
        self.config['expected_mapper']=False;self.message('/freedom/static_pointcloud')
        result=self.drain();self.assertEqual(result['status'],'partial')
        self.assertIn(rec.DEFAULT_INPUT,result['missing_required'])

    def test_stop_drains_queue_deterministically(self):
        thread=threading.Thread(target=self.r.worker);thread.start()
        for seq in range(1,31):self.message(rec.DEFAULT_INPUT,seq)
        self.r.stopping.set()
        with self.r.cv:self.r.cv.notify_all()
        thread.join(2);self.assertFalse(thread.is_alive())
        result=self.r.status(final=True);self.assertEqual(result['topic_counts'][rec.DEFAULT_INPUT],30)
    def test_startup_failure_is_fail_open_and_auditable(self):
        with patch.object(rec.subprocess,'Popen',side_effect=OSError('spawn failure')),patch('sys.stdout',new_callable=io.StringIO) as output:
            self.assertIsNone(rec.start_mapping_recording(self.out))
        manifest=rec.read_json(self.out/'mapping/manifest.json')
        self.assertEqual(manifest['status'],'failed');self.assertIn('flight unchanged',output.getvalue())
    def test_invalid_limits_never_overwrite_existing_recording(self):
        folder=self.out/'mapping';folder.mkdir();rec.atomic_json(folder/'manifest.json',dict(original=True))
        self.assertIsNone(rec.start_mapping_recording(self.out,dict(mapping_recording=dict(buffer_bytes=-1))))
        self.assertEqual(rec.read_json(folder/'manifest.json'),dict(original=True))
    def test_close_log_failure_does_not_escape(self):
        handle=rec.MappingRecording(self.out,self.config)
        handle.stream=SimpleNamespace(close=lambda:(_ for _ in ()).throw(OSError('close')))
        handle.close();self.assertTrue(handle.closed)
    def test_independent_process_lock_excludes_recorder_without_ros(self):
        run=self.out/'logs'/'run';folder=run/'mapping';folder.mkdir(parents=True)
        config=rec.configuration(run,{},dict(route='fixture'),True);rec.atomic_json(folder/'metadata.json',config)
        lock=folder.parent.parent/'.freedom_mapping_recording.lock'
        code="import fcntl,sys;f=open(sys.argv[1],'a');fcntl.flock(f,fcntl.LOCK_EX);print('locked',flush=True);sys.stdin.readline()"
        child=subprocess.Popen([sys.executable,'-c',code,str(lock)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(),'locked')
            rec.run(folder,0,'0')
            self.assertEqual(rec.read_json(folder/'manifest.json')['status'],'failed')
            self.assertIn('unavailable',rec.read_json(folder/'manifest.json')['reason'])
            inode=lock.stat().st_ino
        finally:child.communicate('\n',timeout=5)
        with lock.open('a') as stream:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.assertEqual(lock.stat().st_ino,inode)

    def test_close_reports_failure_discovered_during_finalization(self):
        handle=rec.MappingRecording(self.out,self.config)
        handle.process=SimpleNamespace(poll=lambda:1)
        rec.atomic_json(handle.out/'manifest.json',dict(status='partial',active=False,reason='bag finalization failed'))
        with patch('sys.stdout',new_callable=io.StringIO) as output:handle.close()
        self.assertIn('bag finalization failed',output.getvalue());self.assertTrue(handle.closed)

    def test_dead_child_marks_incomplete(self):
        handle=rec.MappingRecording(self.out,self.config);handle.process=SimpleNamespace(poll=lambda:1)
        handle.check();self.assertEqual(rec.read_json(handle.out/'manifest.json')['status'],'failed')

if __name__=='__main__':unittest.main()
