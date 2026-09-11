#!/usr/bin/env python3
"""Offline comparison on saved samples; never starts ROS, PX4 or Gazebo.

Run in the existing rl_drone environment with OPENBLAS_NUM_THREADS=1 and
OMP_NUM_THREADS=1. Raw comparison arrays remain under ignored logs/_artifacts.
"""
import argparse
import json
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import time
import types

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASELINE = '80445c055f4e12ee278cc512ae06d58b298b76ca'
PACKAGE = 'vision_ws/src/uav_coverage_memory/src'
RUNS = ['coverage_seed32_reference070_20260911_174716',
        'coverage_seed32_reference080_20260911_182732',
        'coverage_seed34_reference080_wait20_20260911_190504']


def load_module(version):
    if version == 'baseline':
        source = subprocess.check_output(['git', 'show', BASELINE+':'+PACKAGE+'/uav_coverage_memory/memory.py'], cwd=ROOT, text=True)
        module = types.ModuleType('baseline_memory')
        sys.modules[module.__name__] = module
        exec(compile(source, '<baseline_memory>', 'exec'), module.__dict__)
        return module
    sys.path.insert(0, str(ROOT/PACKAGE))
    import uav_coverage_memory.memory as module
    return module


def worker(version, mode, output):
    cv2.setNumThreads(1)
    module = load_module(version)
    rows, states, counts, seen = [], [], [], []
    if mode == 'historical':
        for run in RUNS:
            folder = ROOT/'logs'/run/'coverage'
            memory = None
            for path in sorted(folder.glob('[0-9]*.json')):
                sample = json.loads(path.read_text())
                camera = module.Camera(**sample['camera'])
                if memory is None:
                    memory = module.Memory(module.Config(**sample['config']))
                with np.load(folder/sample['arrays']) as arrays:
                    points = arrays['map_points'].astype(np.float64)
                    transform = arrays['camera_to_map']
                image = cv2.imread(str(folder/sample['image']))
                r = sample['result']
                args = dict(image=image, camera=camera, camera_to_map=transform,
                    stamp=r['image_stamp'], now=r['image_stamp']+r['image_age'],
                    tf_stamp=r['tf_stamp'], camera_stamp=r['camera_stamp'],
                    image_frame=camera.frame_id, map_points=points,
                    map_stamp=sample['map_stamp'], map_frame=sample['point_frame'])
                start, cpu = time.perf_counter(), time.process_time()
                result = memory.observe(**args)
                rows.append(dict(run=run, sample=path.stem, points=len(points),
                    wall_ms=(time.perf_counter()-start)*1000, cpu_ms=(time.process_time()-cpu)*1000,
                    result=result))
                states.append(memory.state.copy());counts.append(memory.count.copy())
                seen.append(memory.last_seen.copy())
        np.savez(output.with_suffix('.npz'), states=np.asarray(states), counts=np.asarray(counts), seen=np.asarray(seen))
    else:
        # Worst projection workload: 500k occupied points in the calibrated FOV.
        # Generated fixture, not additional flight data or an obstacle layout.
        sample = json.loads((ROOT/'logs'/RUNS[0]/'coverage/0040.json').read_text())
        camera = module.Camera(**sample['camera'])
        memory = module.Memory(module.Config(**sample['config']))
        transform = np.diag([1., -1., -1., 1.]);transform[2,3] = 2.
        local = np.random.RandomState(18).uniform([-.3,-.2,.6],[.3,.2,1.2],(500000,3))
        points = local @ transform[:3,:3].T+transform[:3,3]
        del local
        image = np.random.RandomState(19).randint(30,225,(720,1280,3),dtype=np.uint8)
        for i in range(10):
            stamp = 10+i*.25
            start, cpu = time.perf_counter(), time.process_time()
            result = memory.observe(image,camera,transform,stamp,stamp+.01,stamp,stamp,
                                    camera.frame_id,points,stamp,memory.config.frame_id)
            rows.append(dict(wall_ms=(time.perf_counter()-start)*1000,
                cpu_ms=(time.process_time()-cpu)*1000, result=result))
        np.savez(output.with_suffix('.npz'), states=memory.state,counts=memory.count,seen=memory.last_seen)
    output.write_text(json.dumps(dict(rows=rows, peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024),indent=2))


def stats(rows, key):
    values = [r[key] for r in rows]
    return dict(zip(['p50','p95','max'], [float(x) for x in np.percentile(values,[50,95,100])]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker', choices=['baseline','current'])
    parser.add_argument('--mode', choices=['historical','stress'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--report', type=Path, default=Path(__file__).with_name('benchmark.json'))
    args = parser.parse_args()
    if args.worker:
        worker(args.worker, args.mode, args.output)
        return
    artifacts = ROOT/'logs/_artifacts/coverage_resources_20260911'/('py'+str(sys.version_info.major)+str(sys.version_info.minor))
    artifacts.mkdir(parents=True,exist_ok=True)
    report = dict(baseline=BASELINE, platform=platform.platform(), python=sys.version.split()[0],
        numpy=np.__version__, opencv=cv2.__version__, native_threads=1,
        scope='OFFLINE_X86_CORE_ONLY_NOT_ONBOARD_OR_END_TO_END_TIMING', modes={})
    for mode in ['historical','stress']:
        data = {}
        for version in ['baseline','current']:
            output = artifacts/(mode+'_'+version+'.json')
            subprocess.run([sys.executable,__file__,'--worker',version,'--mode',mode,'--output',str(output)],
                check=True, env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
            data[version] = json.loads(output.read_text())
        a,b = data['baseline']['rows'],data['current']['rows']
        result_mismatches = sum(x['result'] != y['result'] for x,y in zip(a,b))
        with np.load(artifacts/(mode+'_baseline.npz')) as old, np.load(artifacts/(mode+'_current.npz')) as new:
            array_mismatches = {key:int(np.count_nonzero(old[key] != new[key])) for key in old.files}
        report['modes'][mode] = dict(samples=len(a), result_mismatches=result_mismatches,
            array_mismatches=array_mismatches,
            metrics={version:dict(wall_ms=stats(d['rows'],'wall_ms'),cpu_ms=stats(d['rows'],'cpu_ms'),
                peak_process_rss_mib=d['peak_process_rss_mib']) for version,d in data.items()})
        if mode == 'historical':
            report['modes'][mode]['runs'] = {run:dict(samples=sum(r['run']==run for r in a),
                max_points=max(r['points'] for r in a if r['run']==run)) for run in RUNS}
        if len(a)!=len(b) or result_mismatches or any(array_mismatches.values()):
            raise RuntimeError('observation equivalence failed: '+mode)
    path = args.report
    path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__ == '__main__':
    main()
