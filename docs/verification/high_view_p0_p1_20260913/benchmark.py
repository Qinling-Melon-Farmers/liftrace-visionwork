#!/usr/bin/env python3
"""Synthetic core-only benchmark; no ROS, detector, map conversion or disk input in timing."""
import json
import math
from pathlib import Path
import resource
import statistics
import sys
import time
from uav_high_view.core import Catalog, Config, Epoch, Key, Observation, Edge, rank_routes, label, NS


def main():
    config_path = Path(__file__).resolve().parents[3]/'vision_ws/src/uav_high_view/config/offline.json'
    config = json.loads(config_path.read_text())
    weights = config['weights']
    epoch = Epoch('synthetic', 'lio', 'camera')

    def populated(count, samples):
        c = Catalog(Config(**config['catalog']), weights); c.reset(epoch)
        for j in range(samples):
            t = 10*NS+j*NS//4
            for i in range(1, count+1):
                o = Observation(epoch,Key(i,NS),t,'camera_init',list(weights)[(i-1)%5],
                                (float(i),0.),.9,.9,.1,2.6)
                c.observe(o,t)
        return c, t

    c, now = populated(5,3)
    hints = c.hints(now)
    names = ['@start']+[label(h.key) for h in hints]+['@exit']
    edges = [Edge(a,b,1.,now,epoch,True) for a in names for b in names if a!=b]
    services = {(label(h.key),s):2. for h in hints for s in [1,2,3]}

    def route():
        return rank_routes(hints,weights,edges,epoch,now,3,services,600.,180.,20.)

    def catalog():
        c,t = populated(16,8)
        assert len(c.hints(t))==16

    def measure(fn, count):
        for _ in range(20): fn()
        samples=[];cpu_start=time.process_time()
        for _ in range(count):
            begin=time.perf_counter();fn();samples.append((time.perf_counter()-begin)*1000)
        cpu=(time.process_time()-cpu_start)*1000/count
        samples.sort()
        return dict(iterations=count,median_ms=statistics.median(samples),
                    p95_ms=samples[math.ceil(.95*len(samples))-1],cpu_ms_per_call=cpu)

    result=dict(scope='SYNTHETIC_X86_CORE_ONLY_NOT_ORANGEPI_OR_ROS',python=sys.version.split()[0],
                catalog_16x8=measure(catalog,500),routes_5p3=measure(route,1000),
                process_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
