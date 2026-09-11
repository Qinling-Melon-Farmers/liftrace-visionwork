"""Bounded, opt-in observer samples. No ROS or simulator startup."""
import json
from pathlib import Path
from dataclasses import asdict
import cv2
import numpy as np


class Recorder:
    def __init__(self, directory, interval=6., limit=120):
        if interval <= 0 or not np.isfinite(interval) or int(limit) != limit or limit <= 0:
            raise ValueError('invalid recording bounds')
        self.directory = Path(directory)
        if not self.directory.is_absolute():
            raise ValueError('record directory must be absolute')
        self.directory.mkdir(parents=True, exist_ok=True)
        self.interval, self.limit = interval, int(limit)
        self.count, self.last_stamp = 0, -np.inf
        self.generation = None
        # Never append a restarted observer's samples onto a previous session.
        self.events = (self.directory/'status.jsonl').open('x', encoding='utf-8', buffering=1)

    def status(self, result):
        self.events.write(json.dumps(result, sort_keys=True)+'\n')

    def sample(self, result, image, camera, matrix, points, map_stamp, states, config):
        stamp = result['image_stamp']
        generation = result['generation']
        if generation != self.generation:
            self.last_stamp = -np.inf
            self.generation = generation
        if self.count >= self.limit or stamp-self.last_stamp < self.interval:
            return False
        stem = '%04d' % self.count
        if not cv2.imwrite(str(self.directory/(stem+'.png')), image, [cv2.IMWRITE_PNG_COMPRESSION,1]):
            raise IOError('failed to save observer image')
        np.savez(self.directory/(stem+'.npz'),
                 map_points=np.empty((0,3),np.float32) if points is None else points.astype(np.float32),
                 camera_to_map=matrix, state_grid=states)
        metadata = {'camera':asdict(camera), 'config':asdict(config), 'result':result,
                    'map_stamp':map_stamp, 'point_frame':config.frame_id,
                    'image':stem+'.png','arrays':stem+'.npz'}
        (self.directory/(stem+'.json')).write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
        self.count += 1
        self.last_stamp = stamp
        return True

    def close(self):
        self.events.close()
