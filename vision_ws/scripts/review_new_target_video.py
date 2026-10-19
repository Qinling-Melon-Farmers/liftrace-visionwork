#!/usr/bin/env python3
"""Extract review-only frames; labels are supplied by a reviewer, never a model."""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--video', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--hz', type=float, default=1.)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'frames').mkdir()
    cap = cv2.VideoCapture(str(a.video))
    if not cap.isOpened():
        raise RuntimeError('Cannot decode video')
    fps = cap.get(cv2.CAP_PROP_FPS)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    stride = max(1, round(fps/a.hz))
    samples, tiles = [], []
    frame_id = 0
    while True:
        ok, im = cap.read()
        if not ok:
            break
        if frame_id % stride == 0:
            name = f'new_target_f{frame_id:06d}.jpg'
            cv2.imwrite(str(a.output/'frames'/name), im, [cv2.IMWRITE_JPEG_QUALITY, 96])
            h, w = im.shape[:2]
            samples.append({'frame_id': frame_id, 'time_s': frame_id/fps,
                            'image': str(a.output/'frames'/name), 'width': w, 'height': h,
                            'review_status': 'pending', 'objects': None,
                            'source_group': a.video.stem, 'split': 'train'})
            tile = np.full((315, 480, 3), 25, dtype=np.uint8)
            scale = min(480/w, 270/h)
            thumb = cv2.resize(im, (round(w*scale), round(h*scale)))
            th, tw = thumb.shape[:2]
            tile[(270-th)//2:(270-th)//2+th, (480-tw)//2:(480-tw)//2+tw] = thumb
            cv2.putText(tile, f'{len(samples):03d}  f{frame_id:06d}  {frame_id/fps:.2f}s',
                        (8, 289), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,255), 1)
            tiles.append(tile)
        frame_id += 1
    cap.release()
    pages = []
    for start in range(0, len(tiles), 12):
        subset = tiles[start:start+12]
        canvas = np.full((math.ceil(len(subset)/3)*315, 1440, 3), 25, dtype=np.uint8)
        for j, tile in enumerate(subset):
            canvas[(j//3)*315:(j//3+1)*315, (j%3)*480:(j%3+1)*480] = tile
        page = f'contact_{start//12+1:02d}.jpg'
        cv2.imwrite(str(a.output/page), canvas, [cv2.IMWRITE_JPEG_QUALITY, 94])
        pages.append(page)
    manifest = {'source_video': str(a.video), 'fps': fps, 'declared_frames': total,
                'decoded_frames': frame_id, 'duration_s': frame_id/fps,
                'sample_hz': a.hz, 'actual_stride': stride, 'sample_count': len(samples),
                'names': {0:'bridge',1:'panzer',2:'pillbox',3:'tent',4:'red_cross'},
                'label_scope': 'center black circle and internal graphic for all five classes, including red_cross; exclude outer rings and board',
                'policy': 'Entire video train only; unknown labels excluded pending review; H negative only when all visible targets reviewed.',
                'samples': samples, 'contact_pages': pages}
    (a.output/'review_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    content = '<!doctype html><meta charset="utf-8"><h1>New target video: pending human review</h1>'
    content += '<p>Mapping: bridge=0 panzer=1 pillbox=2 tent=3 red_cross=4. No inferred labels.</p>'
    content += ''.join(f'<p><a href="{page}"><img width="1200" src="{page}"></a></p>' for page in pages)
    content += ''.join(f'<p>{i+1:03d} {s["time_s"]:.2f}s <a href="frames/{Path(s["image"]).name}">original frame {s["frame_id"]}</a></p>' for i,s in enumerate(samples))
    (a.output/'index.html').write_text(content)
    print(json.dumps({k:v for k,v in manifest.items() if k!='samples'}, indent=2))


if __name__ == '__main__':
    main()
