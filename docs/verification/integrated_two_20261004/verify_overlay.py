#!/usr/bin/env python3
"""Probe and fully decode one generated video; never modify its source media."""
import argparse
import csv
import json
import subprocess
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('video', type=Path)
a = p.parse_args()
target = a.video.with_suffix('.validation.json')
if target.exists():
    raise ValueError('Validation already exists; inspect it before repeating')
meta = json.loads(a.video.with_suffix('.json').read_text())
rows = list(csv.DictReader(a.video.with_suffix('.frames.csv').open()))
probe_cmd = ['ffprobe', '-v', 'error', '-threads', '1', '-select_streams', 'v:0',
             '-show_entries', 'stream=codec_name,width,height,r_frame_rate,nb_frames,duration:format=duration,size',
             '-of', 'json', str(a.video)]
probe = subprocess.run(probe_cmd, capture_output=True, text=True)
decode_cmd = ['ffmpeg', '-nostdin', '-v', 'error', '-xerror', '-threads', '1',
              '-filter_threads', '1', '-i', str(a.video), '-map', '0:v:0',
              '-an', '-threads', '1', '-f', 'null', '-']
decode = subprocess.run(decode_cmd, capture_output=True, text=True)
info = json.loads(probe.stdout) if probe.returncode == 0 else {}
stream = next(iter(info.get('streams', [])), {})
checks = dict(ffprobe_exit_zero=probe.returncode == 0,
              full_decode_exit_zero=decode.returncode == 0,
              full_decode_no_error_text=not decode.stderr.strip(),
              final_source_frame_shown=bool(meta.get('final_source_frame_shown')),
              frame_sidecar_matches=len(rows) == meta['output_frames'] == int(stream.get('nb_frames', -1)),
              timestamp_match_within_30ms=all(not r['match_delta_ms'] or abs(float(r['match_delta_ms'])) <= 30 for r in rows))
result = dict(video=str(a.video), status='PASS' if all(checks.values()) else 'FAIL',
              checks=checks, ffprobe_command=probe_cmd, ffprobe_exit=probe.returncode,
              ffprobe=info, ffprobe_stderr=probe.stderr, decode_command=decode_cmd,
              full_decode_exit=decode.returncode, full_decode_stderr=decode.stderr,
              sidecar_rows=len(rows))
with target.open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['status'] == 'PASS' else 1)
