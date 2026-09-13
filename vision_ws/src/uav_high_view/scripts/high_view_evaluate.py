#!/usr/bin/env python3
"""Evaluate annotated high-view segments offline; never starts a detector/ROS."""
import argparse
import json
from pathlib import Path
from uav_high_view.evaluation import evaluate_segments


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--annotations', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    source = Path(args.annotations)
    if source.stat().st_size > 8*1024*1024:
        raise ValueError('annotation batch exceeds 8 MiB; split by complete segments')
    config = json.loads(Path(args.config).read_text())
    segments = json.loads(source.read_text())
    result = evaluate_segments(segments, config['weights'])
    with open(args.output, 'x') as output:
        json.dump(result, output, indent=2, allow_nan=False)


if __name__ == '__main__': main()
