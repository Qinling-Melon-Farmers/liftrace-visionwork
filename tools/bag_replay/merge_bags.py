#!/usr/bin/env python3
"""Merge sequentially split ROS1 bags into one bag for the replay toolkit.

`bag_replay.py` accepts a single bag, but board recordings are split at
512 MiB (`rosbag record --split`), so one run can span several parts.  This
tool re-serialises the parts in order into one bag so the toolkit can be used
unchanged.  Inputs are only read; the output must not exist beforehand.

Parts produced by `--split` are chronological, so messages are streamed in
input order instead of being sorted in memory (a full flight easily exceeds
available RAM once decompressed).  Out-of-order input is reported rather than
silently reordered.

Usage:
  python3 merge_bags.py OUTPUT.bag INPUT0.bag INPUT1.bag [...]
"""

import os
import sys

import rosbag


def main(argv):
    if len(argv) < 3:
        print(__doc__.strip())
        return 2
    out_path = argv[1]
    inputs = argv[2:]
    if os.path.exists(out_path):
        print("output exists; choose a new path: " + out_path)
        return 2
    for path in inputs:
        if os.path.abspath(path) == os.path.abspath(out_path):
            print("refusing to overwrite an input: " + path)
            return 2
        if not os.path.isfile(path):
            print("missing input: " + path)
            return 2

    total = 0
    out_of_order = 0
    last_stamp = None
    with rosbag.Bag(out_path, "w") as out:
        for path in inputs:
            with rosbag.Bag(path) as bag:
                for topic, msg, stamp in bag.read_messages():
                    total += 1
                    if last_stamp is not None and stamp < last_stamp:
                        out_of_order += 1
                    else:
                        last_stamp = stamp
                    out.write(topic, msg, stamp)
    print(
        "merged %d messages from %d bag(s) into %s"
        % (total, len(inputs), out_path)
    )
    if out_of_order:
        print(
            "WARNING: %d message(s) were older than the previous one; "
            "parts may not be chronological" % out_of_order
        )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
