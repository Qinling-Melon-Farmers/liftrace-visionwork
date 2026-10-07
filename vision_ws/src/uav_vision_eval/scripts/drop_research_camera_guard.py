#!/usr/bin/env python3
"""ROS-package entry point for the opt-in research camera observer."""
from pathlib import Path
import runpy
import sys
root = Path(__file__).resolve().parents[4]
script = root / 'docs/verification/drop_precision_ab_20261007/camera_guard.py'
sys.path.insert(0, str(script.parent))
runpy.run_path(str(script), run_name='__main__')
