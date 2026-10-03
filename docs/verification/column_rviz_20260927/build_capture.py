import subprocess
from pathlib import Path
p=Path("/home/xhj/liftrace-worktrees/r2026-high-view-search/docs/verification/column_rviz_20260927")
flags=subprocess.check_output(["pkg-config","--cflags","--libs","OGRE","Qt5Widgets"],text=True).split()
subprocess.run(["g++","-shared","-fPIC","-std=c++14","-I/opt/ros/noetic/include","-I/usr/include/eigen3",str(p/"capture.cpp"),"-L/opt/ros/noetic/lib","-lrviz","-o","/home/xhj/rviz_capture.so"]+flags,check=True)
