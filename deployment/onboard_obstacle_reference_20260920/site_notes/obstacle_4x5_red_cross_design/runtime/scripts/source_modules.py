import sys
from pathlib import Path

import rospkg

source_scripts = Path(rospkg.RosPack().get_path("uav_mission")) / "scripts"
sys.path.insert(0, str(source_scripts))
