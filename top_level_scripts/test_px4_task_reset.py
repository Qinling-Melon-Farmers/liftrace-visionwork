#!/usr/bin/env python3
"""Execute the patched production counter handoff with historical/new resets."""
import argparse,subprocess,tempfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--px4-root',type=Path,required=True);a=p.parse_args()
s=(a.px4_root/'src/modules/flight_mode_manager/FlightModeManager.cpp').read_text()
start=s.index('\tif (isAnyTaskActive()) {',s.index('FlightTaskError FlightModeManager::switchTask(FlightTaskIndex'))
end=s.index('\n\tif (_initTask',start);block=s[start:end]
code='''#include <cassert>
struct Counters {int xy=0,vxy=0,z=0,vz=0,heading=0;};
struct vehicle_local_position_s {int xy_reset_counter=2,vxy_reset_counter=2,z_reset_counter=2,vz_reset_counter=2,heading_reset_counter=1;};
struct Sub {void copy(vehicle_local_position_s *p){*p=vehicle_local_position_s{};}};
struct Task {int getTrajectorySetpoint(){return 0;} Counters getResetCounters(){Counters c;c.heading=0;return c;}};
bool active=false;bool isAnyTaskActive(){return active;}
int main(){
for(int existing=0;existing<2;existing++){
active=existing;int last_setpoint=0;Counters last_reset_counters;Task task;struct {Task *task;} _current_task{&task};Sub _vehicle_local_position_sub;
'''+block+'''
int current_heading_counter=1;double yaw=1.5708;
if(current_heading_counter!=last_reset_counters.heading)yaw+=1.5706;
if(existing){assert(yaw>3.14);}else{assert(yaw==1.5708);assert(last_reset_counters.xy==2);}
last_reset_counters.heading=current_heading_counter;
current_heading_counter=2;
assert(current_heading_counter!=last_reset_counters.heading);
}
}
'''
with tempfile.TemporaryDirectory() as d:
 f=Path(d)/'test.cpp';f.write_text(code);exe=Path(d)/'test';subprocess.run(['g++','-std=c++14','-fsanitize=undefined',str(f),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS: new task does not replay history; inherited and future resets still apply')
