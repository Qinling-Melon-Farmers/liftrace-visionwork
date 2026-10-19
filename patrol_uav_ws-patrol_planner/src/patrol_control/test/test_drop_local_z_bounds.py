"""Execute the production startup guard with signed calibrated delivery Z."""
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE=Path(__file__).resolve().parents[1]


class DropLocalZBounds(unittest.TestCase):
    def test_negative_z_legacy_limits_and_invalid_geometry(self):
        source=(PACKAGE/'src/patrol_control.cpp').read_text(encoding='utf-8')
        start=source.index('    // Calibrated external delivery heights')
        guard=source[start:source.index('    ROS_INFO(',start)]
        program=r'''
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#define ROS_FATAL(...) ((void)0)
struct Config {
 bool external_mission_mode_=true,compensated_alignment_enabled_=true,drop_metric_scale_enabled_=true;
 double drop_ground_z_=-.22,drop_height_threshold=.18,drop_release_setpoint_height_=-.02,drop_release_min_height_=-.07;
 double drop_position_threshold_=.15,external_recovery_height_=1,align_height=1.4;
 double external_standard_recovery_setpoint_height_=1.1,external_cross_recovery_setpoint_height_=1.1;
 double external_planner_max_command_z_=2.;
 void validate(){GUARD}
 bool accepted(){try{validate();return true;}catch(const std::invalid_argument&){return false;}}
};
int main(){
 for(double fc_z:{-.05,0.,.004678211873397231,.12,.30}){
  Config c;c.drop_ground_z_=fc_z-.22;c.drop_release_min_height_=c.drop_ground_z_+.10;
  c.drop_release_setpoint_height_=c.drop_ground_z_+.15;c.drop_height_threshold=c.drop_ground_z_+.43;
  assert(c.accepted());
 }
 for(int lane=0;lane<3;++lane){
  Config c;
  if(lane==0)c.external_mission_mode_=false;
  if(lane==1)c.compensated_alignment_enabled_=false;
  if(lane==2)c.drop_metric_scale_enabled_=false;
  assert(!c.accepted());
  c.drop_release_min_height_=.1;c.drop_release_setpoint_height_=.2;c.drop_height_threshold=.3;assert(c.accepted());
 }
 for(int failure=0;failure<9;++failure){
  Config c;
  if(failure==0)c.drop_ground_z_=NAN;
  if(failure==1)c.drop_release_setpoint_height_=NAN;
  if(failure==2)c.drop_release_min_height_=c.drop_ground_z_;
  if(failure==3)c.drop_release_setpoint_height_=c.drop_ground_z_+.149;
  if(failure==4)c.drop_release_min_height_=c.drop_release_setpoint_height_+.01;
  if(failure==5)c.drop_height_threshold=c.drop_release_setpoint_height_-.01;
  if(failure==6)c.drop_height_threshold=c.drop_ground_z_+1.10001;
  if(failure==7)c.drop_position_threshold_=0;
  if(failure==8)c.external_recovery_height_=c.drop_height_threshold;
  assert(!c.accepted());
 }
}
'''.replace('GUARD',guard)
        with tempfile.TemporaryDirectory(prefix='drop-local-z-') as out:
            cpp=Path(out)/'check.cpp';binary=Path(out)/'check';cpp.write_text(program,encoding='utf-8')
            subprocess.run(['g++','-std=c++14','-O1','-fsanitize=undefined',str(cpp),'-o',str(binary)],check=True,timeout=30)
            subprocess.run([str(binary)],check=True,timeout=10)


if __name__=='__main__':unittest.main()
