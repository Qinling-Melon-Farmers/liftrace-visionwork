"""Compile and execute the production target-selection branch without ROS nodes."""
from pathlib import Path
import os,subprocess,tempfile,unittest

SOURCE=Path(__file__).resolve().parents[1]/'src/patrol_control.cpp'

def block_end(text, opening):
    depth=0
    for i in range(opening,len(text)):
        if text[i]=='{':depth+=1
        elif text[i]=='}':
            depth-=1
            if depth==0:return i+1
    raise AssertionError('Unclosed production block')

class ExternalAlignmentFallbackTest(unittest.TestCase):
    def test_real_branch_holds_target_across_dropout_and_keeps_legacy_behavior(self):
        source=SOURCE.read_text();method=source.index('bool LLController::WayPointDetectDone()')
        start=source.index('if(have_waypoint_mark){',method)
        first_end=block_end(source,source.index('{',start))
        self.assertTrue(source[first_end:].lstrip().startswith('else'))
        end=block_end(source,source.index('{',first_end))
        branch=source[start:end]
        program=r'''
#include <array>
#include <vector>
#include <cassert>
#define ROS_INFO(...) ((void)0)
#define ROS_DEBUG_THROTTLE(...) ((void)0)
namespace tf { double getYaw(double yaw) { return yaw; } }
struct Position { double x=0,y=0,z=0; };
struct Pose { Position position; double orientation=0; };
struct Mark { Pose pose; };
struct Waypoint { float x=0,y=0,z=0,yaw=0; };
struct Controller {
  bool external_mission_mode_=true,have_waypoint_mark=false;
  int waypoint_next=0,times_detect=0;
  float align_height=1.6;
  std::array<float,4> adjust_target_position{{.43287f,4.55342f,1.6f,.4f}};
  std::vector<Waypoint> waypoint_list;
  Mark waypoint_mark_point;
  void selectTarget() { BRANCH }
};
int main() {
  Controller c;
  c.waypoint_list.push_back({0,0,1.4f,0});
  const auto locked=c.adjust_target_position;
  // R60 seed4 dropout: external route table may be absent or only (0,0).
  for(int i=0;i<200;++i)c.selectTarget();
  assert(c.adjust_target_position==locked && !c.have_waypoint_mark);
  c.waypoint_list.clear();c.selectTarget();
  assert(c.adjust_target_position==locked);
  // A new transaction replaces the latch; no old-target leakage on loss.
  c.adjust_target_position={{-1.2f,3.1f,.95f,-.2f}};
  const auto next=c.adjust_target_position;c.selectTarget();assert(c.adjust_target_position==next);
  // Fresh geometry is still accepted, followed by a hold at its new value.
  c.have_waypoint_mark=true;c.waypoint_mark_point.pose.position={.6,4.4,0};
  c.waypoint_mark_point.pose.orientation=.3;c.selectTarget();
  assert(c.adjust_target_position[0]==.6f && c.adjust_target_position[1]==4.4f);
  assert(c.adjust_target_position[2]==1.6f && c.times_detect==1);
  const auto observed=c.adjust_target_position;c.have_waypoint_mark=false;c.selectTarget();
  assert(c.adjust_target_position==observed);
  // Standalone old route remains unchanged, including zero coordinates.
  c.waypoint_list.push_back({0,0,1.4f,0});
  c.external_mission_mode_=false;c.selectTarget();
  assert((c.adjust_target_position==std::array<float,4>{{0,0,1.4f,0}}));
}
'''.replace('BRANCH',branch)
        parent=os.environ.get('TEST_ARTIFACT_DIR')
        with tempfile.TemporaryDirectory(prefix='alignment-regression-',dir=parent) as folder:
            path=Path(folder);(path/'test.cpp').write_text(program)
            subprocess.run(['g++','-std=c++14','-O0','-fsanitize=undefined','-fno-sanitize-recover=all',str(path/'test.cpp'),'-o',str(path/'test')],check=True,capture_output=True,text=True)
            subprocess.run([str(path/'test')],check=True,capture_output=True,text=True)

if __name__=='__main__':unittest.main()
