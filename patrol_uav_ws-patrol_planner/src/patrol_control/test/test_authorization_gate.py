"""Production permission callbacks and gate with generated messages, no ROS master."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
# catkin/nose 按包名导入，unittest 按目录导入；两种入口都显式定位同目录测试工具。
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_async_servo import method
P=Path(__file__).resolve().parents[1]
PROGRAM=r'''
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <patrol_control/ReleaseAuthorization.h>
#include <uav_vision/AlignmentTargetContext.h>
#include <cassert>
#include <boost/make_shared.hpp>
#undef ROS_INFO_THROTTLE
#define ROS_INFO_THROTTLE(...) ((void)0)
namespace patrol_control {
class LLController {
public:
 bool external_mission_mode_=true,mission_release_permission_active_=false,have_servo_alignment_context_=true;
 ros::Time latest_mission_release_permission_time_;
 double mission_release_permission_timeout_=.25;
 ReleaseAuthorization release_authorization_;
 uav_vision::AlignmentTargetContext servo_alignment_context_;
 bool hasFreshMissionReleasePermission()const;
 void missionReleasePermissionCallback(const std_msgs::Bool::ConstPtr&);
 void releaseAuthorizationCallback(const ReleaseAuthorization::ConstPtr&);
};
PRODUCTION
}
using namespace patrol_control;
int main(int argc,char**argv){
 assert(argc==2);std::string test=argv[1];ros::Time::init();ros::Time::setNow(ros::Time(100));
 LLController c;
 auto& x=c.servo_alignment_context_;x.active=true;x.mission_id="m";x.decision_seq=9;x.attempt=1;
 x.payload_slot=1;x.semantic_target_id=7;x.semantic_target_first_seen=ros::Time(90);
 x.semantic_target_class="panzer";x.align_mode="drop_circle";
 auto p=boost::make_shared<ReleaseAuthorization>();p->header.stamp=ros::Time(100);
 p->valid_until=ros::Time(100.2);p->permission_epoch="epoch";p->permission_revision=18;p->permitted=true;
 p->mission_id="m";p->decision_seq=9;p->attempt=1;p->payload_slot=1;p->target_id=7;
 p->target_first_seen=ros::Time(90);p->target_class="panzer";p->align_mode="drop_circle";
 c.releaseAuthorizationCallback(p);
 assert(c.hasFreshMissionReleasePermission());
 if(test=="bool_is_legacy_only"){
   auto b=boost::make_shared<std_msgs::Bool>();b->data=false;c.missionReleasePermissionCallback(b);
   assert(c.hasFreshMissionReleasePermission());
   p->permission_revision=19;p->permitted=false;c.releaseAuthorizationCallback(p);
   b->data=true;c.missionReleasePermissionCallback(b);assert(!c.hasFreshMissionReleasePermission());
   c.external_mission_mode_=false;c.missionReleasePermissionCallback(b);assert(c.hasFreshMissionReleasePermission());
 }else if(test=="temporal"){
   for(int n=0;n<4;++n){auto q=boost::make_shared<ReleaseAuthorization>(*p);q->permission_revision=19+n;
     if(n==0)q->header.stamp=ros::Time(99.7);if(n==1)q->header.stamp=ros::Time(100.001);
     if(n==2)q->valid_until=ros::Time(100);if(n==3)q->header.stamp=ros::Time(0);
     c.releaseAuthorizationCallback(q);assert(!c.hasFreshMissionReleasePermission());}
 }else if(test=="identity"){
   for(int n=0;n<8;++n){auto q=boost::make_shared<ReleaseAuthorization>(*p);q->permission_revision=19+n;
     if(n==0)q->mission_id="other";if(n==1)q->decision_seq++;if(n==2)q->attempt++;
     if(n==3)q->payload_slot++;if(n==4)q->target_id++;if(n==5)q->target_first_seen=ros::Time(91);
     if(n==6)q->target_class="bridge";if(n==7)q->align_mode="drop_cross";
     c.releaseAuthorizationCallback(q);assert(!c.hasFreshMissionReleasePermission());}
 }else if(test=="revocation_replay"){
   auto denied=boost::make_shared<ReleaseAuthorization>(*p);denied->permission_revision=19;denied->permitted=false;
   c.releaseAuthorizationCallback(denied);c.releaseAuthorizationCallback(p);assert(!c.hasFreshMissionReleasePermission());
   p->permission_revision=19;c.releaseAuthorizationCallback(p);assert(!c.hasFreshMissionReleasePermission());
   p->permission_revision=20;c.releaseAuthorizationCallback(p);assert(c.hasFreshMissionReleasePermission());
 }else if(test=="missing_context"){
   c.have_servo_alignment_context_=false;assert(!c.hasFreshMissionReleasePermission());
   c.have_servo_alignment_context_=true;x.active=false;assert(!c.hasFreshMissionReleasePermission());
 }else if(test=="new_epoch"){
   p->permission_epoch="restart";p->permission_revision=1;p->permitted=false;
   c.releaseAuthorizationCallback(p);assert(!c.hasFreshMissionReleasePermission());
   assert(c.release_authorization_.permission_epoch=="restart");
 }else assert(false);
}
'''
class AuthorizationGateTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='authorization-gate-');d=Path(cls.tmp.name)
  source=(P/'src/patrol_control.cpp').read_text()
  signatures=['bool LLController::hasFreshMissionReleasePermission() const',
              'void LLController::missionReleasePermissionCallback(',
              'void LLController::releaseAuthorizationCallback(']
  cpp=d/'gate.cpp';cpp.write_text(PROGRAM.replace('PRODUCTION','\n'.join(method(source,x) for x in signatures)))
  cls.binary=d/'gate'
  # Use this checkout's generated messages; no ROS process is launched.
  ws=P.parents[1]
  flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','roscpp'],text=True))
  roots=[ws/'devel/include',ws.parent/'vision_ws/devel/include']
  if not (roots[0]/'patrol_control/ReleaseAuthorization.h').exists():
   raise unittest.SkipTest('build patrol_control generated messages first')
  subprocess.run(['g++','-std=c++14','-O1',str(cpp),'-o',str(cls.binary)]+['-I'+str(x) for x in roots]+flags,check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def test_bool_is_legacy_only(self):subprocess.run([str(self.binary),'bool_is_legacy_only'],check=True)
 def test_temporal(self):subprocess.run([str(self.binary),'temporal'],check=True)
 def test_identity(self):subprocess.run([str(self.binary),'identity'],check=True)
 def test_revocation_replay(self):subprocess.run([str(self.binary),'revocation_replay'],check=True)
 def test_missing_context(self):subprocess.run([str(self.binary),'missing_context'],check=True)
 def test_new_epoch(self):subprocess.run([str(self.binary),'new_epoch'],check=True)
if __name__=='__main__':unittest.main()