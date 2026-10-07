import subprocess,tempfile,unittest
from pathlib import Path
class HeightAckTest(unittest.TestCase):
 def test_actual_fsm_ack_is_request_scoped_not_periodic(self):
  root=Path(__file__).resolve().parents[2]
  source=(root/'Fast-Planner/fast_planner/plan_manage/src/kino_replan_fsm.cpp').read_text()
  start=source.index('  static ros::Time height_request_polled;',source.index('void KinoReplanFSM::execFSMCallback'))
  end=source.index('  if (exec_state_==EXEC_TRAJ',start)
  code=r'''
#include <XmlRpcValue.h>
#include <string>
#include <cmath>
#include <limits>
#include <cassert>
XmlRpc::XmlRpcValue request_data,ack_data;
int writes=0,reads=0;
namespace ros {
struct Duration {double v;double toSec()const{return v;}};
struct Time {double v=0;Time(){} Time(double x):v(x){} bool isZero()const{return v==0;} double toSec()const{return v;}};
Duration operator-(Time a,Time b){return Duration{a.v-b.v};}
bool operator<(Time a,Time b){return a.v<b.v;}
namespace param {
bool getCached(const std::string&,XmlRpc::XmlRpcValue& out){++reads;out=request_data;return true;}
void set(const std::string&,const XmlRpc::XmlRpcValue& out){++writes;ack_data=out;}
}}
std::string heightConstraintNamespace(){return "/test";}
struct Height {bool enabled=true,valid=true;double max_z=.78;std::string frame="map";} height;
void tick(double t) {
ros::Time now(t);
BLOCK
}
void request(const std::string& id,double cap,const std::string& frame="map"){
request_data["id"]=id;request_data["max_z"]=cap;request_data["frame"]=frame;
}
int main(){
 request("one",.78);
 for(int i=0;i<100;++i)tick(10.+i*.01);
 assert(writes==1 && reads<=10);
 request("two",.78);tick(11.2);assert(writes==2);
 request("three",.7);tick(11.4);assert(writes==2);
 height.max_z=.7;tick(11.6);assert(writes==3);
 request("four",.7,"wrong");tick(11.8);assert(writes==3);
 assert(static_cast<std::string>(ack_data["id"])=="three");
}
'''.replace('BLOCK',source[start:end])
  with tempfile.TemporaryDirectory() as tmp:
   cpp=Path(tmp)/'ack.cpp';cpp.write_text(code);exe=Path(tmp)/'ack'
   subprocess.run(['g++','-std=c++14','-I/opt/ros/noetic/include/xmlrpcpp','-I/opt/ros/noetic/include',str(cpp),'-L/opt/ros/noetic/lib','-lxmlrpcpp','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
