// Standalone candidate executor/plant harness. No ROS subscriptions, MAVROS,
// truth-to-planner bridge, or hardware command publisher. Run only via the
// main agent's sim_run.sh; all health/coverage here is explicitly synthetic.
#include <plan_manage/navigation_recovery.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace fast_planner::recovery;
struct Box { Vec low,high; int source; };

int main(int argc,char** argv) {
  std::string scenario="column";
  for(int i=1;i+1<argc;++i) if(std::string(argv[i])=="--case") scenario=argv[i+1];
  const char* directory=std::getenv("SIM_RUN_DIR");
  if (!directory) { std::cerr<<"Run with the authorized sim_run.sh wrapper\n";return 64; }
  const std::string root(directory);
  std::ofstream timeline(root+"/recovery_mock.csv");
  timeline<<"t,x,y,z,vx,vy,vz,state,reason\n";
  Config c;c.enabled=true;
  Context x;x.now=10;x.action_deadline=25;x.mission_deadline=60;x.soft_max_z=2.98;
  x.goal=7;x.position=Vec(0,0,2.4);x.goal_position=Vec(1,0,2.4);
  x.offboard=x.navigation_owner=true;
  auto& e=x.evidence;e.stamp=e.map_stamp=x.now;e.frame=e.map_frame="synthetic_task";
  e.reference_verified=e.lio_healthy=e.fc_reset_stream_verified=e.coverage_verified=true;
  e.map_revision=1;
  Vec exit(.3,0,2.4);
  std::vector<Box> boxes={{Vec(-.1,-.1,1.5),Vec(.1,.1,3.4),COLUMN},
                         {Vec(-.1,-.1,.7),Vec(.1,.1,1.5),PHYSICAL}};
  bool expect_resume=true;
  std::string expected_rejection;
  if(scenario=="trusted_height") {
    boxes.clear();x.position.z()=3.10;x.velocity.z()=.08;
    e.independent_height_verified=true;exit=Vec(0,0,2.78);
  } else if(scenario=="fc_reset") {
    // Historical comparison only; production never enables this interface.
    c.require_reference_attestation=true;
    boxes.clear();x.position.z()=3.10;exit=Vec(0,0,2.78);
    // FC alone reports an excess; freshness never supplies independent height.
    expect_resume=false;expected_rejection="untrusted_height";
  } else if(scenario=="blocked") {
    boxes.push_back({Vec(.149,-1,2),Vec(.151,1,3),PHYSICAL});
    expect_resume=false;expected_rejection="no_certified_exit";
  } else if(scenario=="takeover") {
    expect_resume=false;expected_rejection="ownership_lost";
  } else if(scenario=="deadline") {
    expect_resume=false;expected_rejection="deadline";
  } else if(scenario!="column") {
    std::cerr<<"Unknown case\n";return 64;
  }
  MapQuery map;
  map.sources=[&](const Vec& low,const Vec& high){
    int mask=0;for(const auto& b:boxes)
      if((high.array()>=b.low.array()).all() && (low.array()<=b.high.array()).all()) mask|=b.source;
    return mask;
  };
  map.physicalClear=[&](const Vec& low,const Vec& high){return !(map.sources(low,high)&PHYSICAL);};
  map.observedFree=[](const Vec&,const Vec&){return true;}; // this synthetic world only
  map.canResume=[](const Vec&,double){return true;}; // no ordinary planner in this mock
  Transaction transaction(c);
  bool begun=transaction.begin(x,map,{exit});
  unsigned commands=0;
  bool physical_clear=true;
  if(begun) {
    for(unsigned i=0;i<1200;++i) {
      x.now=10+i*.01;e.stamp=e.map_stamp=x.now;
      if(i>=40 && scenario=="takeover") x.takeover=true;
      if(i>=40 && scenario=="deadline") x.action_deadline=x.now;
      Vec command,velocity;
      auto state=transaction.step(x,map,command,velocity);
      timeline<<x.now<<','<<x.position.x()<<','<<x.position.y()<<','<<x.position.z()<<','
              <<x.velocity.x()<<','<<x.velocity.y()<<','<<x.velocity.z()<<','<<state<<','
              <<transaction.reason()<<'\n';
      if(state!=Transaction::ACTIVE) break;
      ++commands;
      // First-order, bounded mock plant; not PX4 dynamics or flight evidence.
      Vec desired=velocity+2.0*(command-x.position);
      if(desired.norm()>c.max_speed) desired*=c.max_speed/desired.norm();
      Vec delta=desired-x.velocity;
      if(delta.norm()>c.max_acceleration*.01) delta*=c.max_acceleration*.01/delta.norm();
      const Vec previous=x.position;x.velocity+=delta;x.position+=x.velocity*.01;
      const Vec low=previous.cwiseMin(x.position),high=previous.cwiseMax(x.position);
      if(!map.physicalClear(low,high) || x.position.z()>c.hard_max_z || x.position.z()<c.hard_min_z) {
        physical_clear=false;break;
      }
    }
  }
  const bool resumed=transaction.state()==Transaction::RESUME;
  const bool preserved=(!begun || (transaction.original().goal==7 &&
      transaction.original().action_deadline==25 && transaction.original().mission_deadline==60 &&
      (transaction.original().goal_position-Vec(1,0,2.4)).norm()<1e-9));
  bool passed=physical_clear && preserved && (expect_resume ? resumed :
      (!resumed && transaction.reason()==expected_rejection));
  if(scenario=="fc_reset" || scenario=="blocked") passed=passed && commands==0;
  std::ofstream status(root+"/gate_status.json");
  status<<"{\"status\":\""<<(passed?"PASS":"FAIL")<<"\",\"scope\":\"candidate_core_mock_only\","
        <<"\"case\":\""<<scenario<<"\",\"reason\":\""<<transaction.reason()<<"\","
        <<"\"synthetic_health_and_coverage\":true,\"flight_chain_connected\":false,"
        <<"\"resume_original_goal_requested\":"<<(resumed?"true":"false")<<','
        <<"\"commands\":"<<commands<<",\"release_commands\":0}\n";
  std::cout<<(passed?"PASS":"FAIL")<<" core mock "<<scenario<<": "<<transaction.reason()<<"\n";
  return passed?0:1;
}
