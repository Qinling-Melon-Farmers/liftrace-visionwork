#!/usr/bin/env python3
"""Reuse Oct 6's inert transport fixture, extracting CURRENT map methods.

No ROS master, simulator, FC connection, or copied production algorithm.
The unchanged method extractor is reused; only transport/data stubs are extended.
"""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[3]
out = Path(sys.argv[1]).resolve()
original = root / 'docs/verification/message_recovery_20261006/recovery'
shell = (original / 'prepare_offline.sh').read_text()
code = shell.split("python - <<'PY'\n", 1)[1].rsplit('\nPY', 1)[0]
code = code.replace("out=Path('/tmp/recovery_review_20261006')", 'out=Path(' + repr(str(out)) + ')')
code = code.replace("repo=Path('/home/xhj/liftrace-worktrees/r2026-high-view-search')",
                    'repo=Path(' + repr(str(root)) + ')')
old = "text=subprocess.check_output(['git','show',ref+':'+path],cwd=repo).decode('utf-8')"
assert old in code
code = code.replace(old, "text=(repo/path).read_text() if path.startswith(base) else "
                         "subprocess.check_output(['git','show',ref+':'+path],cwd=repo).decode('utf-8')")
code = code.replace('struct Time {};', 'struct Time { double value=1; bool isZero() const { return value==0; } };')
code = code.replace('struct PointCloud2 {vector<pcl::PointXYZ> points;};',
                    'struct PointCloud2 { struct Header { ros::Time stamp; } header; vector<pcl::PointXYZ> points;};')
code = code.replace('vector<char> occupancy_buffer_inflate_,occupancy_buffer_neg;',
                    'vector<char> occupancy_buffer_inflate_,occupancy_buffer_neg;\nvector<unsigned char> recovery_sources_;')
code = code.replace('ros::NodeHandle node_;', '''ros::NodeHandle node_;
bool recovery_layers_enabled_=false,recovery_layers_valid_=false;
uint64_t recovery_revision_=0;
ros::Time recovery_stamp_;
Eigen::Vector3d recovery_min_,recovery_max_;
double recovery_body_xy_=.25,recovery_body_up_=.2,recovery_body_down_=.1;
int recoverySources(const Eigen::Vector3d&,const Eigen::Vector3d&);
''')
code = code.replace("'void SDFMap::applyFlightCeiling','void SDFMap::cloudCallback'",
                    "'int SDFMap::recoverySources','void SDFMap::applyFlightCeiling','void SDFMap::cloudCallback'")
# Keep metadata honest: source methods are current uncommitted worktree content.
code = code.replace("'source_head':head,", "'source_head':head,'map_source':'CURRENT WORKTREE (uncommitted)',")
exec(compile(code, str(original / 'prepare_offline.sh'), 'exec'), {})

cpp = (original / 'run_cpp.sh').read_text().split("<<'CPP'\n", 1)[1].split('\nCPP', 1)[0]
cpp = '#include <sstream>\n#include <plan_manage/navigation_recovery.h>\n' + cpp.split('class KinodynamicSearchFixture {', 1)[0]
cpp += r'''
int main() {
 using namespace fast_planner::recovery;
 auto enabled=makeMap(true), baseline=makeMap(true);
 enabled->recovery_layers_enabled_=true;
 enabled->md_.recovery_sources_.assign(enabled->md_.occupancy_buffer_inflate_.size(),0);
 rebuild(enabled);rebuild(baseline);
 require(enabled->md_.occupancy_buffer_inflate_==baseline->md_.occupancy_buffer_inflate_,"default merged occupancy must not change");
 require(enabled->md_.distance_buffer_all_==baseline->md_.distance_buffer_all_,"merged ESDF must not change");
 const Vec p(1,0,2.4), real(1,0,1), out(.3,0,2.4);
 require(baseline->recoverySources(p,p)==-1,"disabled layers unavailable");
 require(enabled->recoverySources(p,p)==COLUMN,"pure virtual column source");
 require(enabled->recoverySources(real,real)&PHYSICAL,"real inflation source retained even when overlapping column");
 require(enabled->recoverySources(out,out)==0,"legal neighbor source");
 const Vec edge(1.9,0,2.4);
 require(enabled->recoverySources(edge,edge)==-1,"patch edge is not recovery evidence");
 const Vec below(1,0,.75);
 require(enabled->recoverySources(below,below)&PHYSICAL,"necessary downward body inflation is physical");
 Config c;c.enabled=true;
 Context x;x.now=1;x.action_deadline=20;x.mission_deadline=30;x.soft_max_z=2.98;
 x.position=p;x.velocity=Vec(-.03,0,0);x.goal_position=out;x.offboard=x.navigation_owner=true;
 auto& e=x.evidence;e.stamp=e.map_stamp=1;e.frame=e.map_frame="task";
 e.map_revision=1;e.reference_verified=e.lio_healthy=e.fc_reset_stream_verified=e.coverage_verified=true;
 MapQuery map;
 map.sources=[&](const Vec& a,const Vec& b){return enabled->recoverySources(a,b);};
 map.physicalClear=[&](const Vec& a,const Vec& b){int m=map.sources(a,b);return m>=0 && !(m&PHYSICAL);};
 // Synthetic fully observed fixture ONLY, never the production cloud callback.
 map.observedFree=[](const Vec&,const Vec&){return true;};
 const auto resumeFor=[](SDFMap::Ptr m) {
   return [m](const Vec& p,double slack) {
     return m->recoverySources(p,p)==0 && normalPlannerStartClear(
         m->getInflateOccupancy(p),m->getDistance(p),.20,slack,m->mp_.resolution_);
   };
 };
 map.canResume=resumeFor(enabled);
 Curve curve;std::string why;
 require(plan(c,x,map,{out},curve,why),"current source-classified map supports complete virtual exit");
 require((curve.position(0)-p).norm()<1e-9 && (curve.velocity(0)-x.velocity).norm()<1e-9,"actual start and velocity retained");
 require(enabled->md_.occupancy_buffer_inflate_==baseline->md_.occupancy_buffer_inflate_,"recovery must not mutate map");
 // Rebuilding removes prior source bits as well as normal synthetic occupancy.
 auto empty=std::make_shared<sensor_msgs::PointCloud2>();
 empty->points={{-1.7,-1.7,.01},{1.7,1.7,.01}};
 enabled->cloudCallback(empty);
 require(enabled->recoverySources(p,p)==0,"rebuild clears old virtual provenance");
 enabled->resetBuffer();require(enabled->recoverySources(p,p)==-1,"external reset invalidates provenance");
 auto buffer=makeMap(false);
 buffer->recovery_layers_enabled_=true;
 buffer->md_.recovery_sources_.assign(buffer->md_.occupancy_buffer_inflate_.size(),0);
 buffer->mp_.obstacles_inflation_=.45;
 rebuild(buffer);
 const Vec cushion(.62,0,1.),safe(.10,0,1.);
 require(buffer->recoverySources(cushion,cushion)==BUFFER,"ordinary extra buffer classified independently");
 c.tracking_error=.08;x.position=cushion;x.velocity.setZero();
 map.sources=[&](const Vec& a,const Vec& b){return buffer->recoverySources(a,b);};
 map.canResume=resumeFor(buffer);
 require(plan(c,x,map,{safe},curve,why),"ordinary extra buffer exit on real production map");
 x.position=Vec(.9,0,1.);
 require(!plan(c,x,map,{safe},curve,why),"necessary physical inflation never exempted");
 // Reproduce the first production-chain failure with the actual cloud and
 // ESDF methods: free occupancy at x=.6 did not meet manager clearance=.2.
 auto handoff=makeMap(true);
 handoff->mp_.obstacles_inflation_=.30;
 handoff->recovery_body_xy_=.275;
 handoff->recovery_body_down_=.20;
 handoff->recovery_layers_enabled_=true;
 handoff->md_.recovery_sources_.assign(handoff->md_.occupancy_buffer_inflate_.size(),0);
 rebuild(handoff);
 const Vec rejected(.6,0,2.4);
 require(handoff->getInflateOccupancy(rejected)==0,"failed-run endpoint is occupancy free");
 require(handoff->getDistance(rejected)<.20,"failed-run endpoint violates unchanged normal ESDF clearance");
 map.sources=[&](const Vec& a,const Vec& b){return handoff->recoverySources(a,b);};
 map.canResume=resumeFor(handoff);
 require(!map.canResume(rejected,0.),"actual handoff gate rejects failed-run endpoint");
 x.position=p;x.velocity.setZero();x.goal_position=Vec(-.5,0,2.4);
 const auto original_deadline=x.action_deadline;
 require(!plan(c,x,map,{rejected},curve,why),"occupancy-only exit no longer accepted");
 require(plan(c,x,map,{rejected,Vec(.5,0,2.4),Vec(.4,0,2.4),Vec(.3,0,2.4),Vec(.2,0,2.4)},curve,why),
         "select farther endpoint with ordinary planner clearance");
 const Vec chosen=curve.controls[3];
 require(chosen.x()<rejected.x() && map.canResume(chosen,c.finish_position),"selected endpoint has stopping margin");
 require((chosen-p).norm()<=c.radius && curve.duration+c.settle_seconds<=c.max_seconds,"farther exit remains within recovery budget");
 for(int axis=0;axis<3;++axis) for(double sign:{-1.,1.}) {
   Vec actual=chosen;actual[axis]+=sign*c.finish_position;
   require(map.canResume(actual,0.),"actual endpoint tolerance retains normal occupancy and ESDF clearance");
 }
 require(x.action_deadline==original_deadline,"endpoint selection cannot renew action deadline");
 x.action_deadline=x.now+1.;
 require(!plan(c,x,map,{chosen},curve,why),"farther exit rejected if original budget is too short");
 std::cout << "column handoff: rejected_x=.6 distance=" << handoff->getDistance(rejected)
           << " chosen_x=" << chosen.x() << " distance=" << handoff->getDistance(chosen)
           << " unchanged_clearance=.2\n";
 std::cout << "PASS current production map + recovery checks=" << checks << "\n";
}
'''
(out / 'map_recovery_cases.cpp').write_text(cpp)
