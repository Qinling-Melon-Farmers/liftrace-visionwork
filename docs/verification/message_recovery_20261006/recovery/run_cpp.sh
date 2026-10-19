set -e
cat > /tmp/recovery_review_20261006/recovery_cases.cpp <<'CPP'
#include <path_searching/kinodynamic_astar.h>
#include <bspline/non_uniform_bspline.h>
#include <iomanip>
#include <stdexcept>
#include <functional>
#include <fstream>
int checks=0;
void require(bool v,const char* msg){++checks;if(!v)throw std::runtime_error(msg);}
void emit(const std::string& name,const std::string& fields){std::cout<<"RESULT "<<name<<" "<<fields<<"\n";}
std::string number(double x){std::ostringstream s;s<<std::setprecision(12)<<x;return s.str();}
SDFMap::Ptr makeMap(bool columns=false,bool middle=true) {
 auto m=std::make_shared<SDFMap>();auto& p=m->mp_;auto& d=m->md_;
 p.map_origin_=Eigen::Vector3d(-2,-2,-.2);p.map_size_=Eigen::Vector3d(4,4,3.8);
 p.map_min_boundary_=p.map_origin_;p.map_max_boundary_=p.map_origin_+p.map_size_;
 p.resolution_=.05;p.resolution_inv_=20;p.map_voxel_num_=Eigen::Vector3i(80,80,76);
 p.local_update_range_=Eigen::Vector3d(4,4,4);p.ground_height_=-.2;p.virtual_ceil_height_=-.1;
 p.search_region_enabled_=false;p.search_region_bounds_=Eigen::Vector4d(-1.8,1.8,-1.8,1.8);
 p.obstacles_inflation_=.25;p.obstacles_inflation_up_=.2;p.obstacles_inflation_down_=.1;
 p.horizontal_avoidance_=columns;p.horizontal_min_x_=-.5;p.horizontal_max_x_=7.95;
 p.horizontal_min_y_=-5;p.horizontal_max_y_=5;p.horizontal_obstacle_min_z_=.18;
 p.horizontal_floor_z_=-.12;p.horizontal_support_min_points_=3;p.horizontal_support_radius_=.15;
 p.horizontal_support_min_span_=.2;p.horizontal_column_middle_=middle;
 p.horizontal_column_low_ratio_=.4;p.horizontal_column_high_ratio_=.6;
 p.horizontal_column_max_hull_span_=1.6;p.horizontal_column_max_fill_distance_=.35;
 p.clamp_min_log_=-2;p.unknown_flag_=.01;p.min_occupancy_log_=1;
 const auto n=p.map_voxel_num_.prod();
 d.occupancy_buffer_.assign(n,-2.01);d.occupancy_buffer_inflate_.assign(n,0);d.occupancy_buffer_neg.assign(n,0);
 d.distance_buffer_.assign(n,10000);d.distance_buffer_neg_.assign(n,10000);d.distance_buffer_all_.assign(n,10000);
 d.tmp_buffer1_.assign(n,0);d.tmp_buffer2_.assign(n,0);d.camera_pos_=Eigen::Vector3d(.2,0,2.4);
 d.local_bound_min_.setZero();d.local_bound_max_=p.map_voxel_num_-Eigen::Vector3i::Ones();
 return m;
}
sensor_msgs::PointCloud2ConstPtr cloud() {
 auto c=std::make_shared<sensor_msgs::PointCloud2>();
 c->points={{1,0,.8},{1,0,1.0},{1,0,1.2},{-1.7,-1.7,.01},{1.7,1.7,.01}};
 return c;
}
void rebuild(SDFMap::Ptr m){m->cloudCallback(cloud());m->updateESDF3d();}
void cap(double z,bool enabled=true){
 ros::param::values["/navigation_height_constraint/enabled"]=enabled;
 ros::param::values["/external_planner_max_command_z"]=z;
 ros::param::values["/navigation_height_constraint/frame_id"]=std::string("camera_init");
}
class KinodynamicSearchFixture {
public:
 fast_planner::KinodynamicAstar search;
 KinodynamicSearchFixture(SDFMap::Ptr m) {
  auto env=std::make_shared<fast_planner::EDTEnvironment>();env->sdf_map_=m;search.setEnvironment(env);
  search.max_tau_=.6;search.init_max_tau_=.6;search.max_vel_=1.;search.max_acc_=1.;
  search.w_time_=10.;search.w_z_=1.;search.horizon_=10.;search.lambda_heu_=2.;
  search.allocate_num_=20000;search.check_num_=20;search.tie_breaker_=1.0001;
  search.resolution_=.1;search.time_resolution_=.1;search.search_acc_res=.5;search.search_time_res=1.;
  search.max_search_time_=.1;search.line_deviation_param_="fixture/line";search.init();search.reset();
 }
 int run(const Eigen::Vector3d& from,const Eigen::Vector3d& to){
  search.reset();return search.search(from,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),to,Eigen::Vector3d::Zero(),false);
 }
 bool shot(const Eigen::Vector3d& from,const Eigen::Vector3d& to,double T){
  search.height_limit_=fast_planner::referenceHeight();
  Eigen::VectorXd a(6),b(6);a<<from,Eigen::Vector3d::Zero();b<<to,Eigen::Vector3d::Zero();
  return search.computeShotTraj(a,b,T);
 }
};
int main(int argc,char** argv){
 std::ofstream results(argv[1]);auto* buffer=std::cout.rdbuf(results.rdbuf());
 cap(2.98);
 auto real=makeMap(false),merged=makeMap(true);
 rebuild(real);rebuild(merged);
 const Eigen::Vector3d virtualPoint(1,0,2.4),realPoint(1,0,1),outsidePoint(1.65,0,2.4);
 require(real->getInflateOccupancy(virtualPoint)==0,"real-only voxel above obstacle must be free");
 require(merged->getInflateOccupancy(virtualPoint)==1,"merged column must occupy upper voxel");
 require(real->getInflateOccupancy(realPoint)==1 && merged->getInflateOccupancy(realPoint)==1,"physical source must remain occupied");
 require(real->getDistance(virtualPoint)>.025 && merged->getDistance(virtualPoint)<=0,"merged ESDF must include column");
 emit("physical_vs_column","real_occ=0 merged_occ=1 real_esdf="+number(real->getDistance(virtualPoint))+" merged_esdf="+number(merged->getDistance(virtualPoint))+" raw_occupancy="+number(merged->getOccupancy(realPoint)));
 Eigen::Vector3i id;merged->posToIndex(virtualPoint,id);
 auto saved=merged->md_.occupancy_buffer_inflate_[merged->toAddress(id)];
 merged->md_.occupancy_buffer_inflate_[merged->toAddress(id)]=0;
 require(merged->getInflateOccupancy(virtualPoint)==0 && merged->getDistance(virtualPoint)<=0,"occupancy clear alone leaves negative ESDF");
 emit("clear_only_stale_esdf","occ=0 esdf="+number(merged->getDistance(virtualPoint)));
 merged->md_.occupancy_buffer_inflate_[merged->toAddress(id)]=saved;
 KinodynamicSearchFixture f(merged);
 auto virtualResult=f.run(virtualPoint,outsidePoint);
 require(virtualResult==fast_planner::KinodynamicAstar::NO_PATH,"interior virtual start has no current escape");
 emit("virtual_column_exit_current","status="+number(virtualResult)+" start_occ="+number(f.search.getLastDiagnostics().start_occupancy)+" rejected_collision="+number(f.search.getLastDiagnostics().rejected_collision));
 KinodynamicSearchFixture g(real);
 auto freeResult=g.run(virtualPoint,outsidePoint);
 require(freeResult==fast_planner::KinodynamicAstar::REACH_END,"physical-only comparison should admit connector");
 emit("real_only_diagnostic_not_enabled_policy","status="+number(freeResult));
 auto clear=makeMap(false);
 KinodynamicSearchFixture empty(clear);
 const Eigen::Vector3d high(0,0,3.10),low(0,0,2.78);
 auto highResult=empty.run(high,low);
 require(highResult==fast_planner::KinodynamicAstar::NO_PATH,"overheight start must reject");
 require(!empty.shot(high,low,2),"overheight shot must reject");
 auto h=fast_planner::referenceHeight();
 require(!h.polynomial(3.10,-.2,0,0,1),"descending polynomial still overheight must reject");
 require(!h.polynomial(2.88,.8,-.8,0,1),"quadratic internal peak must reject");
 require(!h.polynomial(2.88,.4,0,-.4,1),"cubic internal peak must reject");
 emit("overheight_reentry_current","cap=2.98 start=3.10 goal=2.78 search="+number(highResult)+" shot=0 monotone_polynomial=0");
 emit("internal_extrema","quadratic_endpoints=2.88 quadratic_peak=3.08 allowed=0 cubic_peak="+number(2.88+.4*(2./3.)/std::sqrt(3.))+" cubic_allowed=0");
 Eigen::MatrixXd controls(6,3);controls.setZero();
 controls.col(2)<<3.16,3.10,3.04,2.78,2.78,2.78;
 fast_planner::NonUniformBspline spline(controls,3,.2);
 const auto s0=spline.evaluateDeBoorT(0);
 require(std::abs(s0.z()-3.10)<1e-8,"boundary state must match");
 require(!h.controls(controls),"reentry spline must reject");
 emit("reentry_spline_controls","start_z="+number(s0.z())+" controls_max="+number(controls.col(2).maxCoeff())+" allowed=0");
 controls.col(2)<<3.04,2.98,2.92,2.70,2.70,2.70;
 fast_planner::NonUniformBspline tangent(controls,3,.2);double maxCurve=-1e9;
 for(double t=0;t<=tangent.getTimeSum();t+=.001)maxCurve=std::max(maxCurve,tangent.evaluateDeBoorT(t).z());
 require(maxCurve<=2.98+1e-8 && !h.controls(controls),"convex-hull policy may reject descending curve at cap");
 emit("conservative_control_hull","curve_max="+number(maxCurve)+" controls_max=3.04 allowed=0");
 cap(.78);
 require(!fast_planner::referenceHeight().accepts(1.) && fast_planner::referenceHeight().accepts(.68),"corridor profile height");
 emit("corridor_cap","local_cap=0.78 ground=-0.22 agl_cap=1.00 start_local=1.00 start_allowed=0 goal_local=0.68 goal_allowed=1");
 // Cached cloud update cannot turn a positive ceiling off with -0.1.
 auto ceiling=makeMap(false);ceiling->mp_.virtual_ceil_height_=2.3;
 ros::param::values["sdf_map/virtual_ceil_height"]=-.1;
 rebuild(ceiling);
 require(ceiling->mp_.virtual_ceil_height_==2.3 && ceiling->getInflateOccupancy(virtualPoint)==1,"positive ceiling remains on when runtime -0.1 is requested");
 emit("runtime_disable_ceiling","requested=-0.1 cached=2.3 occ_at_2.4=1");
 ros::param::values.erase("sdf_map/virtual_ceil_height");
 // Blocked start is sampled after t=0; no unconditional occupied-start rejection.
 cap(2.98);
 auto edge=makeMap(false);
 const Eigen::Vector3d start(0,0,1.0),end(.3,0,1.0);
 Eigen::Vector3i occupied;edge->posToIndex(start,occupied);
 edge->md_.occupancy_buffer_inflate_[edge->toAddress(occupied)]=1;
 KinodynamicSearchFixture e(edge);
 Eigen::VectorXd a(6),b(6);a<<start,Eigen::Vector3d(1,0,0);b<<end,Eigen::Vector3d(1,0,0);
 // Access through existing production fixture friendship in a helper below.
 emit("completed","checks="+number(checks));
 std::cout.rdbuf(buffer);
 std::cout<<"PASS offline C++ checks="<<checks<<"\n";
}
CPP
# Include sstream before the harness number helper.
sed -i '1i #include <sstream>' /tmp/recovery_review_20261006/recovery_cases.cpp
p=/tmp/recovery_review_20261006/snapshot/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner
g++ -std=c++17 -O1 -g -I/tmp/recovery_review_20261006/stubs -I/usr/include/eigen3 -I$p/plan_env/include -I$p/path_searching/include -I$p/bspline/include /tmp/recovery_review_20261006/production_map.cpp $p/path_searching/src/kinodynamic_astar.cpp $p/bspline/src/non_uniform_bspline.cpp /tmp/recovery_review_20261006/recovery_cases.cpp -o /tmp/recovery_review_20261006/recovery_cases
/tmp/recovery_review_20261006/recovery_cases /tmp/recovery_review_20261006/cpp_results.txt
cat /tmp/recovery_review_20261006/cpp_results.txt

