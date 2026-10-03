#define main traj_server_node_main
#include "../src/traj_server.cpp"
#undef main
#include <cassert>

int main(int argc,char**argv) {
 ros::init(argc,argv,"handoff_unit",ros::init_options::AnonymousName);
 require_goal_identity=true;
 auto curve=boost::make_shared<plan_manage::Bspline>();
 curve->order=3;curve->traj_id=7;curve->goal_stamp=ros::Time(20);curve->goal_frame="camera_init";
 curve->start_time=ros::Time(21);curve->yaw_dt=.1;
 curve->pos_pts.resize(4);curve->yaw_pts.resize(4,0.);
 curve->knots={-.3,-.2,-.1,0.,.1,.2,.3,.4};
 bsplineCallback(curve);assert(pending_goal_trajectory && !receive_traj_);
 geometry_msgs::PoseStamped g;g.header.stamp=ros::Time(20);g.header.frame_id="camera_init";g.pose.orientation.w=1;
 goalCallback(g);assert(receive_traj_ && traj_id_==7 && !trajectory_interrupted_ && !pending_goal_trajectory);
 replanCallback(std_msgs::Empty());bsplineCallback(curve);assert(trajectory_interrupted_);
 auto next=boost::make_shared<plan_manage::Bspline>(*curve);next->goal_stamp=ros::Time(30);next->start_time=ros::Time(31);next->traj_id=8;
 g.header.stamp=ros::Time(30);goalCallback(g);bsplineCallback(next);assert(traj_id_==8 && !trajectory_interrupted_);
 replanCallback(std_msgs::Empty());bsplineCallback(curve);assert(trajectory_interrupted_ && !pending_goal_trajectory);
 auto bad=boost::make_shared<plan_manage::Bspline>(*next);bad->goal_stamp=ros::Time(40);bad->start_time=ros::Time(41);bad->traj_id=9;bad->order=2;
 bsplineCallback(bad);g.header.stamp=ros::Time(40);goalCallback(g);assert(trajectory_interrupted_ && traj_id_==8);
 puts("PASS: curve-first, goal-first, duplicate, stale, invalid shape");
}
