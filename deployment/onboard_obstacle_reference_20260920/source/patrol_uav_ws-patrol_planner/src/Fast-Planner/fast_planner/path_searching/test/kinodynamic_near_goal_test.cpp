#include <gtest/gtest.h>
#include <path_searching/kinodynamic_astar.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>

// Exercise the real search against an in-memory occupancy map. No
// sensor publisher, flight simulator or hardware service is needed.
class KinodynamicSearchFixture : public ::testing::Test {
 protected:
  SDFMap::Ptr map;
  fast_planner::KinodynamicAstar search;
  void SetUp() override {
    map.reset(new SDFMap);
    auto& p=map->mp_;
    p.map_origin_=Eigen::Vector3d(-2,-2,0);
    p.map_size_=Eigen::Vector3d(4,4,2);
    p.map_min_boundary_=p.map_origin_;
    p.map_max_boundary_=p.map_origin_+p.map_size_;
    p.resolution_=0.05; p.resolution_inv_=20;
    p.map_voxel_num_=Eigen::Vector3i(80,80,40);
    p.virtual_ceil_height_=-1.0;
    map->md_.occupancy_buffer_inflate_.assign(80*80*40,0);
    fast_planner::EDTEnvironment::Ptr env(new fast_planner::EDTEnvironment);
    env->sdf_map_=map;
    search.setEnvironment(env);
    search.max_tau_=0.6; search.init_max_tau_=0.6;
    search.max_vel_=1.0; search.max_acc_=1.0;
    search.w_time_=10.0; search.w_z_=1.0;
    search.horizon_=10.0; search.lambda_heu_=2.0;
    search.allocate_num_=20000; search.check_num_=20;
    search.tie_breaker_=1.0001;
    search.resolution_=0.1; search.time_resolution_=0.1;
    search.search_acc_res=0.5; search.search_time_res=1.0;
    search.init(); search.reset();
  }
  void wall(double half_width) {
    for(int x=38;x<=41;++x)
      for(int y=0;y<80;++y)
        for(int z=0;z<40;++z) {
          Eigen::Vector3i cell(x,y,z); Eigen::Vector3d pos;
          map->indexToPos(cell,pos);
          if(std::abs(pos.y())<half_width)
            map->md_.occupancy_buffer_inflate_[map->toAddress(cell)]=1;
        }
  }
  int run() {
    return search.search(Eigen::Vector3d(-0.15,0,1),Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),Eigen::Vector3d(0.15,0,1),Eigen::Vector3d::Zero(),false);
  }
  void assertClearPath() {
    auto points=search.getKinoTraj(0.005);
    ASSERT_FALSE(points.empty());
    for(const auto& p:points) ASSERT_EQ(0,map->getInflateOccupancy(p));
    ASSERT_LT((points.back()-Eigen::Vector3d(0.15,0,1)).norm(),0.01);
  }
  void setBudget(double seconds) { search.max_search_time_=seconds; }
  void setHorizon(double metres) { search.horizon_=metres; }
  void occupy(const Eigen::Vector3d& p) {
    Eigen::Vector3i id; map->posToIndex(p,id);
    map->md_.occupancy_buffer_inflate_[map->toAddress(id)]=1;
  }
  void frozenCloud(const char* filename, double resolution) {
    auto& p=map->mp_;
    p.map_origin_=Eigen::Vector3d(-6,-11,-0.2);
    p.map_size_=Eigen::Vector3d(12,22,3);
    p.map_min_boundary_=p.map_origin_;
    p.map_max_boundary_=p.map_origin_+p.map_size_;
    p.resolution_=resolution; p.resolution_inv_=1.0/resolution;
    p.map_voxel_num_=(p.map_size_/resolution).array().round().cast<int>();
    p.map_min_idx_.setZero(); p.map_max_idx_=p.map_voxel_num_-Eigen::Vector3i::Ones();
    p.local_update_range_=Eigen::Vector3d(8.5,8.5,4.5);
    p.ground_height_=-0.2; p.virtual_ceil_height_=2.3;
    p.obstacles_inflation_=0.3; p.obstacles_inflation_up_=0.1; p.obstacles_inflation_down_=0.3;
    p.horizontal_avoidance_=true;
    p.horizontal_min_x_=-2.1; p.horizontal_max_x_=2.1;
    p.horizontal_min_y_=0.2; p.horizontal_max_y_=5.5;
    p.horizontal_obstacle_min_z_=0.4; p.horizontal_floor_z_=0.1;
    auto& d=map->md_; const int count=p.map_voxel_num_.prod();
    d.occupancy_buffer_inflate_.assign(count,0); d.occupancy_buffer_neg.assign(count,0);
    d.distance_buffer_.assign(count,10000); d.distance_buffer_neg_.assign(count,10000);
    d.distance_buffer_all_.assign(count,10000);
    d.tmp_buffer1_.assign(count,0); d.tmp_buffer2_.assign(count,0);
    d.has_odom_=true; d.camera_pos_=Eigen::Vector3d(-2.393654,5.178551,0.796951);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::ifstream input(filename); ASSERT_TRUE(input.good());
    pcl::PointXYZ point;
    while(input >> point.x >> point.y >> point.z) cloud.push_back(point);
    ASSERT_GT(cloud.size(),20000u);
    sensor_msgs::PointCloud2Ptr message(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(cloud,*message);
    auto before=std::chrono::steady_clock::now();
    map->cloudCallback(message); map->updateESDF3d();
    std::cout << "frozen cloud+ESDF wall seconds: "
      << std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count() << std::endl;
    search.origin_=p.map_origin_; search.map_size_3d_=p.map_max_boundary_;
    search.max_tau_=0.9; search.init_max_tau_=0.8;
    search.w_z_=450; search.horizon_=7; search.lambda_heu_=5;
    search.search_acc_res=0.25; search.search_time_res=0.3;
    search.reset();
  }
  void frozenLeg(const Eigen::Vector3d& start,const Eigen::Vector3d& goal) {
    search.reset();
    ASSERT_EQ(fast_planner::KinodynamicAstar::REACH_END,
      search.search(start,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),goal,
                    Eigen::Vector3d::Zero(),true));
    auto points=search.getKinoTraj(0.005);
    ASSERT_FALSE(points.empty());
    for(const auto& p:points) {
      ASSERT_EQ(0,map->getInflateOccupancy(p)) << p.transpose();
      ASSERT_GE(map->getDistance(p),0.025) << p.transpose();
    }
    ASSERT_LT((points.back()-goal).norm(),0.01);
  }
};

TEST_F(KinodynamicSearchFixture, ClearNearbyGoal) {
  ASSERT_EQ(fast_planner::KinodynamicAstar::REACH_END,run());
  assertClearPath();
}
TEST_F(KinodynamicSearchFixture, BlockedConnectorStillFindsHorizontalDetour) {
  wall(0.30);
  ASSERT_EQ(fast_planner::KinodynamicAstar::REACH_END,run());
  assertClearPath();
}
TEST_F(KinodynamicSearchFixture, ClosedWallCannotEscapeMapBoundary) {
  wall(3.0);
  ASSERT_EQ(fast_planner::KinodynamicAstar::NO_PATH,run());
}
TEST_F(KinodynamicSearchFixture, OccupiedGoalCannotProduceHorizonSuccess) {
  occupy(Eigen::Vector3d(0.15,0,1)); setHorizon(0.2);
  auto before=std::chrono::steady_clock::now();
  ASSERT_EQ(fast_planner::KinodynamicAstar::NO_PATH,run());
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count(),0.02);
}
TEST_F(KinodynamicSearchFixture, ShortGoalCannotEscapeAsPartialTrajectory) {
  wall(3.0); setHorizon(0.5); setBudget(0.02);
  auto before=std::chrono::steady_clock::now();
  ASSERT_EQ(fast_planner::KinodynamicAstar::NO_PATH,run());
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count(),0.10);
}
TEST_F(KinodynamicSearchFixture, FrozenR54FineMapApproachAndDoor) {
  const char* filename=std::getenv("LIFTRACE_FROZEN_CLOUD");
  if (!filename) { std::cout << "External R54 cloud not supplied; fixture not exercised." << std::endl; return; }
  frozenCloud(filename,0.05);
  frozenLeg(Eigen::Vector3d(-2.393654,5.178551,0.796951),Eigen::Vector3d(-2.386703,5.672270,0.75));
  frozenLeg(Eigen::Vector3d(-2.386703,5.672270,0.75),Eigen::Vector3d(-2.386703,6.672270,0.75));
}
TEST_F(KinodynamicSearchFixture, FrozenR54CoarseMapReproducesBlockedGoal) {
  const char* filename=std::getenv("LIFTRACE_FROZEN_CLOUD");
  if (!filename) { std::cout << "External R54 cloud not supplied; fixture not exercised." << std::endl; return; }
  frozenCloud(filename,0.10);
  ASSERT_EQ(1,map->getInflateOccupancy(Eigen::Vector3d(-2.386703,5.672270,0.75)));
}
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  ros::init(argc,argv,"kinodynamic_near_goal_test",ros::init_options::NoRosout|ros::init_options::NoSigintHandler);
  // SDFMap owns a NodeHandle, even when no subscriptions are initialized.
  // Standalone fixtures must not wait forever for optional logger services.
  if (std::getenv("LIFTRACE_OFFLINE_UNIT")) ros::master::setRetryTimeout(ros::WallDuration(0.01));
  ros::start();
  const int result=RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
