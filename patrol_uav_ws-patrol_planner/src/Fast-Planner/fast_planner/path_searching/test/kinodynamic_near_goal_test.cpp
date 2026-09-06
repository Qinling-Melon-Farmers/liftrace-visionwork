#include <gtest/gtest.h>
#include <path_searching/kinodynamic_astar.h>

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
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  ros::init(argc,argv,"kinodynamic_near_goal_test",ros::init_options::NoRosout|ros::init_options::NoSigintHandler);
  return RUN_ALL_TESTS();
}
