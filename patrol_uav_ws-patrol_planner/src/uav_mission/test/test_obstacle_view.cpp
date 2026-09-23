#include <gtest/gtest.h>
#include <uav_mission/obstacle_view.h>
#include <limits>
geometry_msgs::Point point(double x, double y, double z) {
  geometry_msgs::Point p; p.x=x; p.y=y; p.z=z; return p;
}
TEST(ObstacleView, SurfaceRemovesInteriorAndDuplicates) {
  std::vector<geometry_msgs::Point> input;
  for (int x=0;x<3;++x) for(int y=0;y<3;++y) for(int z=0;z<3;++z)
    input.push_back(point(x*.1+.025,y*.1+.025,z*.1+.025));
  input.push_back(input.front());
  auto result=obstacle_view::surface(input,.1,12000);
  EXPECT_EQ(result.points.size(),26u);
  EXPECT_DOUBLE_EQ(result.resolution,.1);
}
TEST(ObstacleView, LimitCoarsensAllOccupiedCells) {
  std::vector<geometry_msgs::Point> input;
  for(int x=-20;x<20;++x) for(int y=-20;y<20;++y)
    input.push_back(point(x*.1+.025,y*.1+.025,.025));
  auto result=obstacle_view::surface(input,.1,64);
  EXPECT_LE(result.points.size(),64u);
  EXPECT_GT(result.resolution,.1);
  // This single-layer map exposes every cell; all inputs must remain covered.
  for(const auto& p:input) {
    bool covered=false;
    for(const auto& q:result.points)
      if(std::abs(p.x-q.x)<=result.resolution/2 &&
         std::abs(p.y-q.y)<=result.resolution/2 &&
         std::abs(p.z-q.z)<=result.resolution/2) covered=true;
    EXPECT_TRUE(covered);
  }
}
TEST(ObstacleView, EmptyAndNonfiniteCloud) {
  auto nan=std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(obstacle_view::surface({},.1,12000).points.empty());
  EXPECT_TRUE(obstacle_view::surface({point(nan,0,0)},.1,12000).points.empty());
  EXPECT_THROW(obstacle_view::surface({},0,12000),std::invalid_argument);
}
TEST(ObstacleView, PathKeepsEndpointsAndOrder) {
  std::vector<geometry_msgs::Point> input;
  for(int i=0;i<7001;++i) input.push_back(point(i,i%3,0));
  auto result=obstacle_view::path(input,1000);
  ASSERT_EQ(result.size(),1000u);
  EXPECT_EQ(result.front().x,0);
  EXPECT_EQ(result.back().x,7000);
  for(size_t i=1;i<result.size();++i) EXPECT_GT(result[i].x,result[i-1].x);
  EXPECT_THROW(obstacle_view::path({point(NAN,0,0)},1000),std::invalid_argument);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
