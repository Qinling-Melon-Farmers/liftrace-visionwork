#include <gtest/gtest.h>
#include <plan_env/vertical_obstacle_support.h>

using fast_planner::VerticalObstacleSupport;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(VerticalObstacleSupport, IsolatedAndFlatReturnsDoNotMakeColumns) {
  VerticalObstacleSupport s(30, 30, 3, 3, 0.20);
  s.observe(10, 10, .516);
  s.observe(11, 10, .518);
  s.observe(10, 11, .515);
  EXPECT_FALSE(s.supported(10, 10));
}

TEST(VerticalObstacleSupport, NearbyVerticalStructureSupportsObstacle) {
  VerticalObstacleSupport s(30, 30, 3, 3, 0.20);
  s.observe(10, 10, .8);
  s.observe(11, 10, 1.0);
  s.observe(10, 11, 1.2);
  EXPECT_TRUE(s.supported(10, 10));
  EXPECT_FALSE(s.supported(20, 20));
}

TEST(VerticalObstacleSupport, RemoteOrTooSparsePointsDoNotSupport) {
  VerticalObstacleSupport s(30, 30, 3, 3, 0.20);
  s.observe(10, 10, .5);
  s.observe(10, 11, 1.5);
  s.observe(20, 20, 1.0);
  EXPECT_FALSE(s.supported(10, 10));
  EXPECT_FALSE(s.supported(-1, 10));
}

TEST(VerticalObstacleSupport, LegacyParametersRetainOriginalClassification) {
  VerticalObstacleSupport s(30, 30, 0, 1, 0.0);
  s.observe(10, 10, .5);
  EXPECT_TRUE(s.supported(10, 10));
  EXPECT_FALSE(s.supported(11, 10));
}

TEST(VerticalObstacleSupport, CoarseGridDoesNotRoundRadiusToTwentyCmSquare) {
  VerticalObstacleSupport s(30, 30, .15 / .10, 3, .20);
  s.observe(10, 10, .5);
  s.observe(12, 10, .8);  // 20 cm, previously counted after ceil()
  s.observe(12, 12, 1.0); // 28 cm corner, also previously counted
  EXPECT_FALSE(s.supported(10, 10));
  VerticalObstacleSupport disk(30, 30, .15 / .10, 3, .20);
  disk.observe(10, 10, .5);
  disk.observe(11, 10, .7);
  disk.observe(11, 11, .9); // 14.1 cm, inside the configured disk
  EXPECT_TRUE(disk.supported(10, 10));
}

TEST(VerticalObstacleSupport, FineGridExcludesSquareCornersToo) {
  VerticalObstacleSupport s(30, 30, .15 / .05, 3, .20);
  s.observe(10, 10, .5);
  s.observe(13, 13, .8);
  s.observe(13, 12, 1.0);
  EXPECT_FALSE(s.supported(10, 10));
}

TEST(UpwardObstacleColumns, DoesNotFillUnobservedSpaceBelowCanopy) {
  fast_planner::UpwardObstacleColumns c(20, 20, 60);
  c.mark(10, 10, 24); // canopy underside after downward physical inflation
  EXPECT_FALSE(c.occupied(10, 10, 12));
  EXPECT_FALSE(c.occupied(10, 10, 23));
  EXPECT_TRUE(c.occupied(10, 10, 24));
  EXPECT_TRUE(c.occupied(10, 10, 59)); // never fly over it
  EXPECT_FALSE(c.occupied(9, 10, 59)); // no second XY dilation
}

TEST(UpwardObstacleColumns, FullFootprintModeRetainsLowerShoulder) {
  fast_planner::UpwardObstacleColumns c(30, 30, 70);
  // Synthetic cone surface: broad lower shoulder, narrow upper cone.
  for (int z=10; z<=40; ++z) {
    int radius = 8 - (z-10)/5;
    for (int x=15-radius; x<=15+radius; ++x) c.mark(x,15,z);
  }
  EXPECT_TRUE(c.occupied(22,15,55)); // outside upper cone but above lower tree
  EXPECT_FALSE(c.occupied(24,15,55));
  EXPECT_FALSE(c.occupied(22,15,5));
  c.mark(15,15,3); // observed trunk remains occupied down to its own base
  EXPECT_TRUE(c.occupied(15,15,4));
}

TEST(UpwardObstacleColumns, RebuiltCloudCanRemoveOldSyntheticColumns) {
  fast_planner::UpwardObstacleColumns old_map(20,20,60);
  old_map.mark(10,10,10);
  EXPECT_TRUE(old_map.occupied(10,10,50));
  fast_planner::UpwardObstacleColumns next_map(20,20,60);
  EXPECT_FALSE(next_map.occupied(10,10,50));
}

TEST(MiddleHeightColumns, ConeMiddleFreesLowerShoulderOnlyAbovePhysicalGeometry) {
  fast_planner::MiddleHeightColumnSelector m(60,60,1.5,.4,.6);
  struct Point {int x,y,z;}; std::vector<Point> cloud;
  // Filled XY projection of a cone, sampled at 10 cm voxel centers.
  for (int z=5; z<=25; ++z) {
    const int r=(25-z)/2;
    for (int x=-r; x<=r; ++x)
      for (int y=-r; y<=r; ++y)
        if (x*x+y*y<=r*r && (r==0 || x*x+y*y>=(r-1)*(r-1)))
          cloud.push_back({30+x,30+y,z});
  }
  for (const auto& p:cloud) m.observe(p.x,p.y,p.z*.1);
  m.build();
  for (const auto& p:cloud) m.observeBand(p.x,p.y,p.z*.1);
  fast_planner::UpwardObstacleColumns c(60,60,40);
  bool lower_physical=false;
  for (const auto& p:cloud) {
    if (p.x==39 && p.y==30) lower_physical=true;
    if (m.source(p.x,p.y,p.z*.1)) c.mark(p.x,p.y,p.z);
  }
  EXPECT_TRUE(lower_physical);
  EXPECT_FALSE(c.occupied(39,30,35)); // broad base no longer makes a high column
  EXPECT_TRUE(c.occupied(35,30,35));  // measured mid-canopy still extends upward
  EXPECT_FALSE(c.occupied(35,30,7));  // does not invent a filled underside
}

TEST(MiddleHeightColumns, SeparateHeightsAndSparseBandFallback) {
  fast_planner::MiddleHeightColumnSelector m(30,30,1.5,.4,.6);
  for (double z:{1.,2.,3.}) m.observe(5,5,z);
  for (double z:{2.,4.,6.}) m.observe(20,20,z);
  // Only endpoints in third component: no invented middle footprint.
  for (double z:{1.,3.}) m.observe(5,20,z);
  m.build();
  for (double z:{1.,2.,3.}) m.observeBand(5,5,z);
  for (double z:{2.,4.,6.}) m.observeBand(20,20,z);
  for (double z:{1.,3.}) m.observeBand(5,20,z);
  EXPECT_TRUE(m.source(5,5,2.)); EXPECT_FALSE(m.source(5,5,1.));
  EXPECT_TRUE(m.source(20,20,4.)); EXPECT_FALSE(m.source(20,20,2.));
  EXPECT_TRUE(m.source(5,20,1.)); EXPECT_TRUE(m.source(5,20,3.));
  EXPECT_FALSE(m.source(10,10,2.));
}

TEST(MiddleHeightColumns, BandCanBeTunedWithoutChangingRawObservations) {
  fast_planner::MiddleHeightColumnSelector m(10,10,1.5,.2,.4);
  for (int z=0;z<=10;++z) m.observe(5,5,z);
  m.build();
  for (int z=0;z<=10;++z) m.observeBand(5,5,z);
  EXPECT_TRUE(m.source(5,5,3.));
  EXPECT_FALSE(m.source(5,5,5.));
}

TEST(MiddleHeightColumns, SurfaceRingFillsCenterWithoutJoiningSeparateTrees) {
  fast_planner::MiddleHeightColumnSelector m(60,60,1.5,.4,.6);
  for (int x=10;x<=20;++x) for (int y=10;y<=20;++y)
    if (x==10 || x==20 || y==10 || y==20)
      for (double z:{1.,2.,3.}) m.observe(x,y,z);
  for (double z:{1.,2.,3.}) m.observe(45,45,z);
  m.build();
  for (int x=10;x<=20;++x) for (int y=10;y<=20;++y)
    if (x==10 || x==20 || y==10 || y==20) m.observeBand(x,y,2.);
  m.observeBand(45,45,2.);
  fast_planner::UpwardObstacleColumns c(60,60,50);
  m.forEachFootprint([&](int x,int y,double z){c.mark(x,y,int(z*10));});
  EXPECT_TRUE(c.occupied(15,15,40)); // closed interior of middle ring
  EXPECT_FALSE(c.occupied(30,30,40)); // no global hull bridge to other tree
  EXPECT_TRUE(c.occupied(45,45,40));
}
