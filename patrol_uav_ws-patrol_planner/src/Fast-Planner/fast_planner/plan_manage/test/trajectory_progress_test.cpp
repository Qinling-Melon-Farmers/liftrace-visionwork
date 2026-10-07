#include <gtest/gtest.h>
#include <plan_manage/trajectory_progress.h>

using fast_planner::projectProgress;
using fast_planner::boundedLookahead;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(TrajectoryProgress, DoesNotJumpToLaterLoopBranch) {
  auto line = [](double t) { return Eigen::Vector3d(t <= 2 ? t : 4-t, 0, 0); };
  const Eigen::Vector3d actual(0.2, 0, 0);
  EXPECT_NEAR(projectProgress(line, actual, 0.0, 4.0), 0.2, 0.021);
  EXPECT_LT(boundedLookahead(line, actual, 0.2, 4.0, 0.4), 0.63);
}
TEST(TrajectoryProgress, LostTrackingDoesNotSelectEndpoint) {
  auto line = [](double t) { return Eigen::Vector3d(t, 0, 0); };
  const Eigen::Vector3d actual(0, 2, 0);
  EXPECT_DOUBLE_EQ(boundedLookahead(line, actual, 0, 10, 0.4), 0.0);
}
TEST(TrajectoryProgress, TightTurnInsideSphereStillHasBoundedArc) {
  auto turn = [](double t) {
    return Eigen::Vector3d(0.15 * std::sin(t), 0.15 * (1 - std::cos(t)), 0);
  };
  const double selected = boundedLookahead(turn, Eigen::Vector3d::Zero(), 0, 6.28, 0.4);
  EXPECT_GT(selected, 2.60);
  EXPECT_LT(selected, 2.70);
}
TEST(TrajectoryProgress, ProgressIsMonotonicAndIncludesEndpoint) {
  auto line = [](double t) { return Eigen::Vector3d(t, 0, 0); };
  EXPECT_DOUBLE_EQ(projectProgress(line, Eigen::Vector3d(0.2, 0, 0), 0.5, 2), 0.5);
  EXPECT_DOUBLE_EQ(projectProgress(line, Eigen::Vector3d(1.013, 0, 0), 1, 1.013), 1.013);
}

#include <bspline/non_uniform_bspline.h>
// Frozen seed36 traj20, ROS 203.62: actual field fixture, no Gazebo required.
TEST(TrajectoryProgress, Seed36SlowInitialCurveDoesNotTrapProjectionBehindLookahead) {
  Eigen::MatrixXd points(26,3); points <<
    0.88110339891336331, 8.348432939144903, 0.25227304348122387,
    0.88225066661834717, 8.3393821716308594, 0.24977628886699677,
    0.88339793432333102, 8.3303314041168157, 0.24727953425276966,
    0.88454577927178957, 8.321281696183128, 0.24478277963853368,
    0.88569011464705594, 8.3122255461240453, 0.24228602502434901,
    0.88684860536101839, 8.3031953794146602, 0.23978927040996456,
    0.88795388188773716, 8.2940675334572731, 0.23729251579632249,
    0.89773045726844658, 8.3107221670074551, 0.23479576117991535,
    0.91453829618994809, 8.3493343085345888, 0.23229900657381333,
    0.93957688545715312, 8.4094152453980193, 0.22980225192930126,
    0.96968830953022545, 8.4967447930246234, 0.22730549742795836,
    1.0163047447394327, 8.5886924041275297, 0.22480874239296636,
    1.0963223288605679, 8.6510665963198274, 0.2223119893470977,
    1.2060818299045979, 8.6925488557895463, 0.21981522888698454,
    1.3435809162314607, 8.7120909638362303, 0.21731849606262754,
    1.5204881468706057, 8.7052043077753112, 0.214821660229287,
    1.6812144059118885, 8.6955703660618546, 0.21232520834903462,
    1.8177294470825849, 8.6866881725647573, 0.20982732533397735,
    1.9321890103606085, 8.6775463254575573, 0.20733477667788552,
    2.024000474286594, 8.6686911445259334, 0.20482234496083557,
    2.0933782407013317, 8.6589496990931352, 0.20238402439542058,
    2.1702362823833456, 8.574985668395934, 0.20886701572553978,
    2.2376524272543215, 8.470890159732626, 0.21826566015142543,
    2.2999999999999998, 8.3499999999999996, 0.23000000000000001,
    2.2999999999999998, 8.3499999999999996, 0.23000000000000001,
    2.2999999999999998, 8.3499999999999996, 0.23000000000000001;
  Eigen::VectorXd knots(30); knots << -1.0639394248134726, -0.7092929498756485, -0.35464647493782425, 0, 0.35464647493782425, 0.7092929498756485, 1.0639394248134726, 1.418585899751297, 1.7732323746891214, 2.1278788496269456, 2.48252532456477, 2.8371717995025945, 3.1918182744404189, 3.5464647493782433, 3.9011112243160677, 4.2557576992538912, 4.6104041741917152, 4.9650506491295392, 5.3196971240673632, 5.6743435990051871, 6.0289900739430111, 6.3836365488808351, 6.7382830238186591, 7.092929498756483, 7.447575973694307, 7.802222448632131, 8.1568689235699541, 8.5115153985077789, 8.8661618734456038, 9.2208083483834269;
  fast_planner::NonUniformBspline spline(points,3,0.1);spline.setKnot(knots);
  auto position=[&](double t) -> Eigen::Vector3d { return spline.evaluateDeBoorT(t); };
  const double duration=spline.getTimeSum();
  const double issued=boundedLookahead(position,position(0),0,duration,.15);
  ASSERT_GT(issued,2.0); // less than 15cm along-path, yet beyond old 1s window
  double projection=projectProgress(position,position(issued),0,duration,.15);
  EXPECT_GT(projection,2.0);
  Eigen::Vector3d actual=position(0);projection=0;
  for(int i=0;i<4000;++i) {
    projection=projectProgress(position,actual,projection,duration,.15);
    const double look=boundedLookahead(position,actual,projection,duration,.15);
    actual+=.05*(position(look)-actual);
  }
  EXPECT_LT((actual-position(duration)).norm(),.03);
  EXPECT_GT(projection,duration-.1);
}
