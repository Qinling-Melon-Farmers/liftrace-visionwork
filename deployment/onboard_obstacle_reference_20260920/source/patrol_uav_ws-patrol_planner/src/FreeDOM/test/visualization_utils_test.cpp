#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "freedom/visualization_utils.h"

namespace freedom {

TEST(PublishThrottleTest, EnforcesConfiguredMaximumRate)
{
    PublishThrottle throttle(0.5);

    EXPECT_TRUE(throttle.should_publish(10.0));
    EXPECT_FALSE(throttle.should_publish(11.999));
    EXPECT_TRUE(throttle.should_publish(12.0));
    EXPECT_FALSE(throttle.should_publish(13.0));
    EXPECT_TRUE(throttle.should_publish(14.0));
}

TEST(PublishThrottleTest, RecoversFromWallClockRegression)
{
    PublishThrottle throttle(1.0);

    EXPECT_TRUE(throttle.should_publish(20.0));
    EXPECT_TRUE(throttle.should_publish(5.0));
    EXPECT_FALSE(throttle.should_publish(5.5));
    EXPECT_TRUE(throttle.should_publish(6.0));
}

TEST(PublishThrottleTest, RejectsDisabledAndInvalidTime)
{
    PublishThrottle disabled(0.0);
    EXPECT_FALSE(disabled.should_publish(1.0));

    PublishThrottle throttle(1.0);
    EXPECT_FALSE(throttle.should_publish(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_TRUE(throttle.should_publish(1.0));
}

TEST(VisualizationGateTest, NoSubscriberDoesNotConsumeThrottleWindow)
{
    VisualizationGate gate(0.5);

    EXPECT_FALSE(gate.should_process(0,10.0));
    EXPECT_TRUE(gate.should_process(1,10.0));
    EXPECT_FALSE(gate.should_process(0,12.0));
    EXPECT_TRUE(gate.should_process(1,12.0));

    gate.reset();
    EXPECT_TRUE(gate.should_process(1,12.1));
}

TEST(LazySubscriptionStateTest, TracksFirstConnectAndLastDisconnect)
{
    LazySubscriptionState state;

    EXPECT_FALSE(state.input_required());
    EXPECT_FALSE(state.update(0));
    EXPECT_TRUE(state.update(1));
    EXPECT_TRUE(state.input_required());
    EXPECT_FALSE(state.update(2));
    EXPECT_FALSE(state.update(1));
    EXPECT_TRUE(state.update(0));
    EXPECT_FALSE(state.input_required());
}

TEST(DownsamplePointcloudTest, KeepsCoordinatesInTheInputFrame)
{
    pcl::PointCloud<pcl::PointXYZ> input;
    input.emplace_back(10.01F,20.01F,30.01F);
    input.emplace_back(10.05F,20.05F,30.05F);
    input.emplace_back(-1.0F,2.0F,3.0F);

    pcl::PointCloud<pcl::PointXYZ> output = downsample_pointcloud(input,0.20);
    ASSERT_EQ(2U,output.size());

    const pcl::PointXYZ* positive = nullptr;
    const pcl::PointXYZ* negative = nullptr;
    for(const pcl::PointXYZ& point : output)
    {
        if(point.x > 0.0F)
            positive = &point;
        else
            negative = &point;
    }

    ASSERT_NE(nullptr,positive);
    ASSERT_NE(nullptr,negative);
    EXPECT_NEAR(10.03,positive->x,1e-5);
    EXPECT_NEAR(20.03,positive->y,1e-5);
    EXPECT_NEAR(30.03,positive->z,1e-5);
    EXPECT_NEAR(-1.0,negative->x,1e-5);
    EXPECT_NEAR(2.0,negative->y,1e-5);
    EXPECT_NEAR(3.0,negative->z,1e-5);
}

TEST(OutputHeaderTest, PreservesOrOverridesMetadataAsConfigured)
{
    std_msgs::Header input;
    input.seq = 17;
    input.stamp = ros::Time(123,456);
    input.frame_id = "camera_init";
    const ros::Time now(999,1);

    std_msgs::Header preserved = make_output_header(
        input,FramePolicy::PRESERVE_INPUT,"",StampPolicy::NOW_IF_ZERO,now);
    EXPECT_EQ(input.seq,preserved.seq);
    EXPECT_EQ(input.stamp,preserved.stamp);
    EXPECT_EQ(input.frame_id,preserved.frame_id);

    input.stamp = ros::Time(0);
    std_msgs::Header overridden = make_output_header(
        input,FramePolicy::OVERRIDE,"map",StampPolicy::NOW_IF_ZERO,now);
    EXPECT_EQ(input.seq,overridden.seq);
    EXPECT_EQ(now,overridden.stamp);
    EXPECT_EQ("map",overridden.frame_id);
}

TEST(ParameterPolicyTest, RejectsUnknownPoliciesAndInvalidLeafSize)
{
    EXPECT_EQ(FramePolicy::PRESERVE_INPUT,parse_frame_policy("preserve_input"));
    EXPECT_EQ(FramePolicy::OVERRIDE,parse_frame_policy("override"));
    EXPECT_THROW(parse_frame_policy("guess"),std::invalid_argument);
    EXPECT_THROW(parse_stamp_policy("guess"),std::invalid_argument);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    EXPECT_THROW(downsample_pointcloud(cloud,0.0),std::invalid_argument);
}

TEST(TopicLoopGuardTest, ComparesResolvedNamesInsteadOfRawParameterText)
{
    const std::string raw_input = "static_pointcloud";
    const std::string raw_output = "/freedom/static_pointcloud";
    EXPECT_NE(raw_input,raw_output);

    const std::string resolved_input = "/freedom/static_pointcloud";
    const std::string resolved_output = "/freedom/static_pointcloud";
    EXPECT_TRUE(topics_form_self_loop(resolved_input,resolved_output));
    EXPECT_FALSE(topics_form_self_loop(resolved_input,
                                       "/freedom/static_pointcloud_viz"));
}

}  // namespace freedom

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
