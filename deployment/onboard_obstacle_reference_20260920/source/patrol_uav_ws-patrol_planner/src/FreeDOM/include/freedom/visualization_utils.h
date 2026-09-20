#ifndef FREEDOM_VISUALIZATION_UTILS_H
#define FREEDOM_VISUALIZATION_UTILS_H

#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <std_msgs/Header.h>

namespace freedom {

enum class FramePolicy
{
    PRESERVE_INPUT,
    OVERRIDE
};

enum class StampPolicy
{
    PRESERVE_INPUT,
    NOW_IF_ZERO,
    NOW
};

inline FramePolicy parse_frame_policy(const std::string& value)
{
    if(value == "preserve_input")
        return FramePolicy::PRESERVE_INPUT;
    if(value == "override")
        return FramePolicy::OVERRIDE;
    throw std::invalid_argument("frame_policy must be preserve_input or override");
}

inline StampPolicy parse_stamp_policy(const std::string& value)
{
    if(value == "preserve_input")
        return StampPolicy::PRESERVE_INPUT;
    if(value == "now_if_zero")
        return StampPolicy::NOW_IF_ZERO;
    if(value == "now")
        return StampPolicy::NOW;
    throw std::invalid_argument("stamp_policy must be preserve_input, now_if_zero, or now");
}

inline bool topics_form_self_loop(const std::string& resolved_input_topic,
                                  const std::string& resolved_output_topic)
{
    return resolved_input_topic == resolved_output_topic;
}

inline std_msgs::Header make_output_header(const std_msgs::Header& input,
                                           FramePolicy frame_policy,
                                           const std::string& frame_id_override,
                                           StampPolicy stamp_policy,
                                           const ros::Time& now)
{
    std_msgs::Header output = input;
    if(frame_policy == FramePolicy::OVERRIDE)
        output.frame_id = frame_id_override;

    if(stamp_policy == StampPolicy::NOW ||
       (stamp_policy == StampPolicy::NOW_IF_ZERO && output.stamp.isZero()))
        output.stamp = now;

    return output;
}

inline pcl::PointCloud<pcl::PointXYZ> downsample_pointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,double leaf_size)
{
    if(!std::isfinite(leaf_size) || leaf_size <= 0.0)
        throw std::invalid_argument("voxel leaf size must be a finite positive number");
    if(!input)
        throw std::invalid_argument("input pointcloud must not be null");

    if(input->empty())
        return *input;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input);
    const float leaf = static_cast<float>(leaf_size);
    voxel_filter.setLeafSize(leaf,leaf,leaf);

    pcl::PointCloud<pcl::PointXYZ> output;
    voxel_filter.filter(output);
    return output;
}

inline pcl::PointCloud<pcl::PointXYZ> downsample_pointcloud(
    const pcl::PointCloud<pcl::PointXYZ>& input,double leaf_size)
{
    return downsample_pointcloud(input.makeShared(),leaf_size);
}

class PublishThrottle
{
public:
    explicit PublishThrottle(double max_rate_hz = 0.5)
    {
        configure(max_rate_hz);
    }

    void configure(double max_rate_hz)
    {
        enabled_ = std::isfinite(max_rate_hz) && max_rate_hz > 0.0;
        min_interval_sec_ = enabled_ ? 1.0 / max_rate_hz : 0.0;
        initialized_ = false;
        last_publish_sec_ = 0.0;
    }

    void reset()
    {
        initialized_ = false;
        last_publish_sec_ = 0.0;
    }

    bool should_publish(double now_sec)
    {
        if(!enabled_ || !std::isfinite(now_sec))
            return false;

        if(!initialized_ || now_sec < last_publish_sec_ ||
           now_sec - last_publish_sec_ + 1e-9 >= min_interval_sec_)
        {
            initialized_ = true;
            last_publish_sec_ = now_sec;
            return true;
        }

        return false;
    }

private:
    bool enabled_ = false;
    bool initialized_ = false;
    double min_interval_sec_ = 0.0;
    double last_publish_sec_ = 0.0;
};

class VisualizationGate
{
public:
    explicit VisualizationGate(double max_rate_hz = 0.5)
      : throttle_(max_rate_hz)
    {
    }

    void configure(double max_rate_hz)
    {
        throttle_.configure(max_rate_hz);
    }

    void reset()
    {
        throttle_.reset();
    }

    bool should_process(std::size_t output_subscribers,double now_sec)
    {
        if(output_subscribers == 0)
            return false;
        return throttle_.should_publish(now_sec);
    }

private:
    PublishThrottle throttle_;
};

class LazySubscriptionState
{
public:
    bool update(std::size_t output_subscribers)
    {
        const bool input_required = output_subscribers > 0;
        if(input_required == input_required_)
            return false;
        input_required_ = input_required;
        return true;
    }

    bool input_required() const
    {
        return input_required_;
    }

private:
    bool input_required_ = false;
};

}  // namespace freedom

#endif
