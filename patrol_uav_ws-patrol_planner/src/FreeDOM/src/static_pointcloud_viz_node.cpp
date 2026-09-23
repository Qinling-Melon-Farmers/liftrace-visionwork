#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>

#include <boost/bind/bind.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "freedom/visualization_utils.h"

namespace freedom {

class StaticPointcloudVizNode
{
public:
    StaticPointcloudVizNode()
      : private_nh_("~")
    {
        load_params();
        validate_params();

        frame_policy_ = parse_frame_policy(frame_policy_name_);
        stamp_policy_ = parse_stamp_policy(stamp_policy_name_);
        gate_.configure(max_rate_hz_);

        using boost::placeholders::_1;
        output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            output_topic_,static_cast<std::uint32_t>(output_queue_size_),
            boost::bind(&StaticPointcloudVizNode::connect_callback,this,_1),
            boost::bind(&StaticPointcloudVizNode::disconnect_callback,this,_1));

        ROS_INFO_STREAM("static_pointcloud_viz_node ready: " << resolved_input_topic_
                        << " -> " << resolved_output_topic_ << ", max_rate=" << max_rate_hz_
                        << " Hz, leaf=" << voxel_leaf_size_ << " m");
    }

private:
    void load_params()
    {
        private_nh_.param<std::string>("input_topic",input_topic_,
                                       "/freedom/static_pointcloud");
        private_nh_.param<std::string>("output_topic",output_topic_,
                                       "/freedom/static_pointcloud_viz");
        private_nh_.param<int>("input_queue_size",input_queue_size_,1);
        private_nh_.param<int>("output_queue_size",output_queue_size_,1);
        private_nh_.param<double>("max_rate_hz",max_rate_hz_,0.5);
        private_nh_.param<double>("voxel_leaf_size",voxel_leaf_size_,0.20);
        private_nh_.param<std::string>("frame_policy",frame_policy_name_,
                                       "preserve_input");
        private_nh_.param<std::string>("frame_id_override",frame_id_override_,"");
        private_nh_.param<std::string>("stamp_policy",stamp_policy_name_,
                                       "now_if_zero");
    }

    void validate_params()
    {
        if(input_topic_.empty() || output_topic_.empty())
            throw std::invalid_argument("input_topic and output_topic must not be empty");
        if(input_queue_size_ <= 0 || output_queue_size_ <= 0)
            throw std::invalid_argument("input_queue_size and output_queue_size must be positive");
        if(!std::isfinite(max_rate_hz_) || max_rate_hz_ <= 0.0)
            throw std::invalid_argument("max_rate_hz must be a finite positive number");
        if(!std::isfinite(voxel_leaf_size_) || voxel_leaf_size_ <= 0.0)
            throw std::invalid_argument("voxel_leaf_size must be a finite positive number");
        if(frame_policy_name_ == "override" && frame_id_override_.empty())
            throw std::invalid_argument("frame_id_override is required for override frame_policy");

        resolved_input_topic_ = nh_.resolveName(input_topic_, true);
        resolved_output_topic_ = nh_.resolveName(output_topic_, true);
        if(topics_form_self_loop(resolved_input_topic_,resolved_output_topic_))
            throw std::invalid_argument(
                "input_topic and output_topic resolve/remap to the same ROS name: " +
                resolved_input_topic_);
    }

    void connect_callback(const ros::SingleSubscriberPublisher&)
    {
        if(!subscription_state_.update(1))
            return;

        gate_.reset();
        input_sub_ = nh_.subscribe(input_topic_,
                                   static_cast<std::uint32_t>(input_queue_size_),
                                   &StaticPointcloudVizNode::pointcloud_callback,this);
        ROS_INFO_STREAM("static_pointcloud_viz_node subscribed to " << resolved_input_topic_);
    }

    void disconnect_callback(const ros::SingleSubscriberPublisher&)
    {
        if(!subscription_state_.update(output_pub_.getNumSubscribers()) ||
           subscription_state_.input_required())
            return;

        input_sub_.shutdown();
        gate_.reset();
        ROS_INFO_STREAM("static_pointcloud_viz_node released " << resolved_input_topic_
                        << " because no output subscriber remains");
    }

    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
    {
        if(!gate_.should_process(output_pub_.getNumSubscribers(),
                                 ros::SteadyTime::now().toSec()))
            return;

        try
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
                new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*input_msg,*input_cloud);
            pcl::PointCloud<pcl::PointXYZ> output_cloud =
                downsample_pointcloud(input_cloud,voxel_leaf_size_);

            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(output_cloud,output_msg);
            output_msg.header = make_output_header(input_msg->header,frame_policy_,
                                                   frame_id_override_,stamp_policy_,
                                                   ros::Time::now());
            output_pub_.publish(output_msg);
        }
        catch(const std::exception& error)
        {
            ROS_WARN_STREAM_THROTTLE(5.0,"static pointcloud visualization failed: "
                                     << error.what());
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher output_pub_;
    ros::Subscriber input_sub_;

    std::string input_topic_;
    std::string output_topic_;
    std::string resolved_input_topic_;
    std::string resolved_output_topic_;
    int input_queue_size_ = 1;
    int output_queue_size_ = 1;
    double max_rate_hz_ = 0.5;
    double voxel_leaf_size_ = 0.20;
    std::string frame_policy_name_;
    std::string frame_id_override_;
    std::string stamp_policy_name_;
    FramePolicy frame_policy_ = FramePolicy::PRESERVE_INPUT;
    StampPolicy stamp_policy_ = StampPolicy::NOW_IF_ZERO;
    VisualizationGate gate_;
    LazySubscriptionState subscription_state_;
};

}  // namespace freedom

int main(int argc,char** argv)
{
    ros::init(argc,argv,"static_pointcloud_viz");
    try
    {
        freedom::StaticPointcloudVizNode node;
        ros::spin();
    }
    catch(const std::exception& error)
    {
        ROS_FATAL_STREAM("static_pointcloud_viz_node configuration error: " << error.what());
        return 1;
    }
    return 0;
}
