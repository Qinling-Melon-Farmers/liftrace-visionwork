#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

class TFToOdometryNode
{
public:
    TFToOdometryNode(ros::NodeHandle& nh_private)
    {
        // Initialize ROS node
        ros::NodeHandle nh_;

        // Initialize TF listener
        tf_listener_ = new tf::TransformListener();

        // Get parameters from launch or param server
        nh_private.param<std::string>("base_link_frame_id_", base_link_frame_id_, std::string("aft_mapped"));
        nh_private.param<std::string>("map_frame_id_", map_frame_id_, std::string("camera_init"));
        nh_private.param<std::string>("odom_pub_topic", odom_pub_topic_, std::string("/akm_car/odom"));
        nh_private.param<std::string>("pose_pub_topic", pose_pub_topic_, std::string("/akm_car/pose"));
        nh_private.param<bool>("use_odometry", use_odometry_, true);

        // Create publishers
        if (use_odometry_) {
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_pub_topic_, 10);
        } else {
            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_pub_topic_, 10);
        }

        // Set the rate for publishing
        ros::Rate rate(10.0);

        while (ros::ok())
        {
            try
            {
                tf::StampedTransform transform;
                tf_listener_->waitForTransform(map_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(3.0));
                tf_listener_->lookupTransform(map_frame_id_, base_link_frame_id_, ros::Time(0), transform);

                ros::Time current_time = ros::Time::now();

                if (use_odometry_) {
                    nav_msgs::Odometry odom;
                    odom.header.stamp = current_time;
                    odom.header.frame_id = map_frame_id_;
                    odom.child_frame_id = base_link_frame_id_;

                    odom.pose.pose.position.x = transform.getOrigin().x();
                    odom.pose.pose.position.y = transform.getOrigin().y();
                    odom.pose.pose.position.z = transform.getOrigin().z();

                    tf::Quaternion q = transform.getRotation();
                    odom.pose.pose.orientation.x = q.x();
                    odom.pose.pose.orientation.y = q.y();
                    odom.pose.pose.orientation.z = q.z();
                    odom.pose.pose.orientation.w = q.w();

                    // Optional: set velocity to 0
                    odom.twist.twist.linear.x = 0.0;
                    odom.twist.twist.linear.y = 0.0;
                    odom.twist.twist.linear.z = 0.0;
                    odom.twist.twist.angular.x = 0.0;
                    odom.twist.twist.angular.y = 0.0;
                    odom.twist.twist.angular.z = 0.0;

                    odom_pub_.publish(odom);
                } else {
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = current_time;
                    pose.header.frame_id = map_frame_id_;

                    pose.pose.position.x = transform.getOrigin().x();
                    pose.pose.position.y = transform.getOrigin().y();
                    pose.pose.position.z = transform.getOrigin().z();

                    tf::Quaternion q = transform.getRotation();
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();

                    pose_pub_.publish(pose);
                }

            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            rate.sleep();
        }
    }

    ~TFToOdometryNode()
    {
        delete tf_listener_;
    }

private:
    ros::NodeHandle nh_;
    tf::TransformListener* tf_listener_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;

    std::string base_link_frame_id_;
    std::string map_frame_id_;
    std::string odom_pub_topic_;
    std::string pose_pub_topic_;
    bool use_odometry_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_to_odometry_node");
    ros::NodeHandle nh_private("~");

    TFToOdometryNode tf_to_odometry_node(nh_private);
    ros::spin();
    return 0;
}
