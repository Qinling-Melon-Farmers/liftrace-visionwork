#include <uav_mission/obstacle_view.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <cstring>

class ObstacleView {
  using Marker = visualization_msgs::Marker;
  ros::NodeHandle nh_, private_{"~"};
  ros::Publisher map_pub_, path_pub_;
  ros::Subscriber map_sub_, path_sub_;
  ros::WallTimer timer_;
  sensor_msgs::PointCloud2ConstPtr latest_;
  std::string frame_;
  double resolution_, rate_, line_width_;
  int max_blocks_, max_path_points_;

  Marker marker(const std::string& ns, int type) const {
    Marker m;
    m.header.frame_id = frame_;
    // World-fixed visualization, including clouds whose upstream stamp is zero.
    m.header.stamp = ros::Time(0);
    m.ns = ns;
    m.id = 0;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1;
    m.color.a = 1;
    return m;
  }
  void clear(ros::Publisher& pub, const std::string& ns, int type) {
    auto m = marker(ns, type);
    m.action = Marker::DELETE;
    pub.publish(m);
  }
  void cloud(const sensor_msgs::PointCloud2ConstPtr& msg) { latest_ = msg; }
  void tick(const ros::WallTimerEvent&) {
    if (!latest_) return;
    auto msg = latest_;
    latest_.reset();
    try {
      if (msg->header.frame_id != frame_)
        throw std::invalid_argument("occupancy frame differs from fixed_frame");
      if (!msg->width || !msg->height) {
        clear(map_pub_, "obstacles", Marker::CUBE_LIST);
        return;
      }
      if (!msg->point_step || uint64_t(msg->row_step) < uint64_t(msg->width) * msg->point_step ||
          uint64_t(msg->data.size()) < uint64_t(msg->row_step) * msg->height)
        throw std::invalid_argument("invalid PointCloud2 dimensions");
      for (const std::string name : {"x", "y", "z"}) {
        bool found = false;
        for (const auto& f : msg->fields)
          if (f.name == name && f.datatype == sensor_msgs::PointField::FLOAT32 &&
              f.count == 1 && uint64_t(f.offset) + 4 <= msg->point_step) found = true;
        if (!found) throw std::invalid_argument("expected FLOAT32 xyz cloud");
      }
      if (msg->is_bigendian) throw std::invalid_argument("big endian cloud unsupported");
      std::vector<geometry_msgs::Point> points;
      points.reserve(size_t(msg->width) * msg->height);
      // Row-aware parsing also handles organized clouds with row padding.
      for (uint32_t row = 0; row < msg->height; ++row) {
        for (uint32_t col = 0; col < msg->width; ++col) {
          geometry_msgs::Point p;
          for (const auto& f : msg->fields) {
            if (f.name != "x" && f.name != "y" && f.name != "z") continue;
            float value;
            std::memcpy(&value, &msg->data[size_t(row)*msg->row_step + size_t(col)*msg->point_step + f.offset], 4);
            if (f.name == "x") p.x = value;
            else if (f.name == "y") p.y = value;
            else p.z = value;
          }
          points.push_back(p);
        }
      }
      const auto result = obstacle_view::surface(points, resolution_, size_t(max_blocks_));
      auto m = marker("obstacles", Marker::CUBE_LIST);
      m.scale.x = m.scale.y = m.scale.z = result.resolution;
      m.color.r = 0.25; m.color.g = 0.55; m.color.b = 0.85;
      m.points = result.points;
      if (m.points.empty()) m.action = Marker::DELETE;
      map_pub_.publish(m);
      ROS_INFO_THROTTLE(30, "Obstacle view: %zu input points, %zu surface blocks, %.2f m display resolution",
                        points.size(), m.points.size(), result.resolution);
    } catch (const std::exception& e) {
      clear(map_pub_, "obstacles", Marker::CUBE_LIST);
      ROS_ERROR_THROTTLE(5, "Obstacle view rejected cloud: %s", e.what());
    }
  }
  void trajectory(const Marker::ConstPtr& source) {
    if (source->action == Marker::DELETEALL ||
        (source->id == 300 && source->action == Marker::DELETE)) {
      clear(path_pub_, "planned_path", Marker::LINE_STRIP);
      return;
    }
    if (source->id != 300 || source->action != Marker::ADD) return;
    try {
      if (source->header.frame_id != frame_ || source->type != Marker::SPHERE_LIST)
        throw std::invalid_argument("unexpected final trajectory frame/type");
      auto m = marker("planned_path", Marker::LINE_STRIP);
      m.pose = source->pose;
      m.scale.x = line_width_;
      m.color.r = 1; m.color.g = 0.25; m.color.b = 0.05;
      m.points = obstacle_view::path(source->points, size_t(max_path_points_));
      if (m.points.size() < 2) m.action = Marker::DELETE;
      path_pub_.publish(m);
    } catch (const std::exception& e) {
      clear(path_pub_, "planned_path", Marker::LINE_STRIP);
      ROS_ERROR_THROTTLE(5, "Obstacle view rejected trajectory: %s", e.what());
    }
  }
 public:
  ObstacleView() {
    private_.param<std::string>("fixed_frame", frame_, "camera_init");
    private_.param("display_resolution", resolution_, 0.10);
    private_.param("update_rate", rate_, 1.0);
    private_.param("max_blocks", max_blocks_, 12000);
    private_.param("max_path_points", max_path_points_, 1000);
    private_.param("line_width", line_width_, 0.03);
    if (frame_.empty() || !std::isfinite(resolution_) || resolution_ <= 0 ||
        !std::isfinite(rate_) || rate_ <= 0 || rate_ > 10 || max_blocks_ < 8 ||
        max_blocks_ > 12000 || max_path_points_ < 2 || max_path_points_ > 1000 ||
        !std::isfinite(line_width_) || line_width_ <= 0)
      throw std::invalid_argument("invalid obstacle view parameters");
    map_pub_ = nh_.advertise<Marker>("/navigation/obstacle_blocks", 1, true);
    path_pub_ = nh_.advertise<Marker>("/navigation/planned_path", 1, true);
    map_sub_ = nh_.subscribe("/sdf_map/occupancy_inflate", 1, &ObstacleView::cloud, this);
    // Upstream multiplexes DELETE/ADD for several IDs; preserve the final ADD.
    path_sub_ = nh_.subscribe("/planning_vis/trajectory", 20, &ObstacleView::trajectory, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / rate_), &ObstacleView::tick, this);
    clear(map_pub_, "obstacles", Marker::CUBE_LIST);
    clear(path_pub_, "planned_path", Marker::LINE_STRIP);
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_view");
  try { ObstacleView view; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
