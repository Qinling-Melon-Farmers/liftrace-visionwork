#ifndef LIFTRACE_LOCAL_SEGMENT_SERVICE_H
#define LIFTRACE_LOCAL_SEGMENT_SERVICE_H
#include <plan_env/sdf_map.h>
#include <plan_manage/CheckLocalSegments.h>
#include <stdexcept>

namespace fast_planner {
// Lives in the planner's existing single-threaded callback queue. No second
// map, search, optimizer, goal publication, or mutation of planner/FSM state.
class LocalSegmentService {
 public:
  void init(ros::NodeHandle& nh, const SDFMap::Ptr& map) {
    bool enabled=false;
    nh.param("research/local_segment_probe/enabled",enabled,false);
    if (!enabled) return;
    map_=map;
    nh.param("research/local_segment_probe/max_length",limits_.max_length,4.0);
    nh.param("research/local_segment_probe/max_wall_ms",limits_.max_wall_ms,3.0);
    nh.param("research/local_segment_probe/max_map_age",limits_.max_age,1.0);
    // These hard caps bound work even if caller/config supplies excessive values.
    int segments,voxels;
    nh.param("research/local_segment_probe/max_segments",segments,32);
    nh.param("research/local_segment_probe/max_voxels",voxels,4096);
    if (segments<=0 || segments>32 || voxels<=0 || voxels>8192 ||
        !std::isfinite(limits_.max_length) || limits_.max_length<=0 || limits_.max_length>7 ||
        !std::isfinite(limits_.max_wall_ms) || limits_.max_wall_ms<=0 || limits_.max_wall_ms>5 ||
        !std::isfinite(limits_.max_age) || limits_.max_age<=0 || limits_.max_age>2)
      throw std::invalid_argument("invalid local probe resource limits");
    limits_.max_segments=segments;limits_.max_voxels=voxels;
    session_=std::to_string(ros::WallTime::now().toNSec());
    server_=nh.advertiseService("research/check_local_segments",&LocalSegmentService::query,this);
  }
 private:
  static void point(const Eigen::Vector3d& p,geometry_msgs::Point& out) {
    out.x=p.x();out.y=p.y();out.z=p.z();
  }
  bool query(plan_manage::CheckLocalSegments::Request& req,
             plan_manage::CheckLocalSegments::Response& res) {
    const auto start=ros::WallTime::now();
    const auto view=map_->localCloudView();
    const auto now=ros::Time::now();
    res.header.stamp=now;res.header.frame_id=view.frame;
    res.request_id=req.request_id;res.planner_session=session_;
    res.map_revision=view.revision;res.map_stamp.fromSec(view.stamp);
    point(view.lower,res.window_min);point(view.upper,res.window_max);
    res.resolution=view.resolution;
    res.known_free_proven=false;res.requires_trajectory_validation=true;
    if (req.starts.size()!=req.ends.size() || req.starts.empty() || req.starts.size()>limits_.max_segments) {
      res.reason="invalid_or_oversized_batch";return true;
    }
    res.status.assign(req.starts.size(),NOT_CHECKED);
    if (req.header.frame_id!=view.frame || req.header.stamp.isZero() ||
        now<req.header.stamp || (now-req.header.stamp).toSec()>.5) {
      res.reason="request_frame_or_age";return true;
    }
    if (!last_query_.isZero() && (start-last_query_).toSec()<.2) {
      res.reason="query_rate_limit";return true;
    }
    last_query_=start;
    std::vector<Segment> segments;
    for (size_t i=0;i<req.starts.size();++i) {
      const auto &a=req.starts[i],&b=req.ends[i];
      segments.push_back({Eigen::Vector3d(a.x,a.y,a.z),Eigen::Vector3d(b.x,b.y,b.z)});
    }
    const auto result=probeSegments(view,segments,limits_,now.toSec(),[&](const Eigen::Vector3i& id) {
      if (!map_->isInMap(id)) return -1;
      return map_->isKnownOccupied(id) ? 1 : 0;
    });
    res.accepted=result.accepted;res.reason=result.reason;res.status=result.status;
    res.checked_voxels=result.checked_voxels;
    res.work_ms=(ros::WallTime::now()-start).toSec()*1000.;
    return true;
  }
  SDFMap::Ptr map_;
  ProbeLimits limits_;
  ros::ServiceServer server_;
  ros::WallTime last_query_;
  std::string session_;
};
}
#endif
