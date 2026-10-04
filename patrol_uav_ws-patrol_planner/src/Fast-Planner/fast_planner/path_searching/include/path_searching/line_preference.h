#pragma once
#include <Eigen/Core>
#include <algorithm>
namespace fast_planner {
inline double segmentDistanceSquared(const Eigen::Vector3d& p,
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
  const Eigen::Vector3d d=goal-start;
  const double n=d.squaredNorm();
  if(n<1e-12) return (p-start).squaredNorm();
  const double t=std::max(0.0,std::min(1.0,(p-start).dot(d)/n));
  return (p-start-t*d).squaredNorm();
}
inline double lineDeviationCost(const Eigen::Vector3d& p,const Eigen::Vector3d& v,
    const Eigen::Vector3d& a,double dt,const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,double weight) {
  if(weight<=0.0) return 0.0;
  const Eigen::Vector3d mid=p+v*(dt*.5)+a*(dt*dt*.125);
  const Eigen::Vector3d end=p+v*dt+a*(dt*dt*.5);
  return weight*dt*(segmentDistanceSquared(p,start,goal)+
      4*segmentDistanceSquared(mid,start,goal)+segmentDistanceSquared(end,start,goal))/6.;
}
}
