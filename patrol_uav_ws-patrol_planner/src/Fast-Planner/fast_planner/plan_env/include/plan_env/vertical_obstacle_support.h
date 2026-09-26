#ifndef PLAN_ENV_VERTICAL_OBSTACLE_SUPPORT_H
#define PLAN_ENV_VERTICAL_OBSTACLE_SUPPORT_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace fast_planner {

// Evidence for extending a measured obstacle through the full flight height.
// Ordinary 3-D occupancy is independent of this additional classification.
class VerticalObstacleSupport {
 public:
  VerticalObstacleSupport(int nx, int ny, double radius_cells,
                          int min_points, double min_height_span)
      : nx_(nx), ny_(ny), radius_(static_cast<int>(std::ceil(radius_cells))),
        radius_squared_(radius_cells * radius_cells), min_points_(min_points),
        min_span_(min_height_span), counts_(nx * ny, 0),
        lows_(nx * ny, std::numeric_limits<double>::infinity()),
        highs_(nx * ny, -std::numeric_limits<double>::infinity()),
        cache_(nx * ny, -1) {}

  void observe(int x, int y, double z) {
    if (!inside(x, y) || !std::isfinite(z)) return;
    const int i = x * ny_ + y;
    ++counts_[i];
    lows_[i] = std::min(lows_[i], z);
    highs_[i] = std::max(highs_[i], z);
  }

  // Call after all observations have been supplied for this cloud.
  bool supported(int x, int y) {
    if (!inside(x, y)) return false;
    const int i = x * ny_ + y;
    if (cache_[i] >= 0) return cache_[i] == 1;
    int count = 0;
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    for (int dx = std::max(0, x - radius_); dx <= std::min(nx_ - 1, x + radius_); ++dx)
      for (int dy = std::max(0, y - radius_); dy <= std::min(ny_ - 1, y + radius_); ++dy) {
        // Measure the support disk in grid coordinates, without rounding its
        // radius out to a square. At 10 cm resolution 15 cm includes (1,1),
        // but excludes (2,0) and (2,2). Voxel quantization still applies.
        const double distance_squared = (dx-x)*(dx-x) + (dy-y)*(dy-y);
        if (distance_squared > radius_squared_ + 1e-9) continue;
        const int j = dx * ny_ + dy;
        count += counts_[j];
        low = std::min(low, lows_[j]);
        high = std::max(high, highs_[j]);
      }
    const bool result = count >= min_points_ && high - low + 1e-9 >= min_span_;
    cache_[i] = result ? 1 : 0;
    return result;
  }

 private:
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < nx_ && y < ny_; }
  int nx_, ny_, radius_;
  double radius_squared_;
  int min_points_;
  double min_span_;
  std::vector<int> counts_;
  std::vector<double> lows_, highs_;
  std::vector<signed char> cache_;
};

// Select a measured middle-height band independently for each XY-connected
// supported structure. No tree labels or simulation geometry are consumed.
// Feed only vertically supported observations, build(), observeBand(), then
// query source(). If a component has no measured middle return, keep its full
// supported footprint rather than inventing a narrower unobserved silhouette.
class MiddleHeightColumnSelector {
 public:
  MiddleHeightColumnSelector(int nx, int ny, double link_radius_cells,
                             double low_ratio, double high_ratio)
      : nx_(nx), ny_(ny), radius_(std::max(1.0, link_radius_cells)),
        low_ratio_(low_ratio), high_ratio_(high_ratio), labels_(nx*ny, -1),
        lows_(nx*ny, std::numeric_limits<double>::infinity()),
        highs_(nx*ny, -std::numeric_limits<double>::infinity()),
        middle_lows_(nx*ny, std::numeric_limits<double>::infinity()) {}
  void observe(int x, int y, double z) {
    if (!inside(x,y) || !std::isfinite(z)) return;
    const int i=x*ny_+y;
    lows_[i]=std::min(lows_[i],z); highs_[i]=std::max(highs_[i],z);
  }
  void build() {
    const int r=static_cast<int>(std::ceil(radius_));
    std::vector<int> queue;
    for (int seed=0; seed<nx_*ny_; ++seed) {
      if (labels_[seed]>=0 || !std::isfinite(lows_[seed])) continue;
      const int label=bands_.size();
      labels_[seed]=label; queue.clear(); queue.push_back(seed);
      double low=lows_[seed], high=highs_[seed];
      for (size_t head=0; head<queue.size(); ++head) {
        const int i=queue[head], x=i/ny_, y=i%ny_;
        low=std::min(low,lows_[i]); high=std::max(high,highs_[i]);
        for (int dx=-r; dx<=r; ++dx) for (int dy=-r; dy<=r; ++dy) {
          if (dx*dx+dy*dy>radius_*radius_+1e-9 || !inside(x+dx,y+dy)) continue;
          const int j=(x+dx)*ny_+y+dy;
          if (labels_[j]<0 && std::isfinite(lows_[j])) {
            labels_[j]=label; queue.push_back(j);
          }
        }
      }
      bands_.push_back({low+(high-low)*low_ratio_, low+(high-low)*high_ratio_, false});
    }
  }
  void observeBand(int x, int y, double z) {
    if (!inside(x,y) || !std::isfinite(z)) return;
    const int label=labels_[x*ny_+y];
    if (label>=0 && inBand(bands_[label],z)) {
      bands_[label].observed=true;
      middle_lows_[x*ny_+y]=std::min(middle_lows_[x*ny_+y],z);
    }
  }
  bool source(int x, int y, double z) const {
    if (!inside(x,y) || !std::isfinite(z)) return false;
    const int label=labels_[x*ny_+y];
    return label>=0 && (!bands_[label].observed || inBand(bands_[label],z));
  }
  // Fill each observed middle silhouette, not just its hollow surface ring.
  // Convex hull is computed per component, before the single physical XY
  // inflation. No global hull across unrelated trees; no assumed tree center.
  template<class Emit> void forEachFootprint(Emit emit) const {
    std::vector<std::vector<Cell>> groups(bands_.size());
    std::vector<double> bottoms(bands_.size(),std::numeric_limits<double>::infinity());
    for (int i=0; i<nx_*ny_; ++i) {
      const int label=labels_[i];
      if (label<0) continue;
      const double z=bands_[label].observed ? middle_lows_[i] : lows_[i];
      if (!std::isfinite(z)) continue;
      groups[label].push_back({i/ny_,i%ny_}); bottoms[label]=std::min(bottoms[label],z);
    }
    for (size_t k=0; k<groups.size(); ++k) {
      const auto hull=convexHull(groups[k]);
      if (hull.empty()) continue;
      if (hull.size()==1) {emit(hull[0].first,hull[0].second,bottoms[k]);continue;}
      int min_y=ny_,max_y=0;
      for (const auto& p:hull) {min_y=std::min(min_y,p.second);max_y=std::max(max_y,p.second);}
      for (int y=min_y; y<=max_y; ++y) {
        double left=nx_,right=-1.;
        for (size_t j=0;j<hull.size();++j) {
          const auto& a=hull[j]; const auto& b=hull[(j+1)%hull.size()];
          if (y<std::min(a.second,b.second) || y>std::max(a.second,b.second)) continue;
          if (a.second==b.second) {
            left=std::min(left,double(std::min(a.first,b.first)));
            right=std::max(right,double(std::max(a.first,b.first)));
          } else {
            const double x=a.first+(b.first-a.first)*double(y-a.second)/(b.second-a.second);
            left=std::min(left,x);right=std::max(right,x);
          }
        }
        for (int x=int(std::ceil(left-1e-9)); x<=int(std::floor(right+1e-9)); ++x)
          emit(x,y,bottoms[k]);
      }
    }
  }
 private:
  using Cell=std::pair<int,int>;
  static long cross(const Cell& a,const Cell& b,const Cell& c) {
    return long(b.first-a.first)*(c.second-a.second)-long(b.second-a.second)*(c.first-a.first);
  }
  static std::vector<Cell> convexHull(std::vector<Cell> points) {
    if (points.size()<3) return points;
    std::sort(points.begin(),points.end());
    std::vector<Cell> hull;
    for (const auto& p:points) {
      while (hull.size()>=2 && cross(hull[hull.size()-2],hull.back(),p)<=0) hull.pop_back();
      hull.push_back(p);
    }
    const size_t lower=hull.size();
    for (int i=int(points.size())-2;i>=0;--i) {
      const auto& p=points[i];
      while (hull.size()>lower && cross(hull[hull.size()-2],hull.back(),p)<=0) hull.pop_back();
      hull.push_back(p);
    }
    hull.pop_back();return hull;
  }
  struct Band { double low, high; bool observed; };
  static bool inBand(const Band& band,double z) {
    return z+1e-9>=band.low && z-1e-9<=band.high;
  }
  bool inside(int x,int y) const { return x>=0 && y>=0 && x<nx_ && y<ny_; }
  int nx_,ny_;
  double radius_,low_ratio_,high_ratio_;
  std::vector<int> labels_;
  std::vector<double> lows_,highs_,middle_lows_;
  std::vector<Band> bands_;
};

// Union of upward forbidden spans. Normal 3-D occupancy is kept separately.
// A supported source voxel forbids going OVER its XY footprint, but does not
// invent material underneath an overhang. No height-dependent XY dilation.
class UpwardObstacleColumns {
 public:
  UpwardObstacleColumns(int nx, int ny, int nz)
      : ny_(ny), nz_(nz), bases_(nx * ny, nz) {}
  void mark(int x, int y, int inflated_source_bottom) {
    int& base = bases_[x * ny_ + y];
    base = std::min(base, std::max(0, inflated_source_bottom));
  }
  int bottom(int x, int y) const { return bases_[x * ny_ + y]; }
  bool occupied(int x, int y, int z) const {
    return z >= bottom(x, y) && z < nz_;
  }
 private:
  int ny_, nz_;
  std::vector<int> bases_;
};
}  // namespace fast_planner
#endif
