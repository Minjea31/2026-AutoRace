#include "autorace_objavoid_v3_cpp/outline.hpp"
#include <algorithm>
#include <opencv2/core.hpp>
#include <map>
namespace objavoid {
namespace {
struct Segment {
  XY a,b,n; double dx,dy,inv,length;
  Segment(XY first,XY last):a(first),b(last),dx(b.x-a.x),dy(b.y-a.y),
    inv(1/(dx*dx+dy*dy)),length(std::sqrt(1/inv)) {n={-dy/length,dx/length};}
  double squared(const XY & q,double * along=nullptr) const {
    const double t=std::clamp(((q.x-a.x)*dx+(q.y-a.y)*dy)*inv,0.0,1.0);
    if(along) *along=t*length;
    const double x=q.x-a.x-t*dx,y=q.y-a.y-t*dy;return x*x+y*y;
  }
};
}
RoadOutline::RoadOutline(const Params & p) {
  const double w=p.fork_road_width/2,n=p.start_lane_width/2;
  const double ramp=(p.road_outline_length-p.road_outline_straight)/2;
  if(ramp<=0 || n<=0 || w<=n) throw std::invalid_argument("invalid road outline dimensions");
  for(double side:{-1.0,1.0}) {
    const std::vector<XY> v{{-1,side*n},{0,side*n},{ramp,side*w},
      {ramp+p.road_outline_straight,side*w},{p.road_outline_length,side*n},
      {p.road_outline_length+1,side*n}};
    for(size_t i=1;i<v.size();++i) edges.push_back({v[i-1],v[i]});
  }
}
double RoadOutline::distance(const XY & q) const {
  double best=kInf;for(const auto & e:edges) best=std::min(best,Segment(e.first,e.second).squared(q));
  return std::sqrt(best);
}
bool RoadOutline::island_region(const XY & q) const {
  return q.x>edges[1].second.x && q.x<edges[2].second.x && std::abs(q.y)<0.48;
}
OutlineFit RoadOutline::fit(const std::vector<XY> & input,bool acquire) const {
  // Spatial density rejects isolated speckles; one vote per cell prevents thick
  // paint or a dense noise cluster from dominating the geometric objective.
  std::map<std::pair<int,int>,XY> grid;
  for(const auto & q:input) if(std::isfinite(q.x)&&std::isfinite(q.y)) grid.emplace(std::make_pair(std::lround(q.x/0.03),std::lround(q.y/0.03)),q);
  std::vector<XY> pts;
  for(const auto & cell:grid) {
    int neighbours=0;
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy)
      neighbours+=grid.count({cell.first.first+dx,cell.first.second+dy});
    if(neighbours>=2) pts.push_back(cell.second);
  }
  if(pts.size()>192) {std::vector<XY> sample;for(size_t i=0;i<192;++i) sample.push_back(pts[i*pts.size()/192]);pts.swap(sample);}
  OutlineFit best;if(pts.size()<12) return best;
  std::vector<Segment> segments;for(const auto & e:edges) segments.emplace_back(e.first,e.second);
  double best_cost=kInf;
  auto optimize=[&](double initial_s,double initial_d,double initial_yaw) {
    OutlineFit out;out.s=initial_s;out.d=initial_d;out.yaw=initial_yaw;
    for(int iteration=0;iteration<10;++iteration) {
      cv::Matx33d h=cv::Matx33d::zeros();cv::Vec3d rhs(0,0,0);
      const double c=std::cos(out.yaw),sn=std::sin(out.yaw);
      std::vector<int> count(segments.size());
      std::vector<double> lo(segments.size(),kInf),hi(segments.size(),-kInf),residuals;
      double cost=0;int exterior_count=0;
      for(const auto & q:pts) {
        const XY local{c*(q.x-out.s)+sn*(q.y-out.d),-sn*(q.x-out.s)+c*(q.y-out.d)};
        if(island_region(local)) continue;
        ++exterior_count;
        double dist=kInf,along=0;int index=0;
        for(size_t k=0;k<segments.size();++k) {double a;const double z=segments[k].squared(local,&a);
          if(z<dist) {dist=z;along=a;index=static_cast<int>(k);}}
        // Bounded robust objective includes unmatched evidence instead of rewarding
        // the accidental fit of a tiny subset of points.
        cost+=std::min(dist,0.12*0.12);
        if(dist>0.18*0.18) continue;
        const auto & e=segments[index];
        const double r=e.n.x*(local.x-e.a.x)+e.n.y*(local.y-e.a.y);
        const double u=r/0.06,weight=1/std::pow(1+u*u,2);
        const cv::Vec3d j(-c*e.n.x+sn*e.n.y,-sn*e.n.x-c*e.n.y,
          e.n.x*local.y-e.n.y*local.x);
        h+=weight*(j*j.t());rhs-=weight*j*r;
        if(dist<0.07*0.07) {
          residuals.push_back(dist);++count[index];lo[index]=std::min(lo[index],along);hi[index]=std::max(hi[index],along);
        }
      }
      bool diagonal=false,straight=false;
      for(size_t k=0;k<segments.size();++k) if(count[k]>=4 && hi[k]-lo[k]>=0.12) {
        if(std::abs(segments[k].dy)>0.1) diagonal=true;else straight=true;
      }
      out.longitudinal=diagonal&&straight;out.hits=static_cast<int>(residuals.size());
      if(out.hits<12) return;
      std::sort(residuals.begin(),residuals.end());
      const size_t kept=std::max(size_t(1),residuals.size()*4/5);
      double error=0;for(size_t i=0;i<kept;++i) error+=residuals[i];out.error=std::sqrt(error/kept);
      out.inlier_ratio=double(out.hits)/std::max(1,exterior_count);
      if(!out.longitudinal) {
        if(std::abs(out.s)>1e-9) {out.s=0;continue;}
        for(int k=0;k<3;++k) h(0,k)=h(k,0)=0;
        h(0,0)=1;rhs[0]=0;
      }
      cv::Vec3d delta;if(!cv::solve(h,rhs,delta,cv::DECOMP_SVD)) return;
      // Evaluate the actual current pose, not the unevaluated final iteration step.
      if(iteration==9 || cv::norm(delta)<1e-5) {
        out.valid=out.error<0.04 && out.inlier_ratio>=0.50;
        const double objective=cost/std::max(1,exterior_count)+1e-6*(out.s*out.s+out.d*out.d+out.yaw*out.yaw);
        if(out.valid && objective<best_cost) {best_cost=objective;best=out;}
        return;
      }
      out.s+=std::clamp(delta[0],-0.12,0.12);out.d+=std::clamp(delta[1],-0.08,0.08);
      out.yaw+=std::clamp(delta[2],-0.06,0.06);
      if(std::abs(out.s)>(acquire?1.2:0.45)||std::abs(out.d)>0.5||std::abs(out.yaw)>0.45) return;
    }
  };
  optimize(0,0,0);
  if(acquire || !best.valid) {
    for(double s:{-0.7,0.0,0.7}) for(double d:{-0.25,0.0,0.25})
      for(double yaw:{-0.20,0.0,0.20}) optimize(s,d,yaw);
  }
  return best;
}
}  // namespace objavoid
