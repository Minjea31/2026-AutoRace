#include "autorace_objavoid_v3_cpp/island.hpp"
#include <algorithm>
#include <cmath>
#include <set>
namespace objavoid {
std::array<XY,6> IslandShape::vertices() const {
  const double h=width/2, c=std::cos(pose.yaw), sn=std::sin(pose.yaw);
  std::array<XY,6> v{{{0,0},{h,h},{length-h,h},{length,0},{length-h,-h},{h,-h}}};
  for(auto & q:v) {const double x=q.x; q.x=pose.s+c*x-sn*q.y; q.y=pose.d+sn*x+c*q.y;}
  return v;
}
XY IslandShape::local(double s,double d) const {
  const double c=std::cos(pose.yaw), sn=std::sin(pose.yaw), x=s-pose.s,y=d-pose.d;
  return {c*x+sn*y,-sn*x+c*y};
}
// SAT lower bound of separation, negative when footprints overlap.
double IslandShape::clearance(double s,double d,double yaw,const Params & p) const {
  if(!pose.valid) return kInf;
  const auto poly=vertices();
  const double c=std::cos(yaw), sn=std::sin(yaw), h=p.body_width/2;
  std::array<XY,4> car{{{p.body_front,h},{p.body_front,-h},{-p.body_rear,-h},{-p.body_rear,h}}};
  for(auto & q:car) {const double x=q.x; q.x=s+c*x-sn*q.y; q.y=d+sn*x+c*q.y;}
  double best=-kInf;
  auto axis=[&](double x,double y) {
    const double n=std::hypot(x,y); x/=n; y/=n;
    double a=kInf,b=-kInf,u=kInf,v=-kInf;
    for(const auto & q:poly) {double t=q.x*x+q.y*y; a=std::min(a,t); b=std::max(b,t);}
    for(const auto & q:car) {double t=q.x*x+q.y*y; u=std::min(u,t); v=std::max(v,t);}
    best=std::max(best,std::max(u-b,a-v));
  };
  axis(c,sn); axis(-sn,c);
  for(size_t i=0;i<6;++i) {auto a=poly[i],b=poly[(i+1)%6]; axis(a.y-b.y,b.x-a.x);}
  return best;
}
bool IslandMapper::update(const std::vector<XY> & input,double reference,const Params * vehicle,double car_s,double car_d,double car_yaw) {
  (void)reference;input_count=input.size();candidate_count=0;evaluations=0;rejection="insufficient_evidence";
  // Bounded deterministic sample and coarse-to-fine matching. No allocations per hypothesis.
  std::vector<XY> pts; pts.reserve(96);
  std::set<std::pair<int,int>> occupied;
  for(const auto & q:input) {
    // External road boundaries are near +/-0.77m; island evidence belongs to the centre corridor.
    if(std::isfinite(q.x) && std::isfinite(q.y) && std::abs(q.y)<0.5 &&
        occupied.emplace(std::lround(q.x/0.02),std::lround(q.y/0.02)).second) pts.push_back(q);
  }
  std::sort(pts.begin(),pts.end(),[](const XY & a,const XY & b) {
    return a.x<b.x || (a.x==b.x && a.y<b.y);
  });
  if(pts.size()>96) {
    std::vector<XY> sampled;sampled.reserve(96);
    for(size_t i=0;i<96;++i) sampled.push_back(pts[i*pts.size()/96]);
    pts.swap(sampled);
  }
  auto unavailable=[&]() {
    ++misses;
    // Missing or rejected observations cannot erase a confirmed static map.
    if(!shape.pose.confirmed) {
      shape.pose.stable=0;
      if(misses>=6) shape.pose.valid=false;
    }
    return false;
  };
  if(pts.size()<10) return unavailable();
  // Reject collinear evidence before template search (also saves the expensive grid fit).
  double mx=0,my=0;for(const auto & q:pts) {mx+=q.x;my+=q.y;} mx/=pts.size();my/=pts.size();
  double xx=0,yy=0,xy=0;
  for(const auto & q:pts) {const double x=q.x-mx,y=q.y-my;xx+=x*x;yy+=y*y;xy+=x*y;}
  const double minor=(xx+yy-std::hypot(xx-yy,2*xy))/(2*pts.size());
  if(minor<0.0001 && !shape.pose.confirmed) {rejection="single_line";return unavailable();}
  rejection="no_corner_match";
  IslandPose best; double bestscore=-kInf;
  std::vector<std::pair<double,IslandPose>> hypotheses;hypotheses.reserve(1500);
  IslandShape base=shape;base.pose={0,0,0,0,0,true,false};const auto local_vertices=base.vertices();
  double edge_dx[6],edge_dy[6],edge_inv[6],edge_len[6];
  for(int k=0;k<6;++k) {
    const auto a=local_vertices[k],b=local_vertices[(k+1)%6];
    edge_dx[k]=b.x-a.x;edge_dy[k]=b.y-a.y;
    edge_inv[k]=1/(edge_dx[k]*edge_dx[k]+edge_dy[k]*edge_dy[k]);edge_len[k]=std::sqrt(1/edge_inv[k]);
  }
  auto evaluate=[&](double s,double d,double yaw) {
    ++evaluations;
    if(std::abs(d)>0.22 || std::abs(yaw)>0.3) return;
    IslandShape sh=shape; sh.pose={s,d,yaw,0,0,true,false};
    if(vehicle && (sh.clearance(car_s,car_d,car_yaw,*vehicle)<vehicle->margin_obstacle ||
        (!shape.pose.confirmed && s<car_s+0.1))) return;
    const auto v=sh.vertices();const double cy=std::cos(yaw),sy=std::sin(yaw);
    int support[6]={}; double px[6]={},py[6]={},xx[6]={},yy[6]={},xy[6]={}; double low[6],high[6];
    std::fill(low,low+6,kInf);std::fill(high,high+6,-kInf); int hits=0; double error=0;
    for(const auto & q:pts) {
      const double qx=cy*(q.x-s)+sy*(q.y-d),qy=-sy*(q.x-s)+cy*(q.y-d);
      if(qx< -0.06 || qx>shape.length+0.06 || std::abs(qy)>shape.width/2+0.06) continue;
      double dist2=kInf; int edge=0;double along=0;
      for(int k=0;k<6;++k) {
        const auto a=local_vertices[k];const double x=qx-a.x,y=qy-a.y;
        const double t=std::clamp((x*edge_dx[k]+y*edge_dy[k])*edge_inv[k],0.0,1.0);
        const double ex=x-t*edge_dx[k],ey=y-t*edge_dy[k],z=ex*ex+ey*ey;
        if(z<dist2) {dist2=z;edge=k;along=t*edge_len[k];}
      }
      if(dist2<0.05*0.05) {++hits; ++support[edge]; error+=dist2;
        px[edge]+=q.x;py[edge]+=q.y;xx[edge]+=q.x*q.x;yy[edge]+=q.y*q.y;xy[edge]+=q.x*q.y;
        low[edge]=std::min(low[edge],along);high[edge]=std::max(high[edge],along);}
    }
    // Require a front diagonal and an independent edge: a straight stripe cannot fix pose.
    auto supported=[&](int k) {
      if(support[k]<4 || high[k]-low[k]<0.12) return false;
      const double n=support[k],a=xx[k]-px[k]*px[k]/n,b=yy[k]-py[k]*py[k]/n;
      const double cross=xy[k]-px[k]*py[k]/n;
      const double observed=0.5*std::atan2(2*cross,a-b);
      const auto v0=v[k],v1=v[(k+1)%6];
      const double expected=std::atan2(v1.y-v0.y,v1.x-v0.x);
      return std::abs(std::cos(observed-expected))>std::cos(0.21);
    };
    const bool corner=(supported(0) && (supported(1) || supported(5))) || (supported(5) && supported(4));
    // Initial localization needs a corner. Once fixed in odom, a visible side
    // can maintain that pose without pretending to measure longitudinal position.
    const bool side_tracking=shape.pose.confirmed && (supported(1)||supported(4));
    if(hits<10 || (!corner && !side_tracking)) return;
    if(!corner && std::abs(s-shape.pose.s)>1e-6) return;
    const double rms=std::sqrt(error/hits);
    double coverage=0;int balanced=0;
    for(int k=0;k<6;++k) if(supported(k)) {coverage+=std::min(high[k]-low[k],0.6);balanced+=std::min(support[k],12);}
    const double score=balanced+20*coverage-60*rms;
    ++candidate_count;
    hypotheses.push_back({score,{s,d,yaw,rms,0,true,false}});
    if(score>bestscore) {bestscore=score; best={s,d,yaw,rms,0,true,false};}
  };
  if(shape.pose.valid) {
    // Reuse the last unconfirmed candidate too: confirmation should not repeat
    // a full global grid search on every image.
    const auto seed=shape.pose;
    for(int a=-1;a<=1;++a) for(int b=-1;b<=1;++b) for(int c=-1;c<=1;++c)
      evaluate(seed.s+a*0.03,seed.d+b*0.03,seed.yaw+c*0.03);
  }
  if(!best.valid && !shape.pose.confirmed) {
    const double lower=std::max(pts.front().x-0.4,vehicle?car_s+0.1:pts.front().x-0.4);
    const double upper=pts.back().x;
    for(double s=lower;s<=upper;s+=0.10)
      for(double d=-0.20;d<=0.201;d+=0.05) for(double y=-0.30;y<=0.301;y+=0.15) evaluate(s,d,y);
    if(best.valid) {const auto coarse=best;
      for(int a=-2;a<=2;++a) for(int b=-2;b<=2;++b) for(int c=-1;c<=1;++c)
        evaluate(coarse.s+a*0.02,coarse.d+b*0.02,coarse.yaw+c*0.03);
    }
  }
  if(!best.valid) return unavailable();
  if(!shape.pose.confirmed) for(const auto & hypothesis:hypotheses) {
    const auto & other=hypothesis.second;
    if(hypothesis.first>=bestscore-0.5 && (std::hypot(other.s-best.s,other.d-best.d)>0.2 ||
        std::abs(wrap(other.yaw-best.yaw))>0.15)) {rejection="ambiguous";return unavailable();}
  }
  const auto old=shape.pose;
  const bool consistent=old.valid && std::hypot(best.s-old.s,best.d-old.d)<0.08 && std::abs(wrap(best.yaw-old.yaw))<0.087;
  if(old.confirmed && !consistent) return unavailable();
  best.stable=consistent?old.stable+1:1; best.confirmed=old.confirmed || best.stable>=3;
  if(consistent) {
    // Confidence increases with fresh observations; pose changes remain smooth in odom coordinates.
    const double alpha=old.confirmed?0.25:0.5;
    best.s=old.s+alpha*(best.s-old.s);best.d=old.d+alpha*(best.d-old.d);
    best.yaw=wrap(old.yaw+alpha*wrap(best.yaw-old.yaw));
  }
  misses=0;rejection="matched";
  shape.pose=best;
  return true;
}
}  // namespace objavoid
