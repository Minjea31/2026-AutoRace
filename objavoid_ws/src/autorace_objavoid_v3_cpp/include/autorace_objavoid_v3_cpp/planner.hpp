// Free-space/Gap -> 통로 Lattice(5차 다항식) -> 차체 충돌·운동학 검사 -> DP 선택(Python planner.py).
#pragma once
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/perception.hpp"
#include "autorace_objavoid_v3_cpp/road.hpp"

namespace objavoid {

struct Path
{
  std::vector<double> s, d, d1, d2, psi, kappa, clear, dsteer, bclear;
  double cost = 0;
  bool complete = false;
  std::vector<Pt2> nodes;
  double stamp = 0;
  double end_s() const {return s.back();}
  void state_at(double x, double & d0, double & v0, double & a0) const
  {
    x = std::min(std::max(x, s.front()), s.back());
    d0 = interp(x, s, d);
    v0 = interp(x, s, d1);
    a0 = interp(x, s, d2);
  }
  double d_at(double x) const
  {
    if (x < s.front() || x > s.back()) {return kNaN;}
    return interp(x, s, d);
  }
};
using PathPtr = std::shared_ptr<const Path>;

struct Obs {double cs, cd, hs, hd;};
std::vector<Obs> obstacle_array(const std::vector<Box> & boxes);

// 회전 차체 직사각형과 장애물 AABB 사이 분리축(SAT) 간격 최소값. 음수 = 겹침. 장애물 없으면 inf.
double body_clearance(double s, double d, double psi, const std::vector<Obs> & obs, const Params & p);
double boundary_clearance(double s, double d, double psi, const Boundary & b, const Params & p);

struct PlanStart {double s, d, v, a;};  // (s0, d0, d0', d0'')

struct PlanDiag
{
  std::vector<double> stations;
  std::vector<std::pair<double, std::vector<std::pair<double, double>>>> gaps;
  std::vector<std::pair<double, std::vector<double>>> layers;
  std::vector<std::vector<Pt2>> cand;
  std::map<std::string, int> rejected;
  long edges = 0, feasible = 0;
  bool has_escape = false;
  double esc_free = 0, esc_steer = 0, esc_hold = 0;
  int esc_n = 0;
};

class Planner
{
public:
  Planner(const Params & p, const Boundary * boundary) : p_(p), boundary_(boundary) {}
  void set_boundary(const Boundary * b) {boundary_ = b;}
  double plan_limit(const std::vector<Box> & boxes) const;
  void free_intervals(double sk, const std::vector<Obs> & obs, std::vector<std::pair<double, double>> & free,
    std::vector<std::pair<double, double>> & gaps) const;
  std::vector<double> nodes_for(double sk, const std::vector<std::pair<double, double>> & intervals,
    const Path * prev, double keep) const;
  std::vector<double> stations(double s0, const std::vector<Obs> & obs, double & s_end) const;
  // w_cons < 0 이면 기본값(p.w_cons)
  PathPtr plan(const PlanStart & start, const std::vector<Box> & boxes, const VisibilityGrid * grid,
    const Path * prev, double w_cons = -1.0, double stamp = 0.0);
  PathPtr escape(double s0, double d0, double psi0, double delta0, const std::vector<Box> & boxes, double stamp);
  // s_from 이후 경로의 (장애물 여유 부족, 경계 여유 부족) — 계획 당시보다 얼마나 나빠졌는지(양수=나빠짐)
  std::pair<double, double> path_clearance(const Path & path, double s_from, const std::vector<Box> & boxes) const;
  double verified_until(const Path & path, double s_from, const VisibilityGrid & grid, const Boundary * lane) const;

  PlanDiag last;
  std::pair<double, double> last_ego_clear{0, 0};
  bool has_ego_clear = false;

private:
  void recovery_need(double travel, double ramp, double eo, double eb, double & need_o, double & need_b) const;
  double unknown_cost(const VisibilityGrid * grid, double sk, double node) const;
  const Params & p_;
  const Boundary * boundary_;
  std::vector<Box> boxes_;
  std::pair<double, double> ego_{kInf, kInf};
};

}  // namespace objavoid
