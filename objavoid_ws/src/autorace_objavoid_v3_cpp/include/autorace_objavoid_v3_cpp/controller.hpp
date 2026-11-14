// PD 경로 추종(Python controller.py): steering = atan(L*k_ff) - K_y*e_y - K_psi*e_psi (- K_i*int e_y)
#pragma once
#include "autorace_objavoid_v3_cpp/params.hpp"
#include "autorace_objavoid_v3_cpp/planner.hpp"

namespace objavoid {

struct TrackErr {double rs, rd, rpsi, ey, epsi;};
struct CtrlInfo
{
  bool valid = false;
  double ref_s = 0, ref_d = 0, ref_psi = 0, ey = 0, epsi = 0, k_ff = 0, c_min = 0, room = 0, s_ff = 0;
};

class PathTracker
{
public:
  explicit PathTracker(const Params & p) : p_(p) {reset();}
  void reset();
  TrackErr errors(const Path & path, double s, double d, double psi);
  // -> (speed, steering) 와 info. v_cap: 외부 속도 상한, verified_s < 0 이면 없음
  void step(const Path & path, double s, double d, double psi, double dt, double v_cap, double verified_s,
    double & v_out, double & delta_out, CtrlInfo & info);
  double delta = 0, v = 0, i_err = 0;

private:
  size_t project(const Path & path, double s, double d);
  const Params & p_;
  size_t idx_ = 0;
  const Path * path_id_ = nullptr;
};

}  // namespace objavoid
