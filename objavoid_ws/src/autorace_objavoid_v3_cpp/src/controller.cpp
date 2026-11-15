#include "autorace_objavoid_v3_cpp/controller.hpp"

#include <algorithm>
#include <cmath>

namespace objavoid {

void PathTracker::reset()
{
  delta = 0;
  v = 0;
  i_err = 0;
  idx_ = 0;
  path_id_ = nullptr;
}

size_t PathTracker::project(const Path & path, double s, double d)
{
  const size_t n = path.s.size();
  if (path_id_ != &path || idx_ >= n) {  // 새 경로(같은 주소에 새 경로가 올 수도 있어 범위도 확인)
    path_id_ = &path;
    const auto it = std::lower_bound(path.s.begin(), path.s.end(), s);
    idx_ = std::min(static_cast<size_t>(it - path.s.begin()), n - 1);
  }
  const size_t lo = idx_ >= 10 ? idx_ - 10 : 0;
  const size_t hi = std::min(n, idx_ + 30);
  double best = kInf;
  size_t bi = lo;
  for (size_t k = lo; k < hi; ++k) {
    const double dd = (path.s[k] - s) * (path.s[k] - s) + (path.d[k] - d) * (path.d[k] - d);
    if (dd < best) {best = dd; bi = k;}
  }
  idx_ = bi;
  return idx_;
}

TrackErr PathTracker::errors(const Path & path, double s, double d, double psi)
{
  size_t i = project(path, s, d);
  const size_t n = path.s.size();
  size_t j = std::min(i + 1, n - 1);
  if (j == i) {
    i = i > 0 ? i - 1 : 0;
    j = std::min(i + 1, n - 1);
  }
  const double ps = path.s[i], pd = path.d[i];
  const double ts = path.s[j] - ps, td = path.d[j] - pd;
  const double L2 = ts * ts + td * td;
  const double u = L2 < 1e-12 ? 0.0 : std::clamp(((s - ps) * ts + (d - pd) * td) / L2, 0.0, 1.0);
  TrackErr e;
  e.rs = ps + u * ts;
  e.rd = pd + u * td;
  e.rpsi = path.psi[i] + u * (path.psi[j] - path.psi[i]);
  e.ey = -(s - e.rs) * std::sin(e.rpsi) + (d - e.rd) * std::cos(e.rpsi);
  e.epsi = wrap(psi - e.rpsi);
  return e;
}

void PathTracker::step(const Path & path, double s, double d, double psi, double dt, double v_cap,
  double verified_s, double & v_out, double & delta_out, CtrlInfo & info)
{
  const Params & p = p_;
  dt = std::clamp(dt, 1e-3, 0.2);
  const TrackErr e = errors(path, s, d, psi);
  const double v_now = std::max(v, 0.05);
  const double s_ff = e.rs + v_now * p.ff_preview_time;
  const double k_ff = interp(s_ff, path.s, path.kappa);
  if (p.k_i > 0) {i_err = std::clamp(i_err + e.ey * dt, -p.i_limit, p.i_limit);}
  double fb = -p.k_y * e.ey - p.k_psi * e.epsi - p.k_i * i_err;
  fb = std::clamp(fb, -p.fb_limit, p.fb_limit);
  double target = std::atan(p.wheelbase * k_ff) + fb;
  if (p.steer_gain != 1.0) {target = std::atan(p.steer_gain * std::tan(target));}
  target = std::clamp(target, -p.max_steering, p.max_steering);
  const double stp = p.max_steering_rate * dt;
  delta += std::clamp(target - delta, -stp, stp);

  // ---- 속도 ----
  const double kmax = 1.0 / p.plan_min_radius;
  double k_ahead = 0.0, need = 0.0;
  bool any = false;
  double c_min = kInf;
  for (size_t q = 0; q < path.s.size(); ++q) {
    const double sq = path.s[q];
    if (sq >= e.rs && sq <= e.rs + 0.5) {
      any = true;
      k_ahead = std::max(k_ahead, std::abs(path.kappa[q]));
      if (!path.dsteer.empty()) {need = std::max(need, path.dsteer[q]);}
    }
    if (sq >= e.rs - 0.2 && sq <= e.rs + 0.6) {c_min = std::min(c_min, path.clear[q]);}
  }
  double vv = p.v_max * std::max(0.5, 1.0 - p.curve_slow * k_ahead / kmax);
  if (!path.dsteer.empty() && any && need > 1e-6) {
    vv = std::min(vv, 0.8 * p.max_steering_rate / need);  // 거리당 조향 변화가 큰 구간 감속
  }
  if (c_min < p.narrow_clearance) {vv = std::min(vv, p.v_narrow);}
  if (std::abs(e.ey) > 0.08 || std::abs(e.epsi) > 0.3) {vv = std::min(vv, p.v_narrow);}
  const bool has_ver = verified_s >= -1e8;
  const double stop_s = has_ver ? std::min(path.end_s(), verified_s) : path.end_s();
  const double room = stop_s - e.rs - p.stop_buffer;
  if (!path.complete || has_ver) {vv = std::min(vv, std::sqrt(2 * p.decel * std::max(0.0, room)));}
  vv = std::min(vv, v_cap);
  if (vv < p.v_min) {vv = (room < 0.05 || v_cap <= 0) ? 0.0 : p.v_min;}
  if (vv > v) {vv = std::min(vv, v + p.accel * dt);} else {vv = std::max(vv, v - p.decel * 2 * dt);}
  v = std::max(0.0, vv);
  info.valid = true;
  info.ref_s = e.rs;
  info.ref_d = e.rd;
  info.ref_psi = e.rpsi;
  info.ey = e.ey;
  info.epsi = e.epsi;
  info.k_ff = k_ff;
  info.c_min = c_min;
  info.room = room;
  info.s_ff = s_ff;
  v_out = v;
  delta_out = delta;
}

}  // namespace objavoid
