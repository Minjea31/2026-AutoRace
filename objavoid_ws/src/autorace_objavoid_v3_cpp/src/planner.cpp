#include "autorace_objavoid_v3_cpp/planner.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace objavoid {

std::vector<Obs> obstacle_array(const std::vector<Box> & boxes)
{
  std::vector<Obs> out;
  out.reserve(boxes.size());
  for (const Box & b : boxes) {
    out.push_back({(b.s0 + b.s1) / 2, (b.d0 + b.d1) / 2, (b.s1 - b.s0) / 2, (b.d1 - b.d0) / 2});
  }
  return out;
}

double body_clearance(double s, double d, double psi, const std::vector<Obs> & obs, const Params & p)
{
  if (obs.empty()) {return kInf;}
  const double c = std::cos(psi), sn = std::sin(psi);
  const double lh = 0.5 * (p.body_front + p.body_rear);
  const double off = 0.5 * (p.body_front - p.body_rear);
  const double wh = 0.5 * p.body_width;
  const double bx = s + off * c, by = d + off * sn;
  const double ac = std::abs(c), asn = std::abs(sn);
  double best = kInf;
  for (const Obs & o : obs) {
    const double dx = o.cs - bx, dy = o.cd - by;
    const double sep = std::max({
      std::abs(dx) - (o.hs + lh * ac + wh * asn),
      std::abs(dy) - (o.hd + lh * asn + wh * ac),
      std::abs(dx * c + dy * sn) - (lh + o.hs * ac + o.hd * asn),
      std::abs(-dx * sn + dy * c) - (wh + o.hs * asn + o.hd * ac)});
    best = std::min(best, sep);
  }
  return best;
}

double boundary_clearance(double s, double d, double psi, const Boundary & b, const Params & p)
{
  const double c = std::cos(psi), sn = std::sin(psi);
  const double wh = 0.5 * p.body_width;
  double out = kInf;
  for (double ex : {p.body_front, -p.body_rear}) {
    for (double ey : {wh, -wh}) {
      const double cs = s + ex * c - ey * sn;
      const double cd = d + ex * sn + ey * c;
      double r, l;
      b.limits(cs, r, l);
      out = std::min(out, std::min(cd - r, l - cd));
    }
  }
  return out;
}

// d(s) 5차: 시작 (d0, d0', d0''), 끝 (d1, 0, 0). tau 표본마다 (d, d', d'')
static void quintic(double d0, double v0, double a0, double d1, double T, const std::vector<double> & tau,
  double * d, double * dd, double * ddd, double final_slope=0.0)
{
  const double A = d1 - d0 - v0 * T - 0.5 * a0 * T * T;
  const double B = (final_slope-v0) * T - a0 * T * T;
  const double C = -a0 * T * T;
  const double c3 = 10 * A - 4 * B + 0.5 * C;
  const double c4 = -15 * A + 7 * B - C;
  const double c5 = 6 * A - 3 * B + 0.5 * C;
  for (size_t i = 0; i < tau.size(); ++i) {
    const double t = tau[i], t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    d[i] = d0 + v0 * T * t + 0.5 * a0 * T * T * t2 + c3 * t3 + c4 * t4 + c5 * t5;
    const double dt = v0 * T + a0 * T * T * t + 3 * c3 * t2 + 4 * c4 * t3 + 5 * c5 * t4;
    const double ddt = a0 * T * T + 6 * c3 * t + 12 * c4 * t2 + 20 * c5 * t3;
    dd[i] = dt / T;
    ddd[i] = ddt / (T * T);
  }
}

double Planner::plan_limit(const std::vector<Box> & boxes) const
{
  const Params & p = p_;
  double lim = p.section_end + p.plan_beyond_end - p.body_front;
  for (const Box & b : boxes) {
    if (b.s0 < p.section_end) {lim = std::max(lim, b.s1 + p.body_rear + p.pass_margin + p.stop_buffer + 0.1);}
  }
  if(awaiting_island) lim=p.section_end+0.10;
  return std::min(lim, boundary_->s_max - p.body_front - 0.05);
}

void Planner::free_intervals(double sk, const std::vector<Obs> & obs, std::vector<std::pair<double, double>> & free,
  std::vector<std::pair<double, double>> & gaps) const
{
  const Params & p = p_;
  free.clear();
  gaps.clear();
  const double wh = 0.5 * p.body_width;
  double rmax = -kInf, lmin = kInf;
  for (double ss : {sk - p.body_rear, sk, sk + p.body_front}) {
    double r, l;
    boundary_->limits(ss, r, l);
    rmax = std::max(rmax, r);
    lmin = std::min(lmin, l);
  }
  double lo = rmax + wh + p.margin_boundary, hi = lmin - wh - p.margin_boundary;
  if (p.fork_shape && island.pose.valid) {
    const auto poly=island.vertices(); double low=kInf,high=-kInf;
    const double sa=sk-p.body_rear, sb=sk+p.body_front;
    auto add=[&](double d) {low=std::min(low,d); high=std::max(high,d);};
    for(size_t i=0;i<6;++i) {
      const auto a=poly[i],b=poly[(i+1)%6];
      if(a.x>=sa && a.x<=sb) add(a.y);
      for(double edge:{sa,sb}) if(std::abs(b.x-a.x)>1e-9) {
        const double t=(edge-a.x)/(b.x-a.x); if(t>=0 && t<=1) add(a.y+t*(b.y-a.y));
      }
    }
    const double margin=p.margin_obstacle+p.fork_shape_margin;
    if(low<=high && island_side>0) lo=std::max(lo,high+wh+margin);
    if(low<=high && island_side<0) hi=std::min(hi,low-wh-margin);
  }
  if (hi < lo) {return;}
  std::vector<std::pair<double, double>> blocked;
  const double m = p.margin_obstacle;
  for (const Obs & o : obs) {
    if (o.cs + o.hs + m < sk - p.body_rear || o.cs - o.hs - m > sk + p.body_front) {continue;}
    blocked.push_back({o.cd - o.hd - wh - m, o.cd + o.hd + wh + m});
  }
  std::sort(blocked.begin(), blocked.end());
  double cur = lo;
  for (auto & [a, b] : blocked) {
    if (a > cur) {free.push_back({cur, std::min(a, hi)});}
    cur = std::max(cur, b);
    if (cur >= hi) {break;}
  }
  if (cur < hi) {free.push_back({cur, hi});}
  std::vector<std::pair<double, double>> f2;
  for (auto & iv : free) {
    if (iv.second - iv.first >= 0.0) {f2.push_back(iv);}
  }
  free.swap(f2);
  for (auto & [a, b] : free) {gaps.push_back({a - wh - m, b + wh + m});}
}

std::vector<double> Planner::nodes_for(double sk, const std::vector<std::pair<double, double>> & intervals,
  const Path * prev, double keep) const
{
  const Params & p = p_;
  if(fork_target && std::abs(sk-fork_target->x)<1e-5) {
    for(const auto & iv:intervals)
      if(iv.first<=fork_target->y && fork_target->y<=iv.second) return {fork_target->y};
    return {};
  }
  std::vector<std::pair<double, int>> cand;
  for (auto & [a, b] : intervals) {
    std::vector<std::pair<double, int>> pts{{0.5 * (a + b), 0}};
    if (a <= keep && keep <= b) {pts.push_back({keep, -1});}
    if (b - a > 0.04) {pts.push_back({a + 0.01, 2}); pts.push_back({b - 0.01, 2});}
    const double comfort = p.clearance_soft - p.margin_obstacle;
    if (b - a > 2 * comfort) {pts.push_back({a + comfort, 0}); pts.push_back({b - comfort, 0});}
    for (double lc : p.lane_centers) {
      if (a <= lc && lc <= b) {pts.push_back({lc, 1});}
    }
    const double home=desired_d(sk);
    if (a <= home && home <= b) {pts.push_back({home, 1});}
    if (prev != nullptr) {
      const double dp = prev->d_at(sk);
      if (std::isfinite(dp) && a <= dp && dp <= b) {pts.push_back({dp, -1});}
    }
    cand.insert(cand.end(), pts.begin(), pts.end());
  }
  std::stable_sort(cand.begin(), cand.end(), [](auto & x, auto & y) {return x.second < y.second;});
  std::vector<double> out;
  for (auto & [d, pri] : cand) {
    bool far = true;
    for (double o : out) {
      if (std::abs(d - o) <= p.node_merge) {far = false; break;}
    }
    if (far) {out.push_back(d);}
  }
  if (static_cast<int>(out.size()) > p.max_nodes_per_layer) {out.resize(p.max_nodes_per_layer);}
  return out;
}

std::vector<double> Planner::stations(double s0, const std::vector<Obs> & obs, double & s_end) const
{
  const Params & p = p_;
  const double horizon=fork_target?std::max(p.horizon,fork_target->x-s0+0.25):p.horizon;
  s_end = std::min(s0 + horizon, plan_limit(boxes_));
  std::vector<double> cand;
  for (const Obs & o : obs) {cand.push_back(o.cs);}
  for (const Obs & o : obs) {cand.push_back(o.cs - o.hs - p.body_front - 0.02);}
  std::vector<double> st;
  for (double c : cand) {
    if (s0 + p.station_min_ahead < c && c < s_end - 0.1) {st.push_back(c);}
  }
  std::sort(st.begin(), st.end());
  std::vector<std::vector<double>> merged;
  for (double s : st) {
    if (!merged.empty() && s - merged.back().back() < p.station_merge) {
      merged.back().push_back(s);
    } else {
      merged.push_back({s});
    }
  }
  std::vector<double> st2;
  for (auto & g : merged) {st2.push_back(std::accumulate(g.begin(), g.end(), 0.0) / g.size());}
  if (!st2.empty() && s_end - st2.back() < 0.1) {
    st2.back() = s_end;
  } else {
    st2.push_back(s_end);
  }
  std::vector<double> out;
  double prev = s0;
  for (double s : st2) {
    if (s - prev > p.station_max_gap) {
      const int n = static_cast<int>(std::ceil((s - prev) / p.station_max_gap));
      for (int k = 1; k < n; ++k) {out.push_back(prev + (s - prev) * k / n);}
    }
    out.push_back(s);
    prev = s;
  }
  if (static_cast<int>(out.size()) > p.max_layers) {out.resize(p.max_layers);}
  if(fork_target && fork_target->x>s0+p.station_min_ahead && fork_target->x<=out.back()) {
    out.erase(std::remove_if(out.begin(),out.end(),[&](double s){return std::abs(s-fork_target->x)<0.12;}),out.end());
    out.push_back(fork_target->x);std::sort(out.begin(),out.end());
  }
  return out;
}

void Planner::recovery_need(double travel, double ramp, double eo, double eb, double & need_o,
  double & need_b) const
{
  const Params & p = p_;
  const double mo = p.margin_obstacle * ramp, mb = p.margin_boundary * ramp;
  const double zo = eo >= p.margin_obstacle ? p.start_clear_relax : p.start_recover_len;
  const double zb = eb >= p.margin_boundary ? p.start_clear_relax : p.start_recover_len;
  const double uo = std::clamp(travel / zo, 0.0, 1.0);
  const double ub = std::clamp(travel / zb, 0.0, 1.0);
  const double lo_o = std::min(p.margin_obstacle, eo - p.start_clear_tol);
  const double lo_b = std::min(p.margin_boundary, eb - p.start_clear_tol);
  need_o = std::max(p.start_obstacle_floor, std::min(mo, lo_o + (p.margin_obstacle - lo_o) * uo));
  need_b = std::min(mb, lo_b + (p.margin_boundary - lo_b) * ub);
}

double Planner::unknown_cost(const VisibilityGrid * grid, double sk, double node) const
{
  if (grid == nullptr) {return 0.0;}
  const double wh = 0.5 * p_.body_width;
  const bool seen = grid->is_seen(sk, node) && grid->is_seen(sk, node + wh) && grid->is_seen(sk, node - wh);
  return seen ? 0.0 : p_.w_unknown;
}

namespace {
struct Seg
{
  std::vector<double> s, d, d1, d2, psi, k, c, dd, b;
};
struct Node
{
  double s, d, v, a;
};
}  // namespace

PathPtr Planner::plan(const PlanStart & start,const std::vector<Box> & boxes,const VisibilityGrid * grid,
  const Path * prev,double w_cons,double stamp)
{
  const double preferred=std::max(p_.margin_obstacle,p_.preferred_obstacle_clearance);
  if(preferred<=p_.margin_obstacle+1e-9 || (boxes.empty()&&!island.pose.valid))
    return plan_impl(start,boxes,grid,prev,w_cons,stamp);
  Params comfort=p_;const double extra=preferred-comfort.margin_obstacle;
  comfort.margin_obstacle=preferred;comfort.fork_shape_margin=std::max(0.0,comfort.fork_shape_margin-extra);
  Planner trial(comfort,boundary_);trial.island=island;trial.island_side=island_side;
  trial.fork_target=fork_target;trial.fork_target_yaw=fork_target_yaw;trial.awaiting_island=awaiting_island;
  const auto roomy=trial.plan_impl(start,boxes,grid,prev,w_cons,stamp);
  auto select_roomy=[&]() {
    last=trial.last;last_ego_clear=trial.last_ego_clear;has_ego_clear=trial.has_ego_clear;
    boxes_=trial.boxes_;ego_=trial.ego_;return roomy;
  };
  if(roomy && roomy->complete) return select_roomy();
  const auto baseline=plan_impl(start,boxes,grid,prev,w_cons,stamp);
  if(roomy && (!baseline || (!baseline->complete&&roomy->end_s()>=baseline->end_s()-0.01))) return select_roomy();
  last.clearance_fallback=true;
  return baseline;
}

PathPtr Planner::plan_impl(const PlanStart & start, const std::vector<Box> & boxes, const VisibilityGrid * grid,
  const Path * prev, double w_cons, double stamp)
{
  const Params & p = p_;
  if (w_cons < 0) {w_cons = p.w_cons;}
  const double s0 = start.s, d0 = start.d, v0 = start.v, a0 = start.a;
  boxes_ = boxes;
  std::vector<Box> near;
  for (const Box & b : boxes) {
    if (b.s1 > s0 - p.body_rear - 0.3) {near.push_back(b);}
  }
  const std::vector<Obs> obs = obstacle_array(near);
  const double psi0 = std::atan(v0);
  ego_ = {clearance(s0, d0, psi0, obs, p), boundary_clearance(s0, d0, psi0, *boundary_, p)};
  last_ego_clear = ego_;
  has_ego_clear = true;
  double s_end;
  const std::vector<double> sts = stations(s0, obs, s_end);
  last = PlanDiag();last.clearance_margin=p.margin_obstacle;
  last.stations = sts;
  std::vector<std::vector<double>> layer_nodes;
  for (double sk : sts) {
    std::vector<std::pair<double, double>> free, gaps;
    free_intervals(sk, obs, free, gaps);
    last.gaps.push_back({sk, gaps});
    const double keep = d0 + std::max(-0.3, std::min(0.3, v0 * (sk - s0)));
    layer_nodes.push_back(nodes_for(sk, free, prev, keep));
    last.layers.push_back({sk, layer_nodes.back()});
  }

  // 층 0 = 시작 상태. 간선은 최대 max_skip 층을 건너뛸 수 있다.
  const size_t NL = sts.size() + 1;
  std::vector<std::vector<Node>> L(NL);
  L[0] = {{s0, d0, v0, a0}};
  for (size_t j = 1; j < NL; ++j) {
    for (double d : layer_nodes[j - 1]) {L[j].push_back({sts[j - 1], d, (p.fork_shape && island.pose.confirmed && island_side && sts[j-1]>=island.pose.s-1.2) ? (desired_d(sts[j-1]+0.025)-desired_d(sts[j-1]))/0.025 : 0.0, 0.0});}
  }
  std::vector<std::vector<double>> best(NL), seen_cost(NL);
  struct Parent {int i = -1, k = -1; Seg seg;};
  std::vector<std::vector<Parent>> parent(NL);
  best[0] = {0.0};
  parent[0].resize(1);
  for (size_t j = 1; j < NL; ++j) {
    best[j].assign(L[j].size(), kInf);
    parent[j].resize(L[j].size());
    for (const Node & nd : L[j]) {seen_cost[j].push_back(unknown_cost(grid, nd.s, nd.d));}
  }
  static const char * kChecks[] = {"curvature", "steer_rate", "heading", "obstacle", "boundary"};
  for (const char * c : kChecks) {last.rejected[c] = 0;}

  for (size_t j = 1; j < NL; ++j) {
    if (L[j].empty()) {continue;}
    const double sj = L[j][0].s;
    for (size_t i = (j > static_cast<size_t>(p.max_skip) ? j - p.max_skip : 0); i < j; ++i) {
      const double si = L[i].empty() ? 0.0 : L[i][0].s;
      if(fork_target && si<fork_target->x-1e-6 && sj>fork_target->x+1e-6) continue;
      const double T = sj - si;
      const int n = std::max(4, static_cast<int>(std::ceil(T / p.sample_ds))) + 1;
      std::vector<double> tau(n), S(n), w(n, T / (n - 1));
      for (int q = 0; q < n; ++q) {
        tau[q] = static_cast<double>(q) / (n - 1);
        S[q] = si + T * tau[q];
      }
      w[0] = 0.0;
      const double h = T / (n - 1);
      std::vector<double> dprev(n, kNaN);
      if (prev != nullptr && w_cons > 0) {
        for (int q = 0; q < n; ++q) {
          if (S[q] <= s0 + p.cons_length) {dprev[q] = prev->d_at(S[q]);}
        }
      }
      for (size_t src = 0; src < L[i].size(); ++src) {
        if (!std::isfinite(best[i][src])) {continue;}
        const Node & a = L[i][src];
        for (size_t dst = 0; dst < L[j].size(); ++dst) {
          Seg g;
          g.s = S;
          g.d.resize(n); g.d1.resize(n); g.d2.resize(n);
          quintic(a.d, a.v, a.a, L[j][dst].d, T, tau, g.d.data(), g.d1.data(), g.d2.data(), L[j][dst].v);
          g.psi.resize(n); g.k.resize(n); g.c.resize(n); g.dd.resize(n); g.b.resize(n);
          for (int q = 0; q < n; ++q) {
            g.psi[q] = std::atan(g.d1[q]);
            g.k[q] = g.d2[q] / std::pow(1 + g.d1[q] * g.d1[q], 1.5);
          }
          bool ok_o = true, ok_b = true;
          double cost = 0.0;
          for (int q = 0; q < n; ++q) {
            // np.gradient(edge_order=1): 끝은 한쪽 차분, 안쪽은 중앙 차분
            double dk;
            if (q == 0) {dk = (g.k[1] - g.k[0]) / h;} else if (q == n - 1) {dk = (g.k[n - 1] - g.k[n - 2]) / h;} else {
              dk = (g.k[q + 1] - g.k[q - 1]) / (2 * h);
            }
            const double ddelta = p.wheelbase * dk / (1 + std::pow(p.wheelbase * g.k[q], 2));
            g.dd[q] = std::abs(ddelta);
            g.c[q] = clearance(S[q], g.d[q], g.psi[q], obs, p);
            g.b[q] = boundary_clearance(S[q], g.d[q], g.psi[q], *boundary_, p);
            const double travel = S[q] - s0;
            const double ramp = std::clamp(travel / 0.3, 0.2, 1.0);
            double need_o, need_b;
            recovery_need(travel, ramp, ego_.first, ego_.second, need_o, need_b);
            // Kinematics remain soft costs; only collision checks reject candidates.
            const double kq = g.k[q];
            if (!(g.c[q] >= need_o)) {ok_o = false;}
            if (!(g.b[q] >= need_b)) {ok_b = false;}
            double cq = p.w_lane * std::pow(g.d[q] - desired_d(S[q]), 2) + p.w_kappa * kq * kq + p.w_dkappa * dk * dk +
              p.w_clear * std::pow(std::max(0.0, p.clearance_soft - g.c[q]), 2);
            if (std::isfinite(dprev[q])) {cq += w_cons * std::pow(g.d[q] - dprev[q], 2);}
            cost += cq * w[q];
          }
          last.rejected["obstacle"] += !ok_o;
          last.rejected["boundary"] += !ok_b;
          ++last.edges;
          const bool ok = ok_o && ok_b;
          if (!ok) {continue;}
          ++last.feasible;
          if (last.cand.size() < 40) {
            std::vector<Pt2> pl;
            for (int q = 0; q < n; ++q) {pl.push_back({S[q], g.d[q]});}
            last.cand.push_back(pl);
          }
          const double tot = best[i][src] + cost + seen_cost[j][dst];
          if (tot < best[j][dst]) {
            best[j][dst] = tot;
            parent[j][dst].i = static_cast<int>(i);
            parent[j][dst].k = static_cast<int>(src);
            parent[j][dst].seg = std::move(g);
          }
        }
      }
    }
  }
  // 종단: 지평 끝 층에 도달했으면 그 중 최소, 아니면 가장 멀리 간 층(부분 경로 -> 끝에서 정지)
  int jl = -1;
  for (int j = static_cast<int>(NL) - 1; j > 0; --j) {
    for (double v : best[j]) {
      if (std::isfinite(v)) {jl = j; break;}
    }
    if (jl >= 0) {break;}
  }
  if (jl < 0) {return nullptr;}
  const bool complete = jl == static_cast<int>(NL) - 1;
  std::vector<double> term(L[jl].size());
  for (size_t k = 0; k < L[jl].size(); ++k) {
    term[k] = best[jl][k] + p.w_terminal * std::pow(L[jl][k].d - desired_d(L[jl][k].s), 2);
  }
  if (!complete && p.w_partial > 0) {
    for (size_t j = jl + 1; j < NL; ++j) {
      if (L[j].empty()) {continue;}
      for (size_t k = 0; k < L[jl].size(); ++k) {
        double m = kInf;
        for (const Node & nd : L[j]) {m = std::min(m, std::pow(L[jl][k].d - nd.d, 2));}
        term[k] += p.w_partial * m;
      }
      break;
    }
  }
  int k = static_cast<int>(std::min_element(term.begin(), term.end()) - term.begin());
  std::vector<const Seg *> segs;
  std::vector<Pt2> nodes_out;
  int j = jl;
  while (j > 0) {
    const Parent & pr = parent[j][k];
    segs.push_back(&pr.seg);
    nodes_out.push_back({L[j][0].s, L[j][k].d});
    j = pr.i;
    k = pr.k;
  }
  std::reverse(segs.begin(), segs.end());
  std::reverse(nodes_out.begin(), nodes_out.end());
  auto out = std::make_shared<Path>();
  auto cat = [&](std::vector<double> & dst, std::vector<double> Seg::* field) {
      for (size_t q = 0; q < segs.size(); ++q) {
        const auto & v = segs[q]->*field;
        dst.insert(dst.end(), v.begin() + (q == 0 ? 0 : 1), v.end());
      }
    };
  cat(out->s, &Seg::s); cat(out->d, &Seg::d); cat(out->d1, &Seg::d1); cat(out->d2, &Seg::d2);
  cat(out->psi, &Seg::psi); cat(out->kappa, &Seg::k); cat(out->clear, &Seg::c); cat(out->dsteer, &Seg::dd);
  cat(out->bclear, &Seg::b);
  out->cost = *std::min_element(term.begin(), term.end());
  out->complete = complete;
  out->nodes = nodes_out;
  out->stamp = stamp;
  return out;
}

PathPtr Planner::escape(double s0, double d0, double psi0, double delta0, const std::vector<Box> & boxes,
  double stamp)
{
  const Params & p = p_;
  const double L = p.wheelbase;
  const auto [rl, rr] = escape_radius(p);
  const double dl_max = std::atan(L / rl), dr_max = std::atan(L / rr);
  const double rate = p.max_steering_rate / std::max(p.v_max, 0.1) * p.sample_ds;
  const int n = static_cast<int>(p.horizon / p.sample_ds) + 1;
  struct Cand {double tgt, hold, hold_psi;};
  std::vector<Cand> cands;
  for (int a = 0; a < p.escape_steers; ++a) {
    const double tgt = p.escape_steers > 1 ? -dr_max + (dl_max + dr_max) * a / (p.escape_steers - 1) : -dr_max;
    for (double h : p.escape_holds) {cands.push_back({tgt, h, kNaN});}
  }
  for (double tgt : {-dr_max, dl_max}) {
    for (double th : p.escape_hold_psi) {cands.push_back({tgt, kInf, th});}
  }
  const int E = static_cast<int>(cands.size());
  std::vector<Box> nb;
  for (const Box & b : boxes) {
    if (b.s1 > s0 - p.body_rear - 0.3) {nb.push_back(b);}
  }
  const std::vector<Obs> obs = obstacle_array(nb);
  const double eo = clearance(s0, d0, psi0, obs, p);
  const double eb = boundary_clearance(s0, d0, psi0, *boundary_, p);
  const double psi_lim = std::max(1.35, p.max_heading + 0.1);
  struct Traj {std::vector<double> S, D, PSI, K, DD, C, B; int first; double free, score, cmin;};
  std::vector<Traj> tr(E);
  for (int e = 0; e < E; ++e) {
    Traj & t = tr[e];
    t.S.assign(n, 0); t.D.assign(n, 0); t.PSI.assign(n, 0); t.K.assign(n, 0); t.DD.assign(n, 0);
    t.S[0] = s0; t.D[0] = d0; t.PSI[0] = psi0;
    double dl = std::clamp(delta0, -dr_max, dl_max);
    bool turning = true;
    for (int i = 1; i < n; ++i) {
      const double travelled = (i - 1) * p.sample_ds;
      const bool psi_done = !std::isnan(cands[e].hold_psi) && std::abs(t.PSI[i - 1] - psi0) >= cands[e].hold_psi;
      turning = turning && (travelled < cands[e].hold) && !psi_done;
      const double want = turning ? cands[e].tgt :
        std::clamp(-p.escape_return_gain * t.PSI[i - 1], -dr_max, dl_max);
      const double step = std::clamp(want - dl, -rate, rate);
      t.DD[i] = std::abs(step) / p.sample_ds;
      dl += step;
      const double k = std::tan(dl) / L;
      const double ps = t.PSI[i - 1] + k * p.sample_ds;
      const double mid = 0.5 * (t.PSI[i - 1] + ps);
      t.S[i] = t.S[i - 1] + std::cos(mid) * p.sample_ds;
      t.D[i] = t.D[i - 1] + std::sin(mid) * p.sample_ds;
      t.PSI[i] = ps;
      t.K[i] = k;
    }
    t.K[0] = std::tan(std::clamp(delta0, -dr_max, dl_max)) / L;
    t.C.resize(n); t.B.resize(n);
    t.first = n;
    for (int i = 0; i < n; ++i) {
      t.C[i] = clearance(t.S[i], t.D[i], t.PSI[i], obs, p);
      t.B[i] = boundary_clearance(t.S[i], t.D[i], t.PSI[i], *boundary_, p);
      const double travel = i * p.sample_ds;
      double need_o, need_b;
      recovery_need(travel, std::clamp(travel / 0.3, 0.2, 1.0), eo, eb, need_o, need_b);
      const double ds = i == 0 ? 1.0 : t.S[i] - t.S[i - 1];
      const bool ok = t.C[i] >= need_o && t.B[i] >= need_b && std::abs(t.PSI[i]) < psi_lim && ds > 0 &&
        (!awaiting_island || t.S[i]<=p.section_end+0.10);
      if (!ok) {t.first = i; break;}
    }
    for (int i = t.first; i < n; ++i) {  // 막힌 뒤 표본도 시각화·출력 일관성 위해 계산
      t.C[i] = clearance(t.S[i], t.D[i], t.PSI[i], obs, p);
      t.B[i] = boundary_clearance(t.S[i], t.D[i], t.PSI[i], *boundary_, p);
    }
    const int last_ok = std::max(t.first - 1, 0);
    t.free = last_ok * p.sample_ds;
    t.cmin = kInf;
    for (int i = 0; i < std::max(t.first, 1); ++i) {t.cmin = std::min(t.cmin, t.C[i]);}
    t.score = t.free + p.escape_w_clear * std::min(t.cmin, p.clearance_soft) -
      p.escape_w_heading * std::abs(t.PSI[last_ok]) - 0.05 * std::abs(t.D[last_ok] - p.home_d);
  }
  int e = 0;
  for (int q = 1; q < E; ++q) {
    if (tr[q].score > tr[e].score) {e = q;}
  }
  const Traj & t = tr[e];
  const int m = std::max(t.first - 1, 0);
  last.has_escape = true;
  last.esc_free = std::round(t.free * 100) / 100;
  last.esc_steer = cands[e].tgt;
  last.esc_hold = cands[e].hold;
  last.esc_n = E;
  if (t.free < p.escape_min_len) {return nullptr;}
  auto out = std::make_shared<Path>();
  for (int i = 0; i <= m; ++i) {
    const double d1 = std::tan(t.PSI[i]);
    out->s.push_back(t.S[i]);
    out->d.push_back(t.D[i]);
    out->d1.push_back(d1);
    out->d2.push_back(t.K[i] * std::pow(1 + d1 * d1, 1.5));
    out->psi.push_back(t.PSI[i]);
    out->kappa.push_back(t.K[i]);
    out->clear.push_back(t.C[i]);
    out->dsteer.push_back(t.DD[i] * L);
    out->bclear.push_back(t.B[i]);
  }
  out->cost = -t.score;
  out->complete = t.first >= n;
  out->nodes = {{t.S[m], t.D[m]}};
  out->stamp = stamp;
  // 시각화: 상위 조향 후보 12개
  std::vector<int> order(E);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int x, int y) {return tr[x].score > tr[y].score;});
  for (int q = 0; q < std::min(12, E); ++q) {
    const Traj & u = tr[order[q]];
    std::vector<Pt2> pl;
    for (int i = 0; i < std::max(u.first, 2) && i < n; ++i) {pl.push_back({u.S[i], u.D[i]});}
    last.cand.push_back(pl);
  }
  return out;
}

std::pair<double, double> Planner::path_clearance(const Path & path, double s_from,
  const std::vector<Box> & boxes) const
{
  const Params & p = p_;
  const auto it = std::lower_bound(path.s.begin(), path.s.end(), s_from);
  const long i = std::max(0L, static_cast<long>(it - path.s.begin()) - 1);
  if (i >= static_cast<long>(path.s.size()) - 1) {return {-kInf, -kInf};}
  const std::vector<Obs> obs = obstacle_array(boxes);
  double wc = -kInf, wb = -kInf;
  for (size_t q = i; q < path.s.size(); ++q) {
    const double c = clearance(path.s[q], path.d[q], path.psi[q], obs, p);
    const double b = boundary_clearance(path.s[q], path.d[q], path.psi[q], *boundary_, p);
    const double ref_c = path.clear.empty() ? p.margin_obstacle : std::min(p.margin_obstacle, path.clear[q]);
    const double ref_b = path.bclear.empty() ? p.margin_boundary : std::min(p.margin_boundary, path.bclear[q]);
    wc = std::max(wc, ref_c - c);
    wb = std::max(wb, ref_b - b);
  }
  return {wc, wb};
}

double Planner::verified_until(const Path & path, double s_from, const VisibilityGrid & grid,
  const Boundary * lane) const
{
  const auto it = std::lower_bound(path.s.begin(), path.s.end(), s_from);
  const size_t i = static_cast<size_t>(std::max(0L, static_cast<long>(it - path.s.begin()) - 1));
  const double wh = 0.5 * p_.body_width;
  for (size_t q = i; q < path.s.size(); ++q) {
    const double s = path.s[q], d = path.d[q];
    const bool near = s <= s_from + 0.25;
    bool ok = near || (grid.is_seen(s, d) && grid.is_seen(s, d + wh) && grid.is_seen(s, d - wh));
    if (lane != nullptr && !near) {
      double r, l;
      lane->limits(s, r, l);
      const double band = p_.lane_unknown_band;
      const bool need_l = (l - (d + wh)) < band;
      const bool need_r = ((d - wh) - r) < band;
      ok = ok && (!need_l || lane->known(s, 1)) && (!need_r || lane->known(s, -1));
    }
    if (!ok) {return s;}
  }
  return path.s.back();
}

double Planner::desired_d(double s) const {
  if(fork_target) {
    const double target=fork_target->y+std::tan(fork_target_yaw)*(s-fork_target->x);
    const double tip=fork_target->x-island.width/2-0.5;
    const double u=std::clamp((s-(tip-1.2))/1.2,0.0,1.0);
    return p_.home_d+u*u*u*(10+u*(-15+6*u))*(target-p_.home_d);
  }
  if(p_.fork_shape && island.pose.confirmed && island_side) {
    double right,left;boundary_->limits(s,right,left);
    const double center=island.pose.d+std::tan(island.pose.yaw)*(s-island.pose.s);
    const double half=island.width/(2*std::cos(island.pose.yaw));
    const double target=island_side>0 ? (left+center+half)/2 : (right+center-half)/2;
    // Begin the lattice's lateral transition before the island tip. Smoothstep
    // has zero first/second derivatives at both ends of the transition.
    const double u=std::clamp((s-(island.pose.s-1.2))/1.2,0.0,1.0);
    const double blend=u*u*u*(10+u*(-15+6*u));
    return p_.home_d+blend*(target-p_.home_d);
  }
  return p_.home_d;
}

double Planner::clearance(double s,double d,double psi,const std::vector<Obs> & obs,const Params & p) const {
  double c=body_clearance(s,d,psi,obs,p);
  if(p.fork_shape && island.pose.valid) {
    c=std::min(c,island.clearance(s,d,psi,p)-p.fork_shape_margin);
    const auto q=island.local(s,d);
    if(island_side && q.x>=0 && q.x<=island.length && island_side*q.y<0) c=std::min(c,-0.01);
  }
  return c;
}
PathPtr Planner::observe(double s,double d,double psi,double delta,double ts,double td,
  const std::vector<Box> & boxes,const VisibilityGrid & grid,double stamp) {
  const auto obs=obstacle_array(boxes); const auto radii=plan_radius(p_);
  double best=kInf; std::shared_ptr<Path> result;
  const double initial=std::abs(wrap(std::atan2(td-d,ts-s)-psi));
  for(int k=0;k<9;++k) {
    const double want=-std::atan(p_.wheelbase/radii.second)+k/8.0*
      (std::atan(p_.wheelbase/radii.first)+std::atan(p_.wheelbase/radii.second));
    auto path=std::make_shared<Path>(); double x=s,y=d,a=psi,steer=delta; bool ok=true;
    // Include stopping reserve beyond the commanded 0.4m segment.
    const double reserve=p_.fork_search_speed*p_.fork_search_speed/(2*0.8)+p_.fork_search_speed*0.3+0.12;
    for(double travel=0;travel<=p_.fork_search_step+reserve;travel+=0.02) {
      if(std::abs(a)>1.3) {ok=false;break;}
      const double c=clearance(x,y,a,obs,p_), b=boundary_clearance(x,y,a,*boundary_,p_);
      if(c<p_.margin_obstacle || b<p_.margin_boundary) {ok=false;break;}
      const double ca=std::cos(a),sn=std::sin(a);
      for(double ex:{-p_.body_rear,0.0,p_.body_front}) for(double ey:{-p_.body_width/2,p_.body_width/2})
        if(!grid.is_seen(x+ex*ca-ey*sn,y+ex*sn+ey*ca)) ok=false;
      if(!ok) break;
      if(travel<=p_.fork_search_step+1e-6) {
        const double curv=std::tan(steer)/p_.wheelbase, v=std::tan(a);
        path->s.push_back(x);path->d.push_back(y);path->psi.push_back(a);
        path->d1.push_back(v);path->d2.push_back(curv*std::pow(1+v*v,1.5));
        path->kappa.push_back(curv);path->clear.push_back(c);path->bclear.push_back(b);path->dsteer.push_back(0);
      }
      const double limit=p_.max_steering_rate*0.02/std::max(p_.fork_search_speed,0.05);
      steer+=std::clamp(want-steer,-limit,limit);
      const double next=a+0.02*std::tan(steer)/p_.wheelbase;
      x+=0.02*std::cos((a+next)/2);y+=0.02*std::sin((a+next)/2);a=next;
    }
    if(!ok || path->s.size()<3) continue;
    const size_t n=path->s.size()-1;
    const double angle=std::abs(wrap(std::atan2(td-path->d[n],ts-path->s[n])-path->psi[n]));
    // If a turn is needed, require actual improvement, not a straight approach phase.
    if(initial>0.12 && angle>=initial-0.02) continue;
    const double score=angle+0.08*std::abs(want-delta);
    if(score<best) {best=score;result=path;}
  }
  if(result) {result->complete=false;result->stamp=stamp;result->cost=best;}
  return result;
}

}  // namespace objavoid
