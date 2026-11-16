// 모든 튜닝 파라미터의 기본값(Python autorace_objavoid_v3/params.py와 이름·기본값 동일).
// OBJAVOID_PARAMS X-macro 하나로 구조체 필드와 ROS 파라미터 선언을 같이 만든다.
#pragma once
#include <string>
#include <utility>
#include <vector>

namespace objavoid {

using DVec = std::vector<double>;

// X(type, name, default)  type: D=double, I=int, B=bool, V=vector<double>
#define OBJAVOID_PARAMS(X)                                         \
  /* 차량 */                                                        \
  X(D, wheelbase, 0.200)                                           \
  X(D, body_front, 0.161)                                          \
  X(D, body_rear, 0.161)                                           \
  X(D, body_width, 0.220)                                          \
  X(D, max_steering, 0.40)                                         \
  X(D, max_steering_rate, 1.2)                                     \
  X(D, lidar_x, 0.103)                                             \
  X(D, lidar_y, 0.0)                                               \
  X(D, lidar_yaw, 0.0)                                             \
  /* 사전 도로 경계 */                                              \
  X(V, boundary_s, (DVec{-1.0, 0.0, 0.5, 5.8}))                    \
  X(V, boundary_left, (DVec{0.25, 0.26, 0.768, 0.768}))            \
  X(V, boundary_right, (DVec{-0.25, -0.26, -0.768, -0.768}))       \
  X(D, section_end, 4.25)                                          \
  X(D, roi_beyond_end, 0.6)                                        \
  X(D, plan_beyond_end, 0.1)                                       \
  X(D, home_d, 0.0)                                                \
  X(V, lane_centers, (DVec{-0.52, 0.0, 0.52}))                     \
  /* 카메라 노란 실선 */                                            \
  X(B, use_lane, true)                                             \
  X(D, lane_cell, 0.03)                                            \
  X(I, lane_confirm, 2)                                            \
  X(D, lane_ratio, 0.3)                                            \
  X(D, lane_line_half, 0.015)                                      \
  X(D, lane_center_gap, 0.15)                                      \
  X(D, lane_max_d, 1.2)                                            \
  X(D, lane_unknown_band, 0.15)                                    \
  X(B, lane_align, true)                                           \
  X(D, lane_align_wait, 1.5)                                       \
  X(D, lane_align_period, 0.5)                                     \
  X(D, lane_realign_period, 1.0)                                   \
  X(D, lane_align_s_max, 1.0)                                      \
  X(D, lane_align_max, 0.6)                                        \
  X(D, lane_seg_min_len, 0.4)                                      \
  X(D, lane_seg_max_thick, 0.02)                                   \
  X(D, lane_align_min_len, 0.6)                                    \
  X(D, lane_align_tol, 0.03)                                       \
  X(D, lane_center_tol, 0.04)                                      \
  X(D, lane_reset_tracks_angle, 0.05)                              \
  X(V, lane_pair_widths, (DVec{0.52, 1.537}))                      \
  X(D, lane_width_tol, 0.08)                                       \
  X(D, lane_center_max, 0.35)                                      \
  X(D, lane_single_tol, 0.25)                                      \
  X(D, lane_white_gap, 0.25)                                       \
  X(D, lane_dash_offset, 0.26)                                     \
  X(D, lane_white_exclusion, 0.06)                                 \
  X(B, lane_island, true)                                          \
  X(D, lane_island_d, 0.3)                                         \
  X(D, lane_island_window, 0.8)                                    \
  X(B, lane_lidar_filter, true)                                    \
  X(D, lane_lidar_margin, 0.06)                                    \
  X(I, lane_lidar_beams, 2)                                        \
  /* 인지 */                                                        \
  X(D, scan_range_max, 3.5)                                        \
  X(D, scan_range_min, 0.05)                                       \
  X(D, roi_margin, 0.06)                                           \
  X(D, cluster_jump, 0.12)                                         \
  X(I, cluster_min_points, 3)                                      \
  X(B, obstacle_fixed_size, false)                                \
  X(D, obstacle_min_length, 0.33)                                  \
  X(D, obstacle_min_width, 0.23)                                   \
  X(D, obstacle_max_length, 0.70)                                  \
  X(D, obstacle_max_width, 0.45)                                   \
  X(D, assoc_distance, 0.20)                                       \
  X(I, track_window, 10)                                           \
  X(D, track_lock_tol, 0.04)                                       \
  X(I, confirm_hits, 2)                                            \
  X(D, tentative_timeout, 0.5)                                     \
  X(D, pass_margin, 0.08)                                          \
  X(D, ghost_time, 1.0)                                            \
  X(D, vis_res, 0.05)                                              \
  X(D, vis_range, 3.0)                                             \
  X(D, vis_beam_step_deg, 0.5)                                     \
  /* 계획 */                                                        \
  X(D, horizon, 2.5)                                               \
  X(D, sample_ds, 0.05)                                            \
  X(D, station_merge, 0.15)                                        \
  X(D, station_min_ahead, 0.30)                                    \
  X(D, station_max_gap, 0.90)                                      \
  X(I, max_layers, 8)                                              \
  X(I, max_skip, 3)                                                \
  X(I, max_nodes_per_layer, 10)                                    \
  X(D, node_merge, 0.03)                                           \
  X(D, margin_obstacle, 0.025)                                     \
  X(D, margin_boundary, 0.02)                                      \
  X(D, clearance_soft, 0.12)                                       \
  X(D, plan_min_radius, 0.55)                                      \
  X(D, plan_min_radius_left, 0.0)                                  \
  X(D, plan_min_radius_right, 0.0)                                 \
  X(D, plan_steer_rate, 1.0)                                       \
  X(D, plan_steer_speed, 0.15)                                     \
  X(D, max_heading, 0.9)                                           \
  X(D, start_relax, 0.3)                                           \
  X(D, relax_min_radius, 0.45)                                     \
  X(D, start_clear_relax, 0.5)                                     \
  X(D, start_clear_tol, 0.015)                                     \
  X(D, start_recover_len, 1.0)                                     \
  X(D, start_obstacle_floor, 0.005)                                \
  X(D, lane_measure_tol, 0.03)                                     \
  X(D, start_relax_retry, 0.5)                                     \
  X(I, escape_steers, 9)                                           \
  X(V, escape_holds, (DVec{0.25, 0.5, 0.8}))                       \
  X(V, escape_hold_psi, (DVec{0.6, 0.9, 1.2, 1.4}))                \
  X(D, escape_min_radius, 0.50)                                    \
  X(D, escape_min_radius_left, 0.0)                                \
  X(D, escape_min_radius_right, 0.0)                               \
  X(D, escape_return_gain, 1.5)                                    \
  X(D, escape_w_clear, 2.0)                                        \
  X(D, escape_w_heading, 0.3)                                      \
  X(D, escape_min_len, 0.4)                                        \
  X(D, escape_use_len, 1.0)                                        \
  X(B, observe_mode, false)                                        \
  X(D, observe_min_radius, 0.47)                                   \
  X(D, observe_max_heading, 1.2)                                   \
  X(D, w_lane, 1.0)                                                \
  X(D, w_kappa, 0.05)                                              \
  X(D, w_dkappa, 0.002)                                            \
  X(D, w_clear, 60.0)                                              \
  X(D, w_cons, 8.0)                                                \
  X(D, cons_length, 1.0)                                           \
  X(D, w_unknown, 3.0)                                             \
  X(D, w_terminal, 2.0)                                            \
  X(D, w_partial, 0.0)                                             \
  /* 재계획 */                                                      \
  X(D, replan_remaining, 1.4)                                      \
  X(D, replan_period, 1.0)                                         \
  X(D, stitch_error, 0.05)                                         \
  X(D, commit_len, 0.35)                                           \
  X(D, deviation_replan, 0.12)                                     \
  X(D, blocked_tol, 0.02)                                          \
  X(I, blocked_scans, 2)                                           \
  X(D, deviation_stop, 0.20)                                       \
  /* 제어(PD) */                                                    \
  X(D, control_rate, 25.0)                                         \
  X(D, k_y, 1.3)                                                   \
  X(D, k_psi, 1.0)                                                 \
  X(D, k_i, 0.0)                                                   \
  X(D, i_limit, 0.1)                                               \
  X(D, fb_limit, 0.30)                                             \
  X(D, steer_gain, 1.0)                                            \
  X(B, reverse_enable, false)                                      \
  X(D, reverse_wait, 1.0)                                          \
  X(D, reverse_speed, 0.10)                                        \
  X(D, reverse_dist, 0.20)                                         \
  X(D, reverse_psi, 0.5)                                           \
  X(I, reverse_max, 3)                                             \
  X(D, reverse_timeout, 6.0)                                       \
  X(D, reverse_margin, 0.03)                                       \
  X(D, steer_trim, 0.0)                                            \
  X(D, ff_preview_time, 0.12)                                      \
  X(D, v_max, 0.30)                                                \
  X(D, v_min, 0.08)                                                \
  X(D, v_narrow, 0.20)                                             \
  X(D, narrow_clearance, 0.07)                                     \
  X(D, curve_slow, 0.5)                                            \
  X(D, accel, 0.5)                                                 \
  X(D, decel, 0.8)                                                 \
  X(D, stop_buffer, 0.12)                                          \
  /* 갈림길 A/B */                                                   \
  X(B, fork_enable, true)                                          \
  X(B, fork_turn, false)                                           \
  X(D, fork_turn_after, 1.0)                                       \
  X(D, fork_turn_steer, 0.35)                                      \
  X(D, fork_turn_dist, 0.9)                                        \
  X(D, fork_turn_speed, 0.25)                                      \
  X(D, fork_island_s, 0.0)                                         \
  X(D, fork_sign_to_island, 2.0)                                   \
  X(D, fork_island_len, 1.75)                                      \
  X(D, fork_island_half, 0.26)                                     \
  X(D, fork_plan_beyond, 1.0)                                      \
  X(D, fork_enter_len, 0.3)                                        \
  X(D, fork_wait_dist, 0.7)                                        \
  X(D, fork_sign_timeout, 8.0)                                     \
  X(I, fork_sign_min, 2)                                           \
  X(D, fork_lock_dist, 0.8)                                        \
  X(B, fork_use_sign_post, true)                                   \
  X(D, fork_cam_fx, 487.3)                                         \
  X(D, fork_cam_cx, 321.5)                                         \
  X(D, fork_cam_x, 0.133)                                          \
  X(D, fork_cam_y, -0.05)                                          \
  X(D, fork_bearing_tol, 0.10)                                     \
  X(D, fork_post_max_dist, 3.5)                                    \
  X(D, fork_post_center_d, 0.4)                                    \
  X(D, fork_sign_cache_sec, 30.0) \
  X(D, fork_goal_tolerance, 0.15) \
  X(B, obstacle_reacquire_reset, true) \
  X(D, obstacle_reacquire_absence_sec, 1.0) \
  X(I, obstacle_reacquire_confirm_scans, 2) \
  X(D, preferred_obstacle_clearance, 0.10) \
  X(B, road_outline, false) \
  X(D, road_outline_length, 6.665) \
  X(D, road_outline_straight, 5.529) \
  X(B, fork_shape, false) \
  X(D, fork_shape_length, 1.687) \
  X(D, fork_shape_width, 0.5) \
  X(D, fork_shape_margin, 0.05) \
  X(D, fork_search_speed, 0.2) \
  X(D, fork_search_distance, 1.2) \
  X(D, fork_search_timeout, 6.0) \
  X(D, fork_search_step, 0.4) \
  X(D, fork_road_width, 1.54) \
  X(B, start_entrance, false) \
  X(I, start_entrance_confirm, 3) \
  X(D, start_entrance_window, 5.0) \
  X(I, start_entrance_obstacles, 2) \
  /* 자동 시작 */                                                    \
  X(B, start_legacy, true)                                       \
  X(B, start_auto, true)                                           \
  X(B, start_need_line, false)                                     \
  X(D, start_line_x_min, 0.15)                                     \
  X(D, start_line_x_max, 1.0)                                      \
  X(D, start_line_thick, 0.06)                                     \
  X(D, start_line_after, 0.5)                                      \
  X(D, start_line_span, 0.5)                                       \
  X(D, start_line_fill, 0.7)                                       \
  X(I, start_line_min_pts, 15)                                     \
  X(D, start_dash_dist, 1.5)                                       \
  X(D, start_dash_max_y, 0.45)                                     \
  X(D, start_dash_max_width, 0.08)                                 \
  X(D, start_dash_min_len, 0.05)                                   \
  X(D, start_dash_max_len, 0.40)                                   \
  X(I, start_dash_min, 2)                                          \
  X(B, start_dash_both, false)                                     \
  X(D, start_lane_width, 0.5)                                      \
  X(D, start_lane_width_tol, 0.12)                                 \
  X(I, start_stripe_max, 2)                                        \
  X(D, start_obs_x_max, 3.0)                                       \
  X(D, start_obs_y, 0.25)                                          \
  X(I, start_obs_min, 1)                                           \
  X(D, start_obs_max_size, 0.45)                                   \
  X(D, start_obs_min_width, 0.12)                                  \
  X(B, start_after_crosswalk, false)                               \
  X(D, start_cw_quiet, 1.0)                                        \
  X(D, start_arm_dist, 0.8)                                        \
  X(I, start_confirm, 2)                                           \
  X(D, start_rearm, 20.0)                                          \
  X(B, start_origin_at_line, true)                                 \
  /* 서보 보정 흉내 */                                             \
  X(B, cmd_emulate_calibration, false)                             \
  X(D, cmd_cal_gain, 1.92)                                         \
  X(D, cmd_cal_deadband, 0.23)                                     \
  X(D, cmd_cal_limit, 0.48)                                        \
  X(D, cmd_legacy_scale, 2.47)                                     \
  X(D, cmd_wheelbase, 0.2)                                         \
  X(D, cmd_track, 0.172)                                           \
  /* 안전 */                                                        \
  X(D, scan_timeout, 0.40)                                         \
  X(D, odom_timeout, 0.25)                                         \
  X(D, odom_extrapolate_max, 0.20)                                 \
  X(D, emergency_distance, 0.06)                                   \
  X(D, emergency_time, 0.35)                                       \
  X(I, emergency_points, 3)                                        \
  /* 종료 */                                                        \
  X(I, done_min_passed, 1)                                         \
  X(D, done_center_tol, 0.08)                                      \
  X(D, done_heading_tol, 0.12)                                     \
  X(D, done_hold, 0.3)                                             \
  X(D, done_clear_ahead, 0.8)

#define OBJAVOID_CTYPE_D double
#define OBJAVOID_CTYPE_I int
#define OBJAVOID_CTYPE_B bool
#define OBJAVOID_CTYPE_V DVec

struct Params {
#define OBJAVOID_FIELD(t, n, def) OBJAVOID_CTYPE_##t n = def;
  OBJAVOID_PARAMS(OBJAVOID_FIELD)
#undef OBJAVOID_FIELD
};

// (좌, 우) 최소 반경. kind: plan | escape. 0이면 공통 값.
inline std::pair<double, double> plan_radius(const Params & p)
{
  return {p.plan_min_radius_left > 0 ? p.plan_min_radius_left : p.plan_min_radius,
    p.plan_min_radius_right > 0 ? p.plan_min_radius_right : p.plan_min_radius};
}
inline std::pair<double, double> escape_radius(const Params & p)
{
  return {p.escape_min_radius_left > 0 ? p.escape_min_radius_left : p.escape_min_radius,
    p.escape_min_radius_right > 0 ? p.escape_min_radius_right : p.escape_min_radius};
}

}  // namespace objavoid
