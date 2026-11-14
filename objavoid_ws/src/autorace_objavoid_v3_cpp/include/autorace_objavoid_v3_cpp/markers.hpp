// RViz 표시(Python markers.py와 같은 ns·색·크기): 장애물, Gap, 후보, 선택 경로, 차량·목표점·LFD, 도로 경계, 상태.
#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "autorace_objavoid_v3_cpp/mission.hpp"

namespace objavoid {

// core.mutex()를 잡은 상태에서 호출
visualization_msgs::msg::MarkerArray build_markers(const ObjAvoidCore & core,
  const builtin_interfaces::msg::Time & stamp, double t);
// 활성화 전 미리보기: 지금 자세를 원점으로 이번 스캔 검출만
visualization_msgs::msg::MarkerArray preview_markers(const ObjAvoidCore & core, const std::vector<float> & ranges,
  double angle_min, double angle_inc, double x, double y, double yaw, const builtin_interfaces::msg::Time & stamp);
// odom 좌표 마커 -> 차체(base_link) 기준
void to_base(visualization_msgs::msg::MarkerArray & arr, double x0, double y0, double yaw,
  const std::string & frame);

}  // namespace objavoid
