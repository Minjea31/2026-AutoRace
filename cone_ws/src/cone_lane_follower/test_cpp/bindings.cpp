// Test-only Python bindings: the installed ROS node has no Python dependency.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "cone_lane_follower/core.hpp"
#include "cone_lane_follower/path_memory.hpp"
namespace py = pybind11;
using namespace cone;
PYBIND11_MODULE(cone_native, m){
  m.def("sample_boundary", &sample_boundary);
  m.def("segments_cross", &segments_cross);
  m.def("select_boundary_chain", &select_boundary_chain, py::arg("cones"), py::arg("parameters") =
          BOUNDARY_DEFAULTS, py::arg("opposite") = Path{});
  m.def("select_boundaries", &select_boundaries, py::arg("cones"), py::arg("parameters"), py::arg("width") =
          std::nullopt, py::arg("width_tolerance") = .25);
  m.def("boundary_width_matches", &boundary_width_matches);
  m.def("match_cross_sections", &match_cross_sections);
  m.def("match_projected_sections", [](Path l, Path r, double s, double lo, double hi){
    return match_projected_sections(l, r, s, lo, hi);
  });
  m.def("smooth_center_path", [](Path c, Path l, Path r, double w, double g, double t){
    return smooth_center_path(c, l, r, w, g, t);
  });
  m.def("offset_boundary", &offset_boundary);
  m.def("point_near_boundary", &point_near_boundary);
  m.def("quintic_bridge", &quintic_bridge);
  m.def("offset_safe", [](Path p, Boundaries b){
    return offset_safe(p, b);
  });
  m.def("valid_center_geometry", [](Path p, Params v){
    return valid_center_geometry(p, v);
  });
  m.def("unified_path", [](Path l, Path r, Pairs pairs, Path measured, std::optional<double>w, Params p){
    return unified_path(l, r, pairs, measured, w, p);
  });
  m.def("corner_offset", [](Path b, double d, double s, double r, Params p){
    return corner_offset(b, d, s, r, p);
  });
  m.def("curvature", &curvature);
  m.def("path_control_geometry", [](Path p, double r, double d)->py::object {
    auto v = path_control_geometry(p, r, d);
    if (!v) return py::none();
    return py::make_tuple(v->lateral, v->heading, v->x, v->y, v->a, v->b);
  }, py::arg("path"), py::arg("reference_x"), py::arg("max_distance") = .5);
  m.def("path_errors", [](Path p, double r, double d)->py::object {
    auto v = path_control_geometry(p, r, d);
    if (!v) return py::none();
    return py::make_tuple(v->lateral, v->heading);
  }, py::arg("path"), py::arg("reference_x"), py::arg("max_distance") = .5);
  m.def("path_stop_reason", &path_stop_reason);
  m.def("path_length", &path_length);
  m.def("cluster_points", &cluster_points);
  m.def("filter_clusters", &filter_clusters);
  m.def("cluster_center", &cluster_center);
  py::class_<PathMemory>(m, "PathMemory").def(py::init<>()).def_readwrite("points",
                                                                          &PathMemory::points).def_readwrite(
    "index", &PathMemory::index).def_readwrite("finished",
                                               &
                                               PathMemory::finished).def_readwrite("update_reason",
                                                                                   &PathMemory::update_reason)
  .def("clear",
       &
       PathMemory
       ::
       clear).def("store", &PathMemory::store).def("refresh", [](PathMemory&m, Path p, py::function
                                                                 v)
  {
    return m.refresh(p, [&](const Path&p){
      return v(p).cast<bool>();
    });
  }).def("replace_from_vehicle", &PathMemory::replace_from_vehicle).def("remaining", [](PathMemory&m, Path p,
                                                                                        double g, double d){
    auto[r, reason] = m.remaining(p, g, d);
    return py::make_tuple(r, reason.empty()?py::object(py::none()):py::object(py::str(reason)));
  });
}
