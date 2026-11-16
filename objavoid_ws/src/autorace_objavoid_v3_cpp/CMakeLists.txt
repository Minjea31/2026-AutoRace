cmake_minimum_required(VERSION 3.8)
project(autorace_objavoid_v3_cpp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ROS 독립 코어(단위 테스트에서도 사용)
add_library(objavoid_core STATIC
  src/perception.cpp
  src/planner.cpp
  src/outline.cpp
  src/island.cpp
  src/lane.cpp
  src/controller.cpp
  src/mission.cpp
  src/start_gate.cpp
)
target_include_directories(objavoid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(objavoid_core ${OpenCV_LIBS} yaml-cpp)
set_target_properties(objavoid_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(objavoid_v3_node src/node.cpp src/markers.cpp)
target_link_libraries(objavoid_v3_node objavoid_core)
ament_target_dependencies(objavoid_v3_node rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs
  visualization_msgs ackermann_msgs builtin_interfaces rcl_interfaces)

install(TARGETS objavoid_v3_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(entrance_probe ../../tools/entrance_probe.cpp)
  target_link_libraries(entrance_probe objavoid_core)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core objavoid_core)
  # Runtime YAML includes deliberate tuning overrides. Compare actual Python defaults instead.
  set(PARAM_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../autorace_objavoid_v3/autorace_objavoid_v3/params.py")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${PARAM_SOURCE}")
  execute_process(COMMAND /usr/bin/python3 -c
    "import runpy,dataclasses,json; m=runpy.run_path('${PARAM_SOURCE}'); print(json.dumps({'objavoid_v3_node':{'ros__parameters':dataclasses.asdict(m['Params']())}}))"
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/python_defaults.json" COMMAND_ERROR_IS_FATAL ANY)
  target_compile_definitions(test_core PRIVATE
    PY_PARAMS_FILE="${CMAKE_CURRENT_BINARY_DIR}/python_defaults.json"
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_package()
