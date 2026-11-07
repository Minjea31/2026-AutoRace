# CMake generated Testfile for 
# Source directory: /home/wego/2026-AutoRace/lane_ws/src/autorace_lane_lite
# Build directory: /home/wego/2026-AutoRace/lane_ws/build/autorace_lane_lite
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_lane "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_lite/test_results/autorace_lane_lite/test_lane.gtest.xml" "--package-name" "autorace_lane_lite" "--output-file" "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_lite/ament_cmake_gtest/test_lane.txt" "--command" "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_lite/test_lane" "--gtest_output=xml:/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_lite/test_results/autorace_lane_lite/test_lane.gtest.xml")
set_tests_properties(test_lane PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_lite/test_lane" TIMEOUT "60" WORKING_DIRECTORY "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_lite" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/wego/2026-AutoRace/lane_ws/src/autorace_lane_lite/CMakeLists.txt;31;ament_add_gtest;/home/wego/2026-AutoRace/lane_ws/src/autorace_lane_lite/CMakeLists.txt;0;")
add_test(test_runtime "/usr/bin/python3" "/home/wego/2026-AutoRace/lane_ws/src/autorace_lane_lite/test/test_runtime.py")
set_tests_properties(test_runtime PROPERTIES  _BACKTRACE_TRIPLES "/home/wego/2026-AutoRace/lane_ws/src/autorace_lane_lite/CMakeLists.txt;33;add_test;/home/wego/2026-AutoRace/lane_ws/src/autorace_lane_lite/CMakeLists.txt;0;")
subdirs("gtest")
