# CMake generated Testfile for 
# Source directory: /home/wego/2026-AutoRace/lane_ws/src/autorace_lane_pid
# Build directory: /home/wego/2026-AutoRace/lane_ws/build/autorace_lane_pid
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_pid "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_pid/test_results/autorace_lane_pid/test_pid.gtest.xml" "--package-name" "autorace_lane_pid" "--output-file" "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_pid/ament_cmake_gtest/test_pid.txt" "--command" "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_pid/test_pid" "--gtest_output=xml:/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_pid/test_results/autorace_lane_pid/test_pid.gtest.xml")
set_tests_properties(test_pid PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_pid/test_pid" TIMEOUT "60" WORKING_DIRECTORY "/home/wego/2026-AutoRace/lane_ws/build/autorace_lane_pid" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/wego/2026-AutoRace/lane_ws/src/autorace_lane_pid/CMakeLists.txt;37;ament_add_gtest;/home/wego/2026-AutoRace/lane_ws/src/autorace_lane_pid/CMakeLists.txt;0;")
subdirs("gtest")
