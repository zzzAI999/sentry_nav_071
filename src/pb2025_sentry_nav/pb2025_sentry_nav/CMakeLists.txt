cmake_minimum_required(VERSION 3.8)
project(pb2025_sentry_nav)

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS
  scripts/rm071_explore_map_ready_starter.py
  scripts/rm071_mcu_serial_receiver.py
  scripts/rm071_map_recorder.py
  scripts/rm071_two_point_route.py
  scripts/rm071_pgm_initial_pose.py
  scripts/rm071_valid_map_filter.py
  scripts/rm071_valid_scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
