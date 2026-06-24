cmake_minimum_required(VERSION 3.8)
project(smabo_description)

find_package(ament_cmake REQUIRED)

# Install model / launch assets so other packages (nav2, moveit, bringup) and
# `xacro $(find smabo_description)/urdf/...` can locate them.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
