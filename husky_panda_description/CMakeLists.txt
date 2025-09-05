cmake_minimum_required(VERSION 3.10)
project(husky_panda_description)

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)
# Install resources

# install(PROGRAMS scripts/js_to_gz_cmd.py
#   DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/husky_panda_bridge.py
  scripts/panda_trajectory_bridge.py
  scripts/twist_to_stamped.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz world/ husky_panda_model/ 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()