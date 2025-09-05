cmake_minimum_required(VERSION 3.8)
project(huskypanda_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(husky_panda_description REQUIRED)


# Build an executable named moveit_process from src/moveit_process.cpp
add_executable(moveit_process src/moveit_process.cpp)
target_compile_features(moveit_process PUBLIC cxx_std_17)
ament_target_dependencies(moveit_process rclcpp moveit_ros_planning_interface)

# (Optional) if you have headers in include/moveit_package, uncomment:
# target_include_directories(moveit_process PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# install(DIRECTORY include/ DESTINATION include)

# Install the binary so ros2 run/launch can find it
install(TARGETS moveit_process
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/panda_arm_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (do NOT install src/)
install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
