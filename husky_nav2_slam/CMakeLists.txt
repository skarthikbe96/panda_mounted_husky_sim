cmake_minimum_required(VERSION 3.8)
project(husky_nav2_slam)

find_package(ament_cmake REQUIRED)

# (No binaries; we just install launch/config/etc.)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install LICENSE (since we set a license)
install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
