cmake_minimum_required(VERSION 3.8)
project(nav2_mppi)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
