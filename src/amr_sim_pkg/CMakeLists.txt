cmake_minimum_required(VERSION 3.5)
project(amr_sim_pkg)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

