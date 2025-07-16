cmake_minimum_required(VERSION 3.8)
project(rci_h12_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)



# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

add_library(robot_wrapper src/robot/robot_wrapper.cpp)
ament_target_dependencies(robot_wrapper 
  sensor_msgs
  Eigen3
  pinocchio
)
target_link_libraries(robot_wrapper pinocchio::pinocchio)

add_library(simulator src/simulator/simulator.cpp)
ament_target_dependencies(simulator 
  rclcpp 
  sensor_msgs
  std_msgs
  Eigen3
  pinocchio
)
target_link_libraries(simulator pinocchio::pinocchio)

add_library(solver src/solver/ik_solver.cpp)
ament_target_dependencies(solver 
  rclcpp 
  sensor_msgs
  std_msgs
  Eigen3
  pinocchio
)
target_link_libraries(solver pinocchio::pinocchio)

add_library(trajectory src/trajectory/se3_traj.cpp)
ament_target_dependencies(trajectory 
  rclcpp 
  sensor_msgs
  std_msgs
  Eigen3
  pinocchio
)
target_link_libraries(trajectory pinocchio::pinocchio)

add_library(controller src/controller/controller.cpp)
ament_target_dependencies(controller 
  rclcpp 
  sensor_msgs
  std_msgs
  Eigen3
  pinocchio
  geometry_msgs
)
target_link_libraries(controller pinocchio::pinocchio robot_wrapper solver trajectory)

add_executable(main src/main.cpp src/simulator/simulator.cpp)
target_link_libraries(main pinocchio::pinocchio robot_wrapper solver trajectory controller)
ament_target_dependencies(main 
  rclcpp 
  sensor_msgs
  Eigen3
  pinocchio
)

install(
  TARGETS main
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/ 
)


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
