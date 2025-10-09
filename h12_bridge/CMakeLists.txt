cmake_minimum_required(VERSION 3.8)
project(h12_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(unitree_hg REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

SET(${PROJECT_NAME}_MSG
    sensor_msgs
    geometry_msgs
    std_msgs
    unitree_hg
    
  )

SET(${PROJECT_NAME}_TSID
    pinocchio
    Eigen3
  )


SET(${PROJECT_NAME}_NODE
    src/node/state_publisher.cpp
  )

  
# Library
add_library(${PROJECT_NAME}_LIB SHARED ${${PROJECT_NAME}_NODE})
ament_target_dependencies(${PROJECT_NAME}_LIB 
  rclcpp 
  ${${PROJECT_NAME}_MSG}
  ${${PROJECT_NAME}_TSID}
)
target_link_libraries(${PROJECT_NAME}_LIB pinocchio::pinocchio unitree_sdk2)

# Execute
add_executable(main src/main.cpp src/node/state_publisher.cpp)
target_link_libraries(main ${${PROJECT_NAME}_LIB} unitree_sdk2)
ament_target_dependencies(main 
  rclcpp 
  ${${PROJECT_NAME}_MSG}
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
