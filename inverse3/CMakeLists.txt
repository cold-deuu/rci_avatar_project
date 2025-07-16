cmake_minimum_required(VERSION 3.8)
project(inverse3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(auto_detect src/examples/auto_detect.cpp)
target_link_libraries(auto_detect ${PROJECT_SOURCE_DIR}/lib/libHaply.HardwareAPI.a)

add_executable(combined src/examples/combined.cpp)
target_link_libraries(combined ${PROJECT_SOURCE_DIR}/lib/libHaply.HardwareAPI.a)

add_executable(hello_wall src/examples/hello_wall.cpp)
target_link_libraries(hello_wall ${PROJECT_SOURCE_DIR}/lib/libHaply.HardwareAPI.a)

add_executable(kinematic_measurement src/examples/kinematic_measurement.cpp)
target_link_libraries(kinematic_measurement ${PROJECT_SOURCE_DIR}/lib/libHaply.HardwareAPI.a)

add_executable(print_handle src/examples/print_handle.cpp)
target_link_libraries(print_handle ${PROJECT_SOURCE_DIR}/lib/libHaply.HardwareAPI.a)

add_executable(print_inverse3 src/examples/print_inverse3.cpp)
target_link_libraries(print_inverse3 ${PROJECT_SOURCE_DIR}/lib/libHaply.HardwareAPI.a)

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

install(
  TARGETS auto_detect combined hello_wall kinematic_measurement print_handle print_inverse3
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/ 
)

ament_package()
