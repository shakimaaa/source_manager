cmake_minimum_required(VERSION 3.8)
project(source_manager)

# set c++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xion_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)


# include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
)

# add library
add_library(source_managerlib SHARED
  src/source.cpp
  src/gps.cpp
  src/slam.cpp
  src/source_manager.cpp
  src/util.cpp
  src/ekf.cpp
)

target_include_directories(source_managerlib
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIR}  
)

target_link_libraries(source_managerlib
  rclcpp_lifecycle::rclcpp_lifecycle
)

# link dependencies
ament_target_dependencies(source_managerlib
  rclcpp
  nav_msgs
  xion_msg
  geometry_msgs
  std_msgs
  std_srvs
  sensor_msgs
  rclcpp_lifecycle
  lifecycle_msgs
)


# add executable
add_executable(source_manager_node main.cpp)
target_link_libraries(source_manager_node source_managerlib rclcpp_lifecycle::rclcpp_lifecycle)

ament_target_dependencies(source_manager_node
  rclcpp
  nav_msgs
  xion_msg
  geometry_msgs
  std_msgs
  std_srvs
  sensor_msgs
  rclcpp_lifecycle
  lifecycle_msgs
)

# install configuration
install(TARGETS source_managerlib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS source_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# export configuration
ament_export_include_directories(include)
ament_export_libraries(source_managerlib)
ament_export_dependencies(rclcpp nav_msgs)

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
