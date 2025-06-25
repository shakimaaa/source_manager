cmake_minimum_required(VERSION 3.8)
project(source_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(navx_msgs REQUIRED)


# Include directories
include_directories(
  include
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add the library
add_library(source_managerlib SHARED
  src/source_manager_node.cpp
  src/source.cpp
)

# Link dependenciesh
ament_target_dependencies(source_managerlib
  rclcpp
  nav_msgs
  navx_msgs
  geometry_msgs
  std_msgs
)

# Executable
add_executable(source_manager_node src/source_manager_node.cpp)
target_link_libraries(source_manager_node source_managerlib)

ament_target_dependencies(source_manager_node
  rclcpp
  nav_msgs
  navx_msgs
  geometry_msgs
  std_msgs
)

# Install targets
install(TARGETS source_managerlib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install executable
install(TARGETS source_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Export include directories
ament_export_include_directories(include)
ament_export_libraries(source_managerlib)
ament_export_dependencies(rclcpp nav_msgs )


ament_package() 