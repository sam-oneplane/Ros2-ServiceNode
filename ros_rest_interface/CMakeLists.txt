cmake_minimum_required(VERSION 3.5)
project(ros_rest_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)

# Add JSONCPP
add_subdirectory(third_party/jsoncpp)


# Include directories
include_directories(
  include
  third_party
  third_party/jsoncpp/include
)

# Add executable
add_executable(ros_rest_relay src/ros_rest_relay.cpp)

# Link libraries
ament_target_dependencies(ros_rest_relay rclcpp std_msgs geometry_msgs)
target_link_libraries(ros_rest_relay jsoncpp_lib)

# Install targets
install(TARGETS
  ros_rest_relay
  DESTINATION lib/${PROJECT_NAME})

ament_package()
