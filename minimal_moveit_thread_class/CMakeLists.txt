cmake_minimum_required(VERSION 3.8)
project(minimal_moveit_thread_class)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# DEPENDENCIES

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# CPP
# Add the action client executable
ament_auto_add_executable(moveit_node src/moveit_node.cpp)
# Install the executable
install(TARGETS moveit_node
  DESTINATION lib/${PROJECT_NAME}
)

# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)