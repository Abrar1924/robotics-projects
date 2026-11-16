cmake_minimum_required(VERSION 3.10)
project(mower_supervisor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The ROS 2 node.
add_executable(supervisor src/supervisor.cpp src/node.cpp)
target_include_directories(supervisor PUBLIC src)
ament_target_dependencies(supervisor rclcpp sensor_msgs std_msgs)

# The same supervisor with no ROS: replays readings from standard input.
add_executable(replay src/supervisor.cpp src/replay_main.cpp)
target_include_directories(replay PUBLIC src)

install(TARGETS supervisor replay DESTINATION lib/${PROJECT_NAME})
ament_package()
