cmake_minimum_required(VERSION 3.10)
project(gim_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# The ROS 2 node.
add_executable(monitor src/node.cpp)
target_include_directories(monitor PUBLIC include)
ament_target_dependencies(monitor rclcpp diagnostic_msgs geometry_msgs sensor_msgs std_msgs std_srvs)

# The same tests with no ROS: replays a recorded flight from standard input.
add_executable(replay src/replay_main.cpp)
target_include_directories(replay PUBLIC include)

install(TARGETS monitor replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
