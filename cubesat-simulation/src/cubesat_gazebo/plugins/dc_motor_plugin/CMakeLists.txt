cmake_minimum_required(VERSION 3.5)
project(dc_motor_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./build) 
# Build our plugin
add_library(dc_motor_plugin SHARED src/dc_motor_plugin.cc)
target_link_libraries(dc_motor_plugin ${GAZEBO_LIBRARIES})

add_library(dc_motor_model SHARED src/dc_motor_model.cc)
target_link_libraries(dc_motor_model ${GAZEBO_LIBRARIES})

ament_target_dependencies(dc_motor_plugin rclcpp gazebo_ros std_msgs)

ament_package()
