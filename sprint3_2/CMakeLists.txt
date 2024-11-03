cmake_minimum_required(VERSION 3.8)
project(sprint3_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create the executable
add_executable(${PROJECT_NAME} sprint3_2.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

# Link ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  sensor_msgs
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
