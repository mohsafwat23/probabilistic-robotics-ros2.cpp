cmake_minimum_required(VERSION 3.5)
project(diff_rob_model)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diff_rob_msgs REQUIRED)


add_executable(enc_sim src/encoder_sim.cpp)
add_executable(land_cam src/landmark_cam.cpp)
add_executable(com_vel src/command_vel.cpp)
add_executable(traj src/traj.cpp)


ament_target_dependencies(enc_sim rclcpp geometry_msgs tf2_msgs)
ament_target_dependencies(land_cam rclcpp std_msgs geometry_msgs cv_bridge OpenCV image_transport sensor_msgs diff_rob_msgs)
ament_target_dependencies(com_vel rclcpp geometry_msgs)
ament_target_dependencies(traj rclcpp nav_msgs geometry_msgs)

install(TARGETS
  enc_sim
  land_cam
  com_vel
  traj
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()

