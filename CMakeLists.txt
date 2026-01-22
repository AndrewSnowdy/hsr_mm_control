cmake_minimum_required(VERSION 3.8)
project(hsr_mm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Executables ----

# 1. Base Test Node
add_executable(base_test_node src/base_cmd_vel_test.cpp)
target_include_directories(base_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(base_test_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# 2. Final Pose Node (WBC)
add_executable(dls src/dls.cpp)
target_include_directories(dls PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(dls
  rclcpp
  geometry_msgs
  nav_msgs
  trajectory_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(dls
  pinocchio::pinocchio
  Eigen3::Eigen
)


# 3. Final Pose Node (Analytical IK + Waypoints)
add_executable(kinematic_setpoint_node src/kinematic_setpoint_node.cpp)
target_include_directories(kinematic_setpoint_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(kinematic_setpoint_node
  rclcpp
  geometry_msgs
  nav_msgs
  trajectory_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(kinematic_setpoint_node
  pinocchio::pinocchio
  Eigen3::Eigen
)

# ---- Install ----

install(TARGETS
  base_test_node
  dls
  kinematic_setpoint_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# FIXED: Combined launch and worlds install
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()