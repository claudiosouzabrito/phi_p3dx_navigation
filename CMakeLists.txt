cmake_minimum_required(VERSION 3.8)
project(phi_p3dx_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# ---------- C++ executable node ----------
add_executable(control_example_cpp src/control_example.cpp src/navigation_node.cpp)
ament_target_dependencies(control_example_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS control_example_cpp
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(wall_following src/wall_following_PID.cpp src/navigation_node.cpp)
ament_target_dependencies(wall_following
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS wall_following
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(VFH src/VFH.cpp src/navigation_node.cpp)
ament_target_dependencies(VFH
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS VFH
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts
        DESTINATION share/${PROJECT_NAME})

# ---------- Python Node script ----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/control_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_example_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
