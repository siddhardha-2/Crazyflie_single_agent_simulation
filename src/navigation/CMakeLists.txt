cmake_minimum_required(VERSION 3.8)
project(navigation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------
# Find dependencies
# -----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# -----------------------------------------------------------------------
# Generate custom messages
# -----------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# -----------------------------------------------------------------------
# state_estimator node
# -----------------------------------------------------------------------
add_executable(state_estimator
  src/state_estimator.cpp
)

target_include_directories(state_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(state_estimator
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Link against generated message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(state_estimator "${cpp_typesupport_target}")

# -----------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------
install(TARGETS state_estimator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()