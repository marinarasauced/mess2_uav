cmake_minimum_required(VERSION 3.8)
project(hawk_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)

add_library(_hawk_offboard SHARED src/offboard.cpp)

target_include_directories(
  _hawk_offboard PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

target_compile_definitions(
  _hawk_offboard
  PRIVATE "HAWK_CALIBRATION_SERVER_BUILDING_DLL"
)

ament_target_dependencies(
  _hawk_offboard
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "mavros_msgs"
  "mess2_msgs"
  "mess2_plugins"
)

rclcpp_components_register_node(_hawk_offboard PLUGIN "mess2_nodes::UAVOffboardNode" EXECUTABLE offboard)

install(TARGETS
  _hawk_offboard
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
