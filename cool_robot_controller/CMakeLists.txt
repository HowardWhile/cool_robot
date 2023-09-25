cmake_minimum_required(VERSION 3.8)
project(cool_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add cool_robot_controller library related compile commands
generate_parameter_library(cool_robot_controller_parameters
  src/cool_robot_controller.yaml
)
add_library(
  cool_robot_controller
  SHARED
  src/cool_robot_controller.cpp
)
target_include_directories(cool_robot_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(cool_robot_controller cool_robot_controller_parameters)
ament_target_dependencies(cool_robot_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(cool_robot_controller PRIVATE "COOL_ROBOT_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface cool_robot_controller.xml)

install(
  TARGETS
  cool_robot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  cool_robot_controller
)

ament_package()
