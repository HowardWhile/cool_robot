cmake_minimum_required(VERSION 3.8)
project(cool_robot_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------------------------------------
# node 
# ---------------------------------------
add_executable(xbox_joystick src/xbox_joystick.cpp)
ament_target_dependencies(xbox_joystick rclcpp)
ament_target_dependencies(xbox_joystick sensor_msgs)

# ---------------------------------------
# include 
# ---------------------------------------
target_include_directories(xbox_joystick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# ---------------------------------------
# link lib
# ---------------------------------------

# ---------------------------------------
# Require C99 and C++17
target_compile_features(xbox_joystick PUBLIC c_std_99 cxx_std_17)  


# ---------------------------------------
# install
# ---------------------------------------
# install bin file
install(TARGETS xbox_joystick DESTINATION lib/${PROJECT_NAME})

# install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})  
# ---------------------------------------


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
