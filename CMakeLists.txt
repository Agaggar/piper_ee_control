cmake_minimum_required(VERSION 3.5)
project(ee_velocity_controller)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelativeMove.msg"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ee_velocity_node src/ee_velocity_node.cpp)
add_executable(servo_demo_node src/servo_demo.cpp)
add_executable(velocity_pub src/velocity_pub.cpp)
add_executable(keyboard_relmove src/keyboard_input.cpp)


ament_target_dependencies(ee_velocity_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_servo
  control_msgs
)

ament_target_dependencies(servo_demo_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_servo
  control_msgs
)

ament_target_dependencies(velocity_pub
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_servo
  control_msgs
)

ament_target_dependencies(keyboard_relmove
  rclcpp
  geometry_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface
  config/plugin.xml)

target_link_libraries(ee_velocity_node Eigen3::Eigen)
target_link_libraries(servo_demo_node Eigen3::Eigen)
target_link_libraries(velocity_pub Eigen3::Eigen)
target_link_libraries(keyboard_relmove Eigen3::Eigen)

rosidl_target_interfaces(velocity_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(keyboard_relmove
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install directories
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ee_velocity_node servo_demo_node velocity_pub keyboard_relmove #
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

ament_package()
