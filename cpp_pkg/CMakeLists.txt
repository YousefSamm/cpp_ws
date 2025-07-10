cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_ign_gazebo REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(
  include
  ${ament_include_dirs}
  ${custom_interfaces_INCLUDE_DIRS}
)

# create executables 

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs geometry_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs geometry_msgs)

add_executable(rpm_pub src/rpm_pub.cpp)
ament_target_dependencies(rpm_pub rclcpp std_msgs)

add_executable(speed_calc src/speed_calc.cpp)
ament_target_dependencies(speed_calc rclcpp std_msgs)

add_executable(odd_even_server src/service_server.cpp)
ament_target_dependencies(odd_even_server rclcpp custom_interfaces)
target_link_libraries(odd_even_server fmt::fmt)

add_executable(odd_even_client src/service_client.cpp)
ament_target_dependencies(odd_even_client rclcpp custom_interfaces)

add_executable(camera_turn_server src/turn_camera_service_server.cpp)
ament_target_dependencies(camera_turn_server rclcpp custom_interfaces OpenCV cv_bridge)

add_executable(camera_turn_client src/turn_camera_service_client.cpp)
ament_target_dependencies(camera_turn_client rclcpp custom_interfaces OpenCV cv_bridge)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action custom_interfaces geometry_msgs)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action custom_interfaces geometry_msgs)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp tf2_ros)

add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener geometry_msgs rclcpp tf2_ros)
# config for ROS2 cli tools  
install(TARGETS
  action_client
  action_server
  camera_turn_client
  camera_turn_server
  odd_even_client
  odd_even_server
  speed_calc
  rpm_pub
  publisher
  subscriber
  tf_broadcaster
  tf_listener
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/ )
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/ )
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/ )

  ament_python_install_package(${PROJECT_NAME})
  install(PROGRAMS
    scripts/turn_camera_service_client.py
    scripts/speed_sub.py
    DESTINATION lib/${PROJECT_NAME}
  )

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