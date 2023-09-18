cmake_minimum_required(VERSION 3.8)
project(dh_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AnglesUpdate.srv"
)

add_executable(encoder src/encoder.cpp)
ament_target_dependencies(encoder rclcpp std_msgs sensor_msgs)

add_executable(pose_publisher src/pose_publisher.cpp)
ament_target_dependencies(pose_publisher rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs Eigen3 geometry_msgs sensor_msgs)

add_executable(service_client_lifecycle src/service_client_lifecycle.cpp)
ament_target_dependencies(service_client_lifecycle lifecycle_msgs rclcpp_lifecycle std_msgs)

install(TARGETS
  encoder
  pose_publisher
  service_client_lifecycle
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

target_link_libraries(encoder "${cpp_typesupport_target}")

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
