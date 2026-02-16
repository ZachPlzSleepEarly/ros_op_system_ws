cmake_minimum_required(VERSION 3.8)
project(imu_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(imu_data src/imu_data.cpp)
ament_target_dependencies(imu_data "rclcpp" "sensor_msgs" "tf2")

add_executable(imu_data_class src/imu_data_class.cpp)
ament_target_dependencies(imu_data_class "rclcpp" "sensor_msgs" "tf2")

add_executable(imu_heading_lock src/imu_heading_lock.cpp)
ament_target_dependencies(imu_heading_lock "rclcpp" "sensor_msgs" "geometry_msgs" "tf2")

install(
  TARGETS
    imu_data
    imu_data_class
    imu_heading_lock
  DESTINATION
    lib/${PROJECT_NAME}
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
