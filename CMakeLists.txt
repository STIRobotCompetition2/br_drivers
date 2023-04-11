cmake_minimum_required(VERSION 3.8)
project(br_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


find_package(OpenCV REQUIRED)
# find_package(libpigpio REQUIRED)
# find_package(LIBCAMERA REQUIRED libcamera)

find_package(std_srvs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
include_directories(${LIBCAMERA_INCLUDE_DIRS})

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

add_executable(raspberry_camera_v2_driver src/raspberry_camera_v2_driver.cpp)
add_executable(mpu9150_driver src/mpu9150_driver.cpp)
add_executable(motors_driver src/motors_driver.cpp)

target_link_libraries (mpu9150_driver Eigen3::Eigen)


ament_target_dependencies(
    raspberry_camera_v2_driver
    sensor_msgs
    OpenCV
    rclcpp
    cv_bridge
    std_msgs
)

ament_target_dependencies(
    mpu9150_driver
    sensor_msgs
    rclcpp
    std_srvs
    
)

ament_target_dependencies(
    motors_driver
    rclcpp
    geometry_msgs    
)

target_link_libraries(motors_driver pigpiod_if2 rt)
target_link_libraries(raspberry_camera_v2_driver lccv)


install(TARGETS
  raspberry_camera_v2_driver
  DESTINATION lib/${PROJECT_NAME}/
)

install(TARGETS
  mpu9150_driver
  DESTINATION lib/${PROJECT_NAME}/
)

install(TARGETS
  motors_driver
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)











