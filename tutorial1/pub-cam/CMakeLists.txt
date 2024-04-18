cmake_minimum_required(VERSION 3.8)
project(pub_cam)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options( -Wall -Wextra -Wpedantic)
endif()

#SET(CMAKE_SYSTEM_NAME Linux)
#SET(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
#SET(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-gcc)


#############################################################################
############################################
# Find the path to the pigpio includes.
find_path(pigpio_INCLUDE_DIR
        NAMES pigpio.h pigpiod_if.h pigpiod_if2.h
        HINTS ${rootfs_dir}/usr/local/include)

# Find the pigpio libraries.
find_library(pigpio_LIBRARY
        NAMES libpigpio.so
        HINTS ${rootfs_dir}/usr/local/lib)
find_library(pigpiod_if_LIBRARY
        NAMES libpigpiod_if.so
        HINTS ${rootfs_dir}/usr/local/lib)
find_library(pigpiod_if2_LIBRARY
        NAMES libpigpiod_if2.so
        HINTS ${rootfs_dir}/usr/local/lib)
########################################################################
#find_library(cv_bridge_LIBRARY
#        NAMES libcv_bridge.so
 #       HINTS ${rootfs_dir}/opt/ros/humble/lib)
########################################################################

# Set the pigpio variables to plural form to make them accessible for
# the paramount cmake modules.
set(pigpio_INCLUDE_DIRS ${pigpio_INCLUDE_DIR})
set(pigpio_INCLUDES     ${pigpio_INCLUDE_DIR})

# Handle REQUIRED, QUIET, and version arguments
# and set the <packagename>_FOUND variable.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pigpio
        DEFAULT_MSG
        pigpio_INCLUDE_DIR pigpio_LIBRARY pigpiod_if_LIBRARY pigpiod_if2_LIBRARY)
################# END PIGPIO INCLUSION ##################################################

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

############## Find OpenCV
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)


add_executable(camera src/camera.cpp)
target_link_libraries(camera PRIVATE pigpiod_if2 )
ament_target_dependencies(camera PUBLIC rclcpp std_msgs sensor_msgs  rmw_implementation OpenCV
        cv_bridge  image_transport pigpio)


add_executable(cam2image src/cam2image.cpp src/options.cpp)
#target_link_libraries(camera PRIVATE pigpiod_if2 )
ament_target_dependencies(cam2image PUBLIC rclcpp std_msgs sensor_msgs  OpenCV
        cv_bridge image_transport)

# config for ROS2 cli tools
install(TARGETS
        camera cam2image
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
