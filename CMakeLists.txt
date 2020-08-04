cmake_minimum_required(VERSION 3.5)
project(ros2_thetav)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(libuvc REQUIRED)

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
find_package(PkgConfig REQUIRED)
if ( NOT (PKGCONFIG_FOUND))
      message(FATAL_ERROR "Please Install PPkgConfig: CMake will Exit")
endif()
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.8)
if ( NOT (GST_FOUND))
      message(FATAL_ERROR "Please Install Gstreamer Dev: CMake will Exit")
endif()
set(ENV{PKG_CONFIG_PATH})

include_directories("${GST_INCLUDE_DIRS}")

link_libraries(${GST_LIBRARIES})

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

add_library(thetauvc SHARED
  include/thetauvc.c
)
target_compile_definitions(thetauvc
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)
target_compile_options(thetauvc PUBLIC -Wall)
ament_target_dependencies(thetauvc
  libuvc
)

# add_library(thetauvc include/ros2_thetav/thetauvc.c)
add_executable(thetav_publisher src/thetav_publisher.cpp)
ament_target_dependencies(thetav_publisher
  rclcpp
  std_msgs
  sensor_msgs
  thetauvc
  libuvc
  gst
)

target_link_libraries(thetav_publisher
  uvc
  usb-1.0
  thetauvc
  ${GST_LIBRARIES}
  gstapp-1.0
  gstbase-1.0
  gstreamer-1.0
  gobject-2.0
  glib-2.0
)

target_include_directories(thetav_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  thetav_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
