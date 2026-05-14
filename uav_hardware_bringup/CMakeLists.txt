cmake_minimum_required(VERSION 3.8)
project(uav_hardware_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# tof_frame_relay — re-tags frame_id on sipeed_tof_ms_a010 outputs
# (upstream driver hardcodes frame_id="tof"; we need tof_<N>_link per instance)
add_executable(tof_frame_relay src/tof_frame_relay.cpp)
ament_target_dependencies(tof_frame_relay rclcpp sensor_msgs)

install(TARGETS
    tof_frame_relay
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    config
    udev
    scripts
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
    scripts/discover_maixsense.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
