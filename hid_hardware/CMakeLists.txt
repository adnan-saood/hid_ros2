cmake_minimum_required(VERSION 3.8)
project(hid_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-libusb)

add_library(
  hid_hardware
  SHARED
  src/hid_hardware.cpp
)
target_include_directories(
  hid_hardware
  PUBLIC
  include
  ${HIDAPI_INCLUDE_DIRS}
)
ament_target_dependencies(
  hid_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(
  hid_hardware
  ${HIDAPI_LIBRARIES}
)
# prevent pluginlib from using boost
target_compile_definitions(hid_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface hid_hardware.xml)

install(
  TARGETS
  hid_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  hid_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
