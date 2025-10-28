cmake_minimum_required(VERSION 3.16)
project(hid_generic_broadcaster)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(hid_generic_broadcaster SHARED
  src/hid_generic_broadcaster.cpp
)
target_compile_features(hid_generic_broadcaster PUBLIC cxx_std_17)
target_include_directories(hid_generic_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hid_generic_broadcaster>
)
ament_target_dependencies(hid_generic_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport
target_compile_definitions(hid_generic_broadcaster PRIVATE "HID_GENERIC_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface hid_generic_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/hid_generic_broadcaster
)
install(
  TARGETS hid_generic_broadcaster
  EXPORT export_hid_generic_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_hid_generic_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
