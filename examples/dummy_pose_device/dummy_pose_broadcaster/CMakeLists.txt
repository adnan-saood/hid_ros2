cmake_minimum_required(VERSION 3.16)
project(dummy_pose_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(dummy_pose_broadcaster SHARED
  src/dummy_pose_broadcaster.cpp
)

target_compile_features(dummy_pose_broadcaster PUBLIC cxx_std_17)
target_include_directories(dummy_pose_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dummy_pose_broadcaster>
)
ament_target_dependencies(dummy_pose_broadcaster PUBLIC
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dummy_pose_broadcaster PRIVATE "DUMMY_POSE_BROADCASTER_BUILDING_DLL")

# Export plugin description file
pluginlib_export_plugin_description_file(controller_interface dummy_pose_broadcaster.xml)

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

install(TARGETS dummy_pose_broadcaster
  EXPORT export_dummy_pose_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/dummy_pose_broadcaster
)

install(FILES dummy_pose_broadcaster.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_dummy_pose_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
