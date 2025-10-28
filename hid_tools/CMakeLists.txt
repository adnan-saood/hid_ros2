cmake_minimum_required(VERSION 3.8)
project(hid_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts as executables
install(PROGRAMS
  scripts/validate_schema
  scripts/inspect_device
  scripts/generate_hid_report_device
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package module
install(DIRECTORY hid_tools/
  DESTINATION lib/python3/dist-packages/hid_tools
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
