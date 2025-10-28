cmake_minimum_required(VERSION 3.16)
project(signal_generator_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hid_descriptor_generator REQUIRED)

# Include the HID generator macro
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

# Generate HID descriptor and ROS2 files from YAML schema
hid_generate(
  SCHEMA_FILE "schema/signal_generator.yaml"
)

# Install schema, config, and launch files
install(
  DIRECTORY schema config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
