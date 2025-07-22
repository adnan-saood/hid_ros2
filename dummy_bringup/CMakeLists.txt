cmake_minimum_required(VERSION 3.8)
project(dummy_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hid_descriptor_generator REQUIRED)

# Include the HID generator macro
include("${hid_descriptor_generator_DIR}/hid_generate.cmake")

# Generate HID descriptor and ROS2 files from YAML schema
hid_generate(
  SCHEMA_FILE "schema/hid_device.yaml"
)

# Install schema file
install(
  DIRECTORY schema
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
