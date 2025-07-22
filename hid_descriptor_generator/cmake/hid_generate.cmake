# Copyright 2025 Adnan Saood
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# hid_generate.cmake
# Usage in your package CMakeLists.txt:
#   find_package(hid_descriptor_generator REQUIRED)
#   include("${hid_descriptor_generator_DIR}/hid_generate.cmake")
#   hid_generate(
#     SCHEMA_FILE  "schema/my_device.yaml"
#   )
#
# This will:
# 1. Generate a C header with HID descriptor
# 2. Generate URDF with proper state interfaces
# 3. Generate controller YAML
# 4. Export include directories and install files

function(hid_generate)
  set(options)
  set(oneValueArgs SCHEMA_FILE)
  set(multiValueArgs)
  cmake_parse_arguments(HID_GEN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT HID_GEN_SCHEMA_FILE)
    message(FATAL_ERROR "hid_generate: SCHEMA_FILE is required")
  endif()

  # Resolve schema file path
  if(NOT IS_ABSOLUTE "${HID_GEN_SCHEMA_FILE}")
    set(SCHEMA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${HID_GEN_SCHEMA_FILE}")
  else()
    set(SCHEMA_PATH "${HID_GEN_SCHEMA_FILE}")
  endif()

  if(NOT EXISTS "${SCHEMA_PATH}")
    message(FATAL_ERROR "hid_generate: schema file not found: ${SCHEMA_PATH}")
  endif()

  # Output directories
  set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
  set(GEN_INCLUDE "${GEN_DIR}/include")
  set(GEN_URDF "${GEN_DIR}/urdf")
  set(GEN_CONFIG "${GEN_DIR}/config")
  set(GEN_LAUNCH "${GEN_DIR}/launch")

  # Output files
  set(OUT_HEADER "${GEN_INCLUDE}/${PROJECT_NAME}/hid_descriptor.h")
  set(OUT_URDF "${GEN_URDF}/hid_robot.urdf.xacro")
  set(OUT_YAML "${GEN_CONFIG}/controllers.yaml")
  set(OUT_LAUNCH "${GEN_LAUNCH}/hid.launch.py")

  # Resolve generator script
  set(GEN_SCRIPT "${hid_descriptor_generator_DIR}/../../../lib/hid_descriptor_generator/generate_hid_files.py")
  get_filename_component(GEN_SCRIPT "${GEN_SCRIPT}" REALPATH)

  # Custom command to generate all files
  add_custom_command(
    OUTPUT ${OUT_HEADER} ${OUT_URDF} ${OUT_YAML} ${OUT_LAUNCH}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_INCLUDE}/${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_URDF}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_CONFIG}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_LAUNCH}
    COMMAND ${Python3_EXECUTABLE} ${GEN_SCRIPT}
            --schema ${SCHEMA_PATH}
            --output-dir ${GEN_DIR}
            --package-name ${PROJECT_NAME}
    DEPENDS ${SCHEMA_PATH} ${GEN_SCRIPT}
    COMMENT "Generating HID descriptor and ROS2 files from ${SCHEMA_PATH}"
    VERBATIM
  )

  # Create target
  add_custom_target(${PROJECT_NAME}_hid_generation ALL
    DEPENDS ${OUT_HEADER} ${OUT_URDF} ${OUT_YAML} ${OUT_LAUNCH}
  )

  # Export include directory for use in this package
  include_directories(${GEN_INCLUDE})

  # Install generated files
  install(DIRECTORY ${GEN_INCLUDE}/
    DESTINATION include
  )
  install(DIRECTORY ${GEN_URDF}/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
  install(DIRECTORY ${GEN_CONFIG}/
    DESTINATION share/${PROJECT_NAME}/config
  )
  install(DIRECTORY ${GEN_LAUNCH}/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  install(FILES ${SCHEMA_PATH}
    DESTINATION share/${PROJECT_NAME}/schema
  )

  # Export for downstream packages
  ament_export_include_directories(include)
endfunction()
