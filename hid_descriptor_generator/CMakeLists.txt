cmake_minimum_required(VERSION 3.16)
project(hid_descriptor_generator)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install the CMake macro and the Python generator script
install(DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(PROGRAMS scripts/generate_hid_files.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
