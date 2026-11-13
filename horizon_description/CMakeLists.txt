cmake_minimum_required(VERSION 3.10)
project(horizon_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The raw fusion2urdf-ros2 export of the Fusion design so101_mobile_base (after fix_export.py), as data:
# the URDF, its meshes, the launch files and the RViz setup. (Made ament_cmake by to_ament_cmake.py.)
install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
