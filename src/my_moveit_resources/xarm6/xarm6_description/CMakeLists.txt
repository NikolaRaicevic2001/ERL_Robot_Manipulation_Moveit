cmake_minimum_required(VERSION 3.10.2)
project(my_moveit_xarm6_description)

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)
