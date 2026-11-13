cmake_minimum_required(VERSION 3.5.1)
project(jo_description)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

# Lets gz-sim resolve package://jo_description/... mesh URIs
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")


ament_package()
