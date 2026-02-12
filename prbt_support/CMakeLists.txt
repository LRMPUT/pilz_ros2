cmake_minimum_required(VERSION 3.8)
project(prbt_support)

find_package(ament_cmake REQUIRED)

# install(DIRECTORY urdf meshes config launch
install(DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
