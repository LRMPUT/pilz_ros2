cmake_minimum_required(VERSION 3.8)
project(pilz_tutorial)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/taskBK.py
  scripts/testApp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
