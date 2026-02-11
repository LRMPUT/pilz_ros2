cmake_minimum_required(VERSION 3.0.2)
project(prbt_hardware_support)

find_package(catkin REQUIRED COMPONENTS
  # canopen_chain_node
  message_filters
  message_generation
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  dynamic_reconfigure
)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Werror)
add_definitions(-std=c++11)
add_definitions(-Wconversion) # At least this line needs to be below find_package to keep
                              #  the flag away from the gmock build
add_definitions(-Wpedantic)

# message generation
add_message_files(
  FILES
  BrakeTestErrorCodes.msg
  ModbusMsgInStamped.msg
  ModbusRegisterBlock.msg
)

# action generation
add_service_files(
  FILES
  BrakeTest.srv
  SendBrakeTestResult.srv
  WriteModbusRegister.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/FakeSpeedOverride.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime pilz_msgs roscpp std_msgs std_srvs sensor_msgs
)

################
## Clang tidy ##
################
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()


###########
## Build ##
###########

include_directories(
  include
  test/include
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}) #Must be declared SYSTEM to avoid Warnings from ros system includes
include_directories(${pilz_utils_INCLUDE_DIRS})



add_executable(fake_speed_override_node
  src/fake_speed_override_node.cpp
)
add_dependencies(fake_speed_override_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(fake_speed_override_node ${catkin_LIBRARIES})


#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(TARGETS
  fake_speed_override_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()


  #--- FakeSpeedOverride unit test ---
  add_rostest_gtest(unittest_fake_speed_override
      test/unit_tests/unittest_fake_speed_override.test
      test/unit_tests/unittest_fake_speed_override.cpp
  )
  target_link_libraries(unittest_fake_speed_override
    ${catkin_LIBRARIES}
  )
  add_dependencies(unittest_fake_speed_override
    fake_speed_override_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
  #----------------------------------------

  #--- DisableSpeedMonitoring unit test ---
  add_rostest_gmock(unittest_disable_speed_monitoring
      test/unit_tests/unittest_disable_speed_monitoring.test
      test/unit_tests/unittest_disable_speed_monitoring.cpp
  )
  target_link_libraries(unittest_disable_speed_monitoring
    ${catkin_LIBRARIES}
  )
  add_dependencies(unittest_disable_speed_monitoring
    fake_speed_override_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
  #----------------------------------------
  endif()