cmake_minimum_required(VERSION 2.8.3)
project(inertial_sense)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
)
find_package(Threads)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed")

add_message_files(
  FILES
  SatInfo.msg
  GPS.msg
  GPSInfo.msg
  DThetaVel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

SET(IS_SP_DIR lib/inertialsense_serial_protocol)

set(IS_SRC
    ${IS_SP_DIR}/data_sets.c
    ${IS_SP_DIR}/data_sets.h
    ${IS_SP_DIR}/ISComm.c
    ${IS_SP_DIR}/ISComm.h
    ${IS_SP_DIR}/ISConstants.h
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${IS_SP_DIR}
)


add_executable(inertial_sense_node
        src/inertial_sense.cpp
        include/inertial_sense.h
        src/serial.cpp
        include/serial.h
        ${IS_SRC}
)
target_link_libraries(inertial_sense_node ${catkin_LIBRARIES})
add_dependencies(inertial_sense_node inertial_sense_generate_messages_cpp)
