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

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -O3 -Wl,--no-as-needed")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -O3 -Wl,--no-as-needed")

add_message_files(
  FILES
  GPS.msg
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

SET(INERTIALSENSE_SDK_DIR lib/InertialSenseSDK)

set(INERTIALSENSE_SDK_SRC
  ${INERTIALSENSE_SDK_DIR}/src/com_manager.c
  ${INERTIALSENSE_SDK_DIR}/src/data_sets.c
  ${INERTIALSENSE_SDK_DIR}/src/DataChunk.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DataChunkSorted.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DataCSV.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DataKML.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DeviceLog.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DeviceLogCSV.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DeviceLogKML.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DeviceLogSerial.cpp
  ${INERTIALSENSE_SDK_DIR}/src/DeviceLogSorted.cpp
  ${INERTIALSENSE_SDK_DIR}/src/InertialSense.cpp
  ${INERTIALSENSE_SDK_DIR}/src/inertialSenseBootLoader.c
  ${INERTIALSENSE_SDK_DIR}/src/ISDataMappings.cpp
  ${INERTIALSENSE_SDK_DIR}/src/ISDisplay.cpp
  ${INERTIALSENSE_SDK_DIR}/src/ISLogger.cpp
  ${INERTIALSENSE_SDK_DIR}/src/ISMatrix.c
  ${INERTIALSENSE_SDK_DIR}/src/ISPose.c
  ${INERTIALSENSE_SDK_DIR}/src/ISTcpClient.cpp
  ${INERTIALSENSE_SDK_DIR}/src/ISUtilities.cpp
  ${INERTIALSENSE_SDK_DIR}/src/linked_list.c
  ${INERTIALSENSE_SDK_DIR}/src/Rtcm3Reader.cpp
  ${INERTIALSENSE_SDK_DIR}/src/serialPort.c
  ${INERTIALSENSE_SDK_DIR}/src/serialPortPlatform.c
  ${INERTIALSENSE_SDK_DIR}/src/time_conversion.c
  ${INERTIALSENSE_SDK_DIR}/src/tinystr.cpp
  ${INERTIALSENSE_SDK_DIR}/src/tinyxml.cpp
  ${INERTIALSENSE_SDK_DIR}/src/tinyxmlerror.cpp
  ${INERTIALSENSE_SDK_DIR}/src/tinyxmlparser.cpp
  ${INERTIALSENSE_SDK_DIR}/src/UbloxReader.cpp
)

include_directories(include
  ${INERTIALSENSE_SDK_DIR}/src
  ${catkin_INCLUDE_DIRS}
)


add_executable(inertial_sense_node
	src/inertial_sense.cpp
	include/inertial_sense.h
        ${INERTIALSENSE_SDK_SRC}
)
target_link_libraries(inertial_sense_node ${catkin_LIBRARIES})
add_dependencies(inertial_sense_node inertial_sense_generate_messages_cpp)



