cmake_minimum_required(VERSION 2.8.7)

project(ros_rs_sensor_board)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

#########################################################
# INCLUDES
set(ULTRASND_INC_PATH
    ${${PROJECT_NAME}_SOURCE_DIR}/src)

include_directories(${ULTRASND_INC_PATH})
#########################################################

#########################################################
# SOURCES
file(GLOB_RECURSE ULTRASND_SRC *.cpp)
# HEADERS
file(GLOB_RECURSE ULTRASND_HDR *.h)
#########################################################

#########################################################
# MSG FILES
# Adding MSG files to project source, without compiling them
file(GLOB_RECURSE MSG . *.msg)
set_source_files_properties(${MSG} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
# SRV FILES
# Adding SRV files to project source, without compiling them
file(GLOB_RECURSE SRV . *.srv)
set_source_files_properties(${SRV} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
# LAUNCH FILES
# Adding LAUNCH files to project source, without compiling them
file(GLOB_RECURSE LAUNCH . *.launch)
set_source_files_properties(${LAUNCH} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  rs_ultrasnd_bump_ranges.msg
)

add_service_files(
  FILES
  rs_camera_light.srv
  rs_ledbar.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    sensor_msgs
    geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
#########################################################


#########################################################
# EXECUTABLE
add_executable(
    rs_sensor_board_node
    ${ULTRASND_HDR}
    ${ULTRASND_SRC}
    ${MSG}
    ${SRV}
    ${LAUNCH}
)

target_link_libraries(
    rs_sensor_board_node
    ${catkin_LIBRARIES}
)
#########################################################
