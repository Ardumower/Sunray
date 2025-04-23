#
# Compiles Sunray ROS node
#
#
#  cmake .. 
#  cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..
#
#

cmake_minimum_required(VERSION 3.7)


project(sunray_node)


set(CATKIN_DEPENDS
  roscpp

  pcl_ros
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs

  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  mcl_3dl_msgs  
  std_srvs
  visualization_msgs
)



## add_definitions(-DNO_MAIN)
## add_definitions(-DBLE_PROTOCOL_DABBLE)
## add_definitions(-DBLE_PROTOCOL_SUNRAY)
## add_definitions(-DCUSTOM_SETTINGS -DINCLUDE_GAMEPAD_MODULE)


## enable_language(C)
## enable_language(CXX)


## FIND_PACKAGE(PythonLibs)
##  http://wiki.ros.org/ROS/Tutorials/CreatingPackage
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
##find_package(pybind11 REQUIRED CONFIG)
## find_package(Eigen3 REQUIRED CONFIG)


catkin_package()

# Specify locations of header files.

message("[DEBUG] CATKIN_INCLUDES: ${catkin_INCLUDE_DIRS}")


include_directories(${catkin_INCLUDE_DIRS})

## include_directories(${pybind11_INCLUDE_DIRS})
## INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
## INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})


SET(FIRMWARE_PATH ${CMAKE_SOURCE_DIR}/../../sunray)

set(CMAKE_CXX_STANDARD 14)
SET(EXCLUDE_REGEX "agcm4|due|esp")


if(WIN32)
	message(INFO "This package is not tested on Windows. Feel free to report if it works!")
endif()


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -lbluetooth")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -x c")
SET(CMAKE_BUILD_TYPE Debug)


message("CMAKE_SOURCE_DIR ${CMAKE_SOURCE_DIR}")
message("Using firmware path ${FIRMWARE_PATH}")
message("CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR}")


file(GLOB_RECURSE sunray_cpp ${FIRMWARE_PATH}/**.cpp)
list(FILTER sunray_cpp EXCLUDE REGEX ${EXCLUDE_REGEX})
message("---sunray_cpp---")
foreach (filename ${sunray_cpp})
    message(${filename})
endforeach()

file(GLOB_RECURSE sunray_c ${FIRMWARE_PATH}/**.c)
list(FILTER sunray_c EXCLUDE REGEX ${EXCLUDE_REGEX})
message("---sunray_c---")
foreach (filename ${sunray_c})
    message(${filename})
endforeach()


message("---pi_sources---")
file(GLOB_RECURSE pi_sources ${FIRMWARE_PATH}/../alfred/src/**.cpp ${FIRMWARE_PATH}/../alfred/src/**.c)
##list(FILTER pi_sources EXCLUDE REGEX "wiring_main.cpp")
foreach (filename ${pi_sources})
    message(${filename})
endforeach()


message("--copy CONFIG_FILE to sunray source--")
if (NOT DEFINED CONFIG_FILE)
    set(CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../alfred/config_fuxtec_ros.h)
endif()
message("CONFIG_FILE: ${CONFIG_FILE}  (to set config file, use: cmake -D CONFIG_FILE=myconfig.h ..)")
if(NOT EXISTS "${CONFIG_FILE}")
    message( FATAL_ERROR "CONFIG_FILE does not exist: ${CONFIG_FILE}")
endif()

configure_file(${CONFIG_FILE} ${FIRMWARE_PATH}/config.h COPYONLY)


set_source_files_properties(src/sunray_node.cpp PROPERTIES LANGUAGE CXX)
set_source_files_properties(src/sunray_node.cpp PROPERTIES COMPILE_FLAGS "-x c++")


## ADD_LIBRARY( sunray_node SHARED ${pi_sources} ${sunray_cpp} ${sunray_c} )
add_executable(sunray_node ${pi_sources} ${sunray_cpp} ${sunray_c} src/sunray_node.cpp)

target_include_directories(sunray_node PRIVATE ${FIRMWARE_PATH} ${FIRMWARE_PATH}/../alfred/src)
target_link_libraries(sunray_node ${catkin_LIBRARIES})

##set_target_properties(sunray_node PROPERTIES PREFIX "")
##set_target_properties(sunray_node PROPERTIES OUTPUT_NAME "sunray_node")

