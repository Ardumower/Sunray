#
# Compiles Sunray firmware for Raspberry pi
#
#  cmake .. 
#  cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..
#

cmake_minimum_required(VERSION 3.7)

project(sunray)

SET(FIRMWARE_PATH ${CMAKE_SOURCE_DIR}/../sunray)

set(CMAKE_CXX_STANDARD 14)
SET(EXCLUDE_REGEX "agcm4|due|esp")


if(WIN32)
	message(INFO "This package is not tested on Windows. Feel free to report if it works!")
endif()

# find_package(CUDA REQUIRED)
# find_package(OpenCV REQUIRED)

# find_library(LIBIW_LIB iw)
# find_library(LIBNL_LIB nl-3)
# find_library(LIBNL_GENL_LIB nl-genl-3)
# SET(LIBNL_LIBS ${LIBNL_LIB} ${LIBNL_GENL_LIB})

# find_path (LIBNL_INCLUDE_DIR netlink/netlink.h libnl3)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -lbluetooth")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -x c")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -x c -I /usr/include/libnl3/")
SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

# set(DL_LIB_DIR "/lib/arm-linux-gnueabihf") 

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
file(GLOB_RECURSE pi_sources src/**.cpp src/**.c)
## list(FILTER pi_sources EXCLUDE REGEX "wiring_main.cpp")
foreach (filename ${pi_sources})
    message(${filename})
endforeach()

message("--copy CONFIG_FILE to sunray source--")
if (NOT DEFINED CONFIG_FILE)
    set(CONFIG_FILE ${CMAKE_SOURCE_DIR}/config.h)
endif()
message("CONFIG_FILE: ${CONFIG_FILE}  (to set config file, use: cmake -D CONFIG_FILE=myconfig.h ..)")
if(NOT EXISTS "${CONFIG_FILE}")
    message( FATAL_ERROR "CONFIG_FILE does not exist: ${CONFIG_FILE}")
endif()

configure_file(${CONFIG_FILE} ${FIRMWARE_PATH}/config.h COPYONLY)

message("main file ${FIRMWARE_PATH}/sunray.ino")

## set_source_files_properties({sunray_c} PROPERTIES LANGUAGE C)

set_source_files_properties(test/test.ino PROPERTIES LANGUAGE CXX)
set_source_files_properties(test/test.ino PROPERTIES COMPILE_FLAGS "-x c++")
set_source_files_properties(${FIRMWARE_PATH}/sunray.ino PROPERTIES LANGUAGE CXX)
set_source_files_properties(${FIRMWARE_PATH}/sunray.ino PROPERTIES COMPILE_FLAGS "-x c++")

## if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
##  add_definitions("-x c++")
## endif()

add_executable(sunray ${pi_sources} ${sunray_cpp} ${sunray_c} ${FIRMWARE_PATH}/sunray.ino)
target_include_directories(sunray PRIVATE src ${FIRMWARE_PATH}/src)
# target_link_libraries(sunray "${CMAKE_SOURCE_DIR}/lib/libarduino_${CMAKE_SYSTEM_PROCESSOR}.a")

# target_include_directories(sunray PUBLIC ${LIBNL_INCLUDE_DIR})
# target_link_libraries(sunray ${LIBIW_LIB} ${LIBNL_LIB} ${LIBNL_GENL_LIB})

## set_property(TARGET sunray PROPERTY C_STANDARD 90)
## set_target_properties(sunray PROPERTIES LINKER_LANGUAGE CXX)


#ADD_LIBRARY( arduino_linux STATIC ${pi_sources} )
#target_include_directories(arduino_linux PRIVATE src)

