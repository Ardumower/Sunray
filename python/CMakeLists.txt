#
# Compiles Sunray Python wrapper for Raspberry pi
#
#  install pybind11 with:  
#     sudo apt install pybind11-dev 
#     pip install "pybind11[global]"
#
#  cmake .. 
#  cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..
#
#
# to run the python interpreter as sudo:
#  1. which python  =>    /home/alex/miniconda3/envs/py38/bin/python
#  2. sudo /home/alex/miniconda3/envs/py38/bin/python test.py


cmake_minimum_required(VERSION 3.7)


project(pysunray)

add_definitions(-DNO_MAIN)
add_definitions(-DBLE_PROTOCOL_DABBLE)
## add_definitions(-DBLE_PROTOCOL_SUNRAY)
## add_definitions(-DCUSTOM_SETTINGS -DINCLUDE_GAMEPAD_MODULE)


## enable_language(C)
## enable_language(CXX)


## FIND_PACKAGE(PythonLibs)
find_package(pybind11 REQUIRED CONFIG)
## find_package(Eigen3 REQUIRED CONFIG)

# checking property of the target
# get_target_property(inc_dir Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
# message("[DEBUG] inc_dir: ${inc_dir}")

# or checking the Eigen variable
message("[DEBUG] PYTHON_INCLUDE_PATH: ${PYTHON_INCLUDE_PATH}")
message("[DEBUG] PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")

message("[DEBUG] pybind11_INCLUDE_DIRS: ${pybind11_INCLUDE_DIRS}")

message("[DEBUG] EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")



include_directories(${pybind11_INCLUDE_DIRS})
## INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
## INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})


SET(FIRMWARE_PATH ${CMAKE_SOURCE_DIR}/../sunray)

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



file(GLOB_RECURSE sunray_cpp ${FIRMWARE_PATH}/**.cpp dabble/**.cpp)
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
file(GLOB_RECURSE pi_sources ../alfred/src/**.cpp ../alfred/src/**.c ../python/pysunray.cpp)
##list(FILTER pi_sources EXCLUDE REGEX "wiring_main.cpp")
foreach (filename ${pi_sources})
    message(${filename})
endforeach()


ADD_LIBRARY( pysunray SHARED ${pi_sources} ${sunray_cpp} ${sunray_c} )
## add_executable(pysunray ${pi_sources} src/wiring_main.cpp)

target_include_directories(pysunray PRIVATE ../alfred/src ../alfred/src ../sunray)
target_link_libraries(pysunray ${PYTHON_LIBRARIES})

set_target_properties(pysunray PROPERTIES PREFIX "")
set_target_properties(pysunray PROPERTIES OUTPUT_NAME "pysunray")

