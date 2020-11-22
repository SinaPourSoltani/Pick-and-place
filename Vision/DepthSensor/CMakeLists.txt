cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(DepthSensor)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RW_ROOT $ENV{RW_ROOT})
set(RobWork_DIR ${RW_ROOT}/cmake) 
set(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})

set(RWS_ROOT $ENV{RWS_ROOT})
set(RobWorkStudio_DIR ${RWS_ROOT}/cmake) 
set(CMAKE_MODULE_PATH ${RWS_ROOT}/build ${CMAKE_MODULE_PATH})

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# RobWork
find_package(RobWork REQUIRED)
include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})
add_definitions(${ROBWORK_DEFINITIONS})

# RobWorkStudio
find_package(RobWorkStudio REQUIRED)
include_directories(${ROBWORKSTUDIO_INCLUDE_DIRS})
link_directories(${ROBWORKSTUDIO_LIBRARY_DIRS})
add_definitions(${ROBWORKSTUDIO_DEFINITIONS})

add_executable(${PROJECT_NAME} depthSensor.cpp)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})