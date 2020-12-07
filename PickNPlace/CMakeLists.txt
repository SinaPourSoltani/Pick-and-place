cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PickNPlace)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RW_ROOT $ENV{RW_ROOT})
set(RobWork_DIR ${RW_ROOT}/cmake) 
set(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})

set(RWS_ROOT $ENV{RWS_ROOT})
set(RobWorkStudio_DIR ${RWS_ROOT}/cmake) 
set(CMAKE_MODULE_PATH ${RWS_ROOT}/build ${CMAKE_MODULE_PATH})

option (SS "Use Sparse Stereo" OFF)
option (DS "Use Depth Sensor" OFF)

set(D3D false) 
set(D2D false)

# OpenCV
if(SS)
	message("* * * * * * * * * * * * * * SPARSE STEREO * * * * * * * * * * * * * *")
	find_package(OpenCV REQUIRED)
	set(D2D true)
endif()

# PCL
if(DS)
	message("* * * * * * * * * * * * * * DEPTH SENSOR * * * * * * * * * * * * * *")
	find_package(PCL 1.7 REQUIRED)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	set(D3D true) 
endif()

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

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-DD2D=${D2D})
add_definitions(-DD3D=${D3D})

add_executable(${PROJECT_NAME} pickAndPlace.cpp)

if(SS)
	message("* * * * * * * * * * * * * * SPARSE STEREO * * * * * * * * * * * * * *")
	target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()


if(DS)
	message("* * * * * * * * * * * * * * DEPTH SENSOR * * * * * * * * * * * * * *")
	target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
endif()


target_link_libraries(${PROJECT_NAME} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})