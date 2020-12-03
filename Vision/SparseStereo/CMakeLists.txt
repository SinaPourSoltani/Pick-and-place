cmake_minimum_required(VERSION 3.1)
# set (CMAKE_CXX_STANDARD 11)
project(sparseStereo)

# Opencv
find_package(OpenCV REQUIRED)

# PCL
# find_package(PCL 1.4 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

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




add_executable(${PROJECT_NAME} sparseStereo.cpp methods.hpp)
target_link_libraries(${PROJECT_NAME} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${OpenCV_LIBS} )
# ${PCL_LIBRARIES}
