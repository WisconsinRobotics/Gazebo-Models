cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(InsomniaPlugin SHARED plugins/InsomniaPlugin.cpp utils/UdpSocket.cpp)
target_link_libraries(InsomniaPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(AscentPlugin SHARED plugins/AscentPlugin.cpp utils/UdpSocket.cpp)
target_link_libraries(AscentPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
