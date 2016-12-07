cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

file(GLOB GAZEBO_LRF_SRC
    ${CMAKE_SOURCE_DIR}/Sensors/LRF/*.cpp
)

add_library(InsomniaPlugin SHARED InsomniaPlugin.cpp UdpSocket.cpp GAZEBO_LRF_SRC)
target_link_libraries(InsomniaPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
