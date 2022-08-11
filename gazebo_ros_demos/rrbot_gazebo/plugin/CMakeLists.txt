cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# add_library(move_soup_can SHARED move_soup_can.cc)
# target_link_libraries(move_soup_can ${GAZEBO_LIBRARIES})

add_library(circle_soup_can SHARED circle_soup_can.cc)
target_link_libraries(circle_soup_can ${GAZEBO_LIBRARIES})