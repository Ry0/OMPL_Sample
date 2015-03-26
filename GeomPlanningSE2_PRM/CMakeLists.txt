cmake_minimum_required(VERSION 2.8)

ADD_DEFINITIONS(
  -Wall
  -g
  -O2
  -std=c++11
)

project(GeomPlanningSE2_PRM)

find_package(OMPL REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
add_definitions(${OMPL_DEFINITIONS})

FIND_PACKAGE(Boost COMPONENTS system REQUIRED)

add_executable (GeomPlanningSE2_PRM GeomPlanningSE2_PRM.cpp)
target_link_libraries (GeomPlanningSE2_PRM ${Boost_LIBRARIES} ${OMPL_LIBRARIES})