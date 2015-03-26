cmake_minimum_required(VERSION 2.8)

ADD_DEFINITIONS(
  -Wall
  -g
  -O2
  -std=c++11
)

project(CtrlPlanningSE2_PRM)

find_package(OMPL REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
add_definitions(${OMPL_DEFINITIONS})

add_executable (CtrlPlanningSE2_PRM CtrlPlanningSE2_PRM.cpp)
target_link_libraries (CtrlPlanningSE2_PRM ${OMPL_LIBRARIES})