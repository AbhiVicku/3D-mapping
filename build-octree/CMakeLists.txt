cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(octo_test)

find_package(octomap REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
add_definitions(${OCTOMAP_DEFINITIONS})

add_executable (output octree_test.cpp)
target_link_libraries (output ${OCTOMAP_LIBRARIES})
