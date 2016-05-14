cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mode_force_torque)

# Find packages

find_package(Gazebo REQUIRED)

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create libraries and executables

add_library(model_force_torque SHARED model_force_torque.cc)
target_link_libraries(model_force_torque ${GAZEBO_LIBRARIES})
