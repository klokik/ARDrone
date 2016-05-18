cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

message("Running CMake")

if (NOT UNIX)
	message(FATAL_ERROR "Not Unix")
endif()

project(ARDrone)

message("Searching stuff...")
find_package(gazebo REQUIRED)
find_package(ignition-math2 QUIET REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

include_directories(${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
link_directories(${GAZEBO_LIBRARY_DIRS} ${IGNITION-MATH_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

set(ARDRONE_LIBS
    ${GAZEBO_LIBRARIES}
    ${IGNITION-MATH_LIBRARIES}
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${ARUCO_LIBS})

add_subdirectory(src)
