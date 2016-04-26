cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)

find_package(gazebo REQUIRED)
find_package(ignition-math2 QUIET REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# ArUco FIXME: integrate into project
# set(ARUCO_LIBS ArUco)
# add_library(${ARUCO_LIBS} SHARED IMPORTED)
# set(ARUCO_ROOT_DIR /home/klokik/Software/SDK/aruco/aruco-1.3.0)
# set(ARUCO_LIBRARY_PATH ${ARUCO_ROOT_DIR}/build/src/libaruco.so.1.3.0)
# set(ARUCO_INCLUDE_DIRS ${ARUCO_ROOT_DIR}/src)
# set_property(TARGET ArUco PROPERTY IMPORTED_LOCATION ${ARUCO_LIBRARY_PATH})

#set (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${IGNITION-MATH_CXX_FLAGS})

include_directories(${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${IGNITION-MATH_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(ALL_LIBS
    ${GAZEBO_LIBRARIES}
    ${IGNITION-MATH_LIBRARIES}
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${ARUCO_LIBS})

add_executable(marker_location marker_location.cc)
add_executable(camera_location camera_location.cc)

target_link_libraries(marker_location ${ALL_LIBS})
target_link_libraries(camera_location ${ALL_LIBS})
