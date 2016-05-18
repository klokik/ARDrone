
add_executable(marker_location marker_location.cc)
add_executable(camera_location camera_location.cc)
add_executable(camera_location_api camera_location_api.cc)

target_link_libraries(marker_location ${ARDRONE_LIBS})
target_link_libraries(camera_location ${ARDRONE_LIBS})
target_link_libraries(camera_location_api ${ARDRONE_LIBS})
