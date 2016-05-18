
add_library(model_force_torque SHARED model_force_torque.cc)
target_link_libraries(model_force_torque ${GAZEBO_LIBRARIES})
