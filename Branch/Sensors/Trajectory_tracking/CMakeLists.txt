icv_set_module_vars(LaneTracking)

add_library(icvLaneTracking SHARED ACC_control.cxx lane_track.cxx)
# TODO: How to reduce the module library size?


target_link_libraries(icvLaneTracking icvCore  icvNet ${Boost_LIBRARIES})

icv_set_solution_folder(icvLaneTracking Sensors)

icv_install_module(LaneTracking)
