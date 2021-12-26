icv_set_module_vars(RadarConti)

add_library(icvRadarConti SHARED RadarContiLong.cxx RadarContiSide.cxx)
# TODO: How to reduce the module library size?
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

target_link_libraries(icvRadarConti icvCore  icvNet ${Boost_LIBRARIES})

icv_set_solution_folder(icvRadarConti Sensors)

icv_install_module(RadarConti)
