
icv_set_module_vars(IMUOxTS)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

add_library(icvIMUOxTS SHARED IMUOxTS.cxx)

TARGET_LINK_LIBRARIES( icvIMUOxTS icvCore icvNet ${Boost_LIBRARIES})
icv_set_solution_folder(icvIMUOxTS Sensors)
icv_install_module(IMUOxTS)


