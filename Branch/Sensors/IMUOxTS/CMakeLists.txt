
icv_set_module_vars(IMUOxTS)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${OpenICV_ini_path}/Branch/Kernel/Include
                    ${OpenICV_ini_path}/Branch/Sensors/IMUOxTS/include)

add_library(icvIMUOxTS SHARED IMUOxTS.cxx utils.cxx)

link_directories(${OpenICV_ini_path}/Branch/Sensors/IMUOxTS/lib)


TARGET_LINK_LIBRARIES( icvIMUOxTS icvCore icvNet ${Boost_LIBRARIES})

target_link_libraries(icvIMUOxTS ${OpenICV_ini_path}/Branch/Sensors/IMUOxTS/lib/libgeos.so)

target_link_libraries(icvIMUOxTS proj)


icv_set_solution_folder(icvIMUOxTS Sensors)
icv_install_module(IMUOxTS)




