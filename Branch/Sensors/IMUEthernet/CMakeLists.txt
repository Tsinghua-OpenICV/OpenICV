
icv_set_module_vars(IMUEthernet)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )


add_library(icvIMUEthernet SHARED IMUEthernet.cxx)

TARGET_LINK_LIBRARIES( icvIMUEthernet icvCore icvNet ${Boost_LIBRARIES})
icv_set_solution_folder(icvIMUEthernet Sensors)
icv_install_module(IMUEthernet)


