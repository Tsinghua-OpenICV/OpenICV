
icv_set_module_vars(Mobileye)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )


add_library(icvMobileye SHARED icvMobileye.cxx )

TARGET_LINK_LIBRARIES( icvMobileye icvCore icvNet ${Boost_LIBRARIES})
icv_set_solution_folder(icvMobileye Sensors)

icv_install_module(Mobileye)

