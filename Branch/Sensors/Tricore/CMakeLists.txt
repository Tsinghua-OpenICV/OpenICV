
icv_set_module_vars(Tricore)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

add_library(icvTricore SHARED Tricore.cxx structure_all_objs.h)

TARGET_LINK_LIBRARIES( icvTricore icvCore icvNet ${Boost_LIBRARIES})
icv_set_solution_folder(icvTricore Sensors)
icv_install_module(Tricore)


