icv_set_module_vars(LidarHDL)
find_package(PCL 1.2 REQUIRED ) 



file(GLOB ${MODULE_HDR_NAME}
    ../../../Include/OpenICV/Extensions/PCL/*.h
    ../../../Include/OpenICV/Extensions/PCL/*.hxx)
#list(APPEND ${MODULE_HDR_NAME} /../../../Include/OpenICV/icvPCL.h)
list(APPEND ${MODULE_HDR_NAME} /home/yining/OpenICV/Branch/Include/OpenICV/icvPCL.h)


add_library(icvLidarHDL SHARED LidarHDL.cxx)
# TODO: How to reduce the module library size?
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
#message("pcl=${PCL_INCLUDE_DIRS}")
target_link_libraries(icvLidarHDL icvCore ${PCL_LIBRARIES} ${Boost_LIBRARIES} )

icv_set_solution_folder(icvLidarHDL Sensors)

icv_install_module(LidarHDL)
