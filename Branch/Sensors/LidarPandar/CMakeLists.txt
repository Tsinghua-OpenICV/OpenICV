icv_set_module_vars(LidarPandar)
add_compile_options(-std=c++11)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
file(GLOB_RECURSE ${MODULE_HDR_NAME} *.h)
find_package(PCL REQUIRED ) 
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}} LidarPandar.cxx)

#message("header files =  ${${MODULE_HDR_NAME}} ")
# TODO: How to reduce the module library size?
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
#message("pcl=${PCL_INCLUDE_DIRS}")
target_link_libraries(${MODULE_TARGET_NAME} icvCore ${PCL_LIBRARIES} ${Boost_LIBRARIES} hesaiLidarSDK )
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

icv_set_solution_folder(${MODULE_TARGET_NAME} Sensors)

icv_install_module(LidarPandar)
