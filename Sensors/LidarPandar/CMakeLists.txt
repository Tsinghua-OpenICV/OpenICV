icv_set_module_vars(LidarPandar)

   file(GLOB_RECURSE ${MODULE_HDR_NAME} *.h)
find_package(PCL 1.2 REQUIRED ) 
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}} structpandar.hxx)

#message("header files =  ${${MODULE_HDR_NAME}} ")
# TODO: How to reduce the module library size?
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

#message("pcl=${PCL_INCLUDE_DIRS}")
target_link_libraries(${MODULE_TARGET_NAME} icvCore ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

icv_set_solution_folder(${MODULE_TARGET_NAME} Sensors)

icv_install_module(LidarPandar)
